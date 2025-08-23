/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file imu.c
 * @brief Implement the generic IMU FreeRTOS task
 *
 * @author Gilles Henrard
 * @date 23/08/2025
 */
#include "imu.h"

#include <FreeRTOS.h>
#include <errorstack.h>
#include <math.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stddef.h>
#include <stdint.h>
#include <task.h>

#include "hardware_events.h"
#include "main.h"
#include "sensorfusion.h"

enum {
    kStackSize = 250U,      ///< Amount of words in the task stack
    kTaskLowPriority = 8U,  ///< FreeRTOS number for a low priority task
    kStartupTimeMS = 10U,   ///< Number of milliseconds to wait after a reset
    kTimeoutMS = 1000U,     ///< Max number of milliseconds to wait for a notification
    kMutexMS = 100U,        ///< Max number of milliseconds to wait for a mutex
};

/**
 * Enumeration of the IDs of the functions used by the BMI270 implementation
 */
typedef enum {
    kFunctionTask = 1,      ///< taskIMU() : Function running the IMU state machine
    kStateStartup = 2,      ///< stateStartup() : State in which the IMU is restarted
    kstateConfiguring = 3,  ///< stateConfiguring() : State in which the IMU is being initialised
    kStateMeasuring = 4,    ///< stateMeasuring() : State in which measurements are received from the BMI270
    kIgnoreSamples = 5,     ///< ignoreSamples() : Function in which samples are ignored after configuration
    kStateError = 6,        ///< State in which the IMU is in error mode
} FunctionCode;

// State machine functions
static void taskIMU(void* argument);
static ErrorCode stateStartup(void);
static ErrorCode stateConfiguring(void);
static ErrorCode ignoreSamples(void);
static ErrorCode stateMeasuring(void);

// constants
static const float kRadiansToDegreesTenths = 572.957795F;   ///< One radian in tenths of degrees (= 10 * (180°/PI))
static const float kDegreesTenthsToRadians = 0.001745329F;  ///< One tenth of degree in radians (= (180°/PI) / 10)

//state variables
static volatile TaskHandle_t task_handle = NULL;  ///< handle of the FreeRTOS task
static SemaphoreHandle_t angles_mutex = NULL;     ///< handle of the mutex used to protect angles measurements
static FunctionCode imu_state = kStateStartup;    ///< Current IMU state
static ErrorCode result;                          ///< Variable used to receive the functions' result codes
static volatile uint8_t task_notifiable = 0;      ///< Flag indicating whether the task is ready to treat notifications
static float angles_zeroing_rad[kNBaxis - 1] = {0, 0};  ///< Angles used to zero out the measurements
static MahonyContext filter_context = {.ki = kIntegralGain,
                                       .kp = kProportionalGain};  ///< Current Mahony filter context

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Handle an INT1 or INT2 GPIO interrupt from the IMU
 *
 * @param interrupt_pin Pin which triggered the interrupt
 */
void IMUinterruptTriggered(uint8_t interrupt_pin) {
    BaseType_t has_woken = 0;

    // vPortEnterCritical();

    //if INT1, notify the IMU task
    if (task_notifiable && (interrupt_pin == 1)) {
        vTaskNotifyGiveFromISR(task_handle, &has_woken);
    }

    // vPortExitCritical();

    portYIELD_FROM_ISR(has_woken);
}

/**
 * Create the FreeRTOS static task taking care of the IMU sensor
 */
void createIMUtask(void) {
    static StackType_t task_stack[kStackSize] = {0};     ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};                ///< Task state variables
    static StaticSemaphore_t measure_mutex_state = {0};  ///< ADC value mutex state variables

    //create the static task
    task_handle = xTaskCreateStatic(taskIMU, "IMU task", kStackSize, NULL, kTaskLowPriority, task_stack, &task_state);
    if (!task_handle) {
        Error_Handler();
    }

    //create a semaphore to protect mearurements
    angles_mutex = xSemaphoreCreateMutexStatic(&measure_mutex_state);
    if (!angles_mutex) {
        Error_Handler();
    }
}

/**
 * @brief Transpose a measurement to an angle in tenths of degrees with the Z axis
 *
 * @param axis Axis for which get the angle with the Z axis
 * @return Angle with the Z axis
 */
int16_t getAngleDegreesTenths(Axis axis) {
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdFALSE) {
        return 0;
    }
    float current_angle_rad = angleAlongAxis(&filter_context, axis);
    xSemaphoreGive(angles_mutex);

    float total = (current_angle_rad + angles_zeroing_rad[axis]);
    return (int16_t)(total * kRadiansToDegreesTenths);
}

/**
 * @brief Check if angle measurements have changed
 *
 * @retval 0 No new values available
 * @retval 1 New values are available
 */
uint8_t anglesChanged(void) {
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdFALSE) {
        return 0;
    }
    const float current_angle_rad = getAttitudeAngle(&filter_context);
    xSemaphoreGive(angles_mutex);

    //check if angle changed above minimum threshold (0.1° in any direction)
    static float previous_angle_rad = 0.0F;
    uint8_t changed = (fabsf(previous_angle_rad - current_angle_rad) > kDegreesTenthsToRadians);
    if (changed) {
        previous_angle_rad = current_angle_rad;
    }

    return (changed);
}

/**
 * @brief Set the measurements in relative mode and zero down the values
 */
void IMUzeroDown(void) {
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdFALSE) {
        return;
    }

    angles_zeroing_rad[kXaxis] = -angleAlongAxis(&filter_context, kXaxis);
    angles_zeroing_rad[kYaxis] = -angleAlongAxis(&filter_context, kYaxis);

    xSemaphoreGive(angles_mutex);
}

/**
 * @brief Set the measurements in absolute mode (no zeroing compensation)
 */
void IMUcancelZeroing(void) {
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdFALSE) {
        return;
    }

    angles_zeroing_rad[kXaxis] = 0.0F;
    angles_zeroing_rad[kYaxis] = 0.0F;

    xSemaphoreGive(angles_mutex);
}

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Run the IMU state machine
 * 
 * @param argument Unused
 */
static void taskIMU(void* argument) {
    (void)argument;
    while (1) {
        //run the state machine
        switch (imu_state) {
            case kStateStartup:
                result = stateStartup();
                imu_state = kstateConfiguring;
                break;

            case kstateConfiguring:
                result = stateConfiguring();
                imu_state = kStateMeasuring;
                break;

            case kStateMeasuring:
                result = stateMeasuring();
                break;

            case kStateError:
                Error_Handler();
                break;

            case kFunctionTask:
            case kIgnoreSamples:
            default:
                result = createErrorCode(kFunctionTask, 0, kErrorCritical);
                break;
        }

        //if an error occurred, get to the error state
        if (isError(result)) {
            imu_state = kStateError;
        }

        if (anglesChanged()) {
            triggerHardwareEvent(kEventXValue);
            triggerHardwareEvent(kEventYValue);
        }
    };
}

/**
 * State in which the IMU is resetted and the communication is checked
 * 
 * @retval 0 Success
 * @retval 1 Error while checking the communication
 * @retval 2 Error while requesting the soft reset
 */
static ErrorCode stateStartup(void) {
    task_notifiable = 0;

    vTaskDelay(pdMS_TO_TICKS(kStartupTimeMS));

    result = IMUcheckDeviceID();
    EXIT_ON_ERROR(result, kStateStartup, 1)

    result = IMUsoftReset();
    EXIT_ON_ERROR(result, kStateStartup, 2)
    vTaskDelay(pdMS_TO_TICKS(kStartupTimeMS));

    //attempt self-testing accelerometer and gyroscope
    uint8_t attempts = 5U;  // NOLINT(*-magic-numbers)
    do {
        result = selfTestAccelerometer();
        if (isError(result)) {
            continue;
        }

        result = selfTestGyroscope();
    } while (--attempts && isError(result));

    //if either self-test failed, error
    EXIT_ON_ERROR(result, kStateStartup, 3)

    return kSuccessCode;
}

/**
 * State in which the IMU is configured
 * 
 * @retval 0 Success
 * @retval 1 Error while initialising the IMU
 * @retval 2 Error while writing the IMU configuration
 * @retval 3 Error while ingoring samples after configuration
 */
static ErrorCode stateConfiguring(void) {
    //run the IMU initial configuration (e.g. power-after-reset procedure)
    result = IMUinitialise();
    EXIT_ON_ERROR(result, kstateConfiguring, 1)

    //write the config to the IMU
    result = IMUconfigure();
    EXIT_ON_ERROR(result, kstateConfiguring, 2)

    task_notifiable = 1;

    result = ignoreSamples();
    EXIT_ON_ERROR(result, kstateConfiguring, 3)

    //reset the Mahony filter's persistant variables
    resetMahonyFilter(&filter_context);
    IMUsetupTimebase(&filter_context);

    //TODO allow for samples to be dropped in the LSM6DSO, thus using task notifications
    return kSuccessCode;
}

/**
 * Ignore samples after configuration
 *
 * @retval 0 Success
 * @retval 1 Error while waiting for a sample 
 */
static ErrorCode ignoreSamples(void) {
    uint8_t remaining = getNbSamplesToIgnore();
    while (remaining) {
        //wait for a notification indicating a sample is ready
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kTimeoutMS)) == pdFALSE) {
            return createErrorCode(kIgnoreSamples, 1, kErrorError);
        }

        IMUsample dummy;
        IMUgetSample(&dummy);
        remaining--;
    }

    return kSuccessCode;
}

/**
 * State in which the IMU is measuring
 * 
 * @retval 0 Success
 * @retval 1 No interrupt received from the IMU in a timely manner
 * @retval 2 Error while getting the latest IMU sample
 */
static ErrorCode stateMeasuring(void) {
    //wait for measurements to be ready
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kTimeoutMS)) == pdFALSE) {
        return (createErrorCode(kStateMeasuring, 1, kErrorCritical));
    }

    //get the latest sample
    IMUsample sample;
    result = IMUgetSample(&sample);
    EXIT_ON_ERROR(result, kStateMeasuring, 2)

    //apply sensor fusion to the measurements
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        filter_context.dt.current_tick = sample.latest_tick;
        updateMahonyFilter(&filter_context, &sample);
        xSemaphoreGive(angles_mutex);
    }

    return kSuccessCode;
}
