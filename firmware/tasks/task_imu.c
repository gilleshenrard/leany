/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file task_imu.c
 * @brief Implement the generic IMU FreeRTOS task
 *
 * @author Gilles Henrard
 */
#include "task_imu.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <math.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stddef.h>
#include <stdint.h>
#include <task.h>

#include "errorstack.h"
#include "hardware_events.h"
#include "imu.h"
#include "main.h"
#include "orientation.inc"
#include "sensorfusion.h"
#include "task_serial.h"

enum {
    kStackSize = 350U,      ///< Amount of words in the task stack
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
    kSetOrientation = 6,    ///< setDisplayOrientation() : Function in which the screen orientation is set
    kgetOrientation = 7,    ///< getDisplayOrientation() : Function in which the screen orientation is retrieved
    kStateError = 8,        ///< State in which the IMU is in error mode
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
static volatile TaskHandle_t task_handle = NULL;      ///< handle of the FreeRTOS task
static SemaphoreHandle_t angles_mutex = NULL;         ///< handle of the mutex used to protect angles measurements
static SemaphoreHandle_t orientation_mutex = NULL;    ///< handle of the mutex used to protect display orientation
static Orientation current_orientation = kLandscape;  ///< Current screen orientation
static FunctionCode imu_state = kStateStartup;        ///< Current IMU state
static ErrorCode result;                              ///< Variable used to receive the functions' result codes
static volatile uint8_t task_notifiable = 0;  ///< Flag indicating whether the task is ready to treat notifications
static float angles_zeroing_rad[kNBaxis - 1] = {0, 0};  ///< Angles used to zero out the measurements
static uint8_t holding = 0;                             ///< Flag indicating whether the measurements are held
static MahonyContext filter_context = {
    .ki = kIntegralGain, .kp = kProportionalGain, .align_check_enabled = 1};  ///< Current Mahony filter context

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
    static StackType_t task_stack[kStackSize] = {0};         ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};                    ///< Task state variables
    static StaticSemaphore_t measure_mutex_state = {0};      ///< Angles value mutex state variables
    static StaticSemaphore_t orientatino_mutex_state = {0};  ///< orientation mutex state variables

    //create the static task
    task_handle = xTaskCreateStatic(taskIMU, "IMU task", kStackSize, NULL, kTaskLowPriority, task_stack, &task_state);
    configASSERT(task_handle);

    //create a semaphore to protect mearurements
    angles_mutex = xSemaphoreCreateMutexStatic(&measure_mutex_state);
    configASSERT(angles_mutex);

    //create a semaphore to protect orientation
    orientation_mutex = xSemaphoreCreateMutexStatic(&orientatino_mutex_state);
    configASSERT(orientation_mutex);

    logSerial(kErrorInfo, "IMU task and angles mutex created");
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
    (void)xSemaphoreGive(angles_mutex);

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
    (void)xSemaphoreGive(angles_mutex);

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

    (void)xSemaphoreGive(angles_mutex);
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

    (void)xSemaphoreGive(angles_mutex);
}

/**
 * Check if the IMU has been zeroed (relative mode)
 *
 * @retval 1 IMU is in relative mode
 * @retval 0 IMU is in absolute mode 
 */
uint8_t isIMUzeroed(void) {
    const float close_to_zero = 1e-3F;
    uint8_t current_zeroing = 0;

    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        current_zeroing =
            (((angles_zeroing_rad[kXaxis] > close_to_zero) || (angles_zeroing_rad[kXaxis] < -close_to_zero)) &&
             ((angles_zeroing_rad[kYaxis] > close_to_zero) || (angles_zeroing_rad[kYaxis] < -close_to_zero)));
        (void)xSemaphoreGive(angles_mutex);
    }

    return current_zeroing;
}

/**
 * Get the Mahony filter's Proportional value
 *
 * @return kP value
 */
float getIMU_KP(void) {
    float current_kp = 0.0F;

    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        current_kp = filter_context.kp;
        (void)xSemaphoreGive(angles_mutex);
    }

    return current_kp;
}

/**
 * Get the Mahony filter's Integral value
 *
 * @return kI value
 */
float getIMU_KI(void) {
    float current_ki = 0.0F;

    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        current_ki = filter_context.ki;
        (void)xSemaphoreGive(angles_mutex);
    }

    return current_ki;
}

/**
 * Set the Mahony filter's Integral value
 *
 * @param value Filter's integral value
 */
void setIMU_KI(float value) {
    if (value < 0.0F) {
        return;
    }

    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        filter_context.ki = value;
        resetMahonyFilter(&filter_context);
        (void)xSemaphoreGive(angles_mutex);
    }
}

/**
 * Set the Mahony filter's Proportional value
 *
 * @param value Filter's Proportional value
 */
void setIMU_KP(float value) {
    if (value < 0.0F) {
        return;
    }

    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        filter_context.kp = value;
        resetMahonyFilter(&filter_context);
        (void)xSemaphoreGive(angles_mutex);
    }
}

/**
 * Toggle the IMU measurements holding
 *
 * @return New Holding value
 */
uint8_t toggleIMU_hold(void) {
    uint8_t new_holding = 0;
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        holding = !holding;
        new_holding = holding;
        (void)xSemaphoreGive(angles_mutex);
    }

    return new_holding;
}

/**
 * Get the current measurements holding status
 *
 * @retval 1 IMU is in hold mode
 * @retval 0 IMU is running normally
 */
uint8_t isIMUmeasurementsHolding(void) {
    uint8_t current_holding = 0;

    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        current_holding = holding;
        (void)xSemaphoreGive(angles_mutex);
    }

    return current_holding;
}

/**
 * Set whether the estimated and actual vectors alignment is invalid and should reset the filter
 *
 * @param value 1 if check enabled, 0 otherwise
 */
void setIMUalignmentCheckEnabled(uint8_t value) {
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        filter_context.align_check_enabled = (value != 0);
        resetMahonyFilter(&filter_context);
        (void)xSemaphoreGive(angles_mutex);
    }
}

/**
 * Check whether the estimated and actual vectors alignment validity is checked
 *
 * @retval 1 Enabled
 * @retval 0 Disabled
 */
uint8_t isIMUalignmentCheckEnabled(void) {
    uint8_t enabled = 0;

    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        enabled = filter_context.align_check_enabled;
        (void)xSemaphoreGive(angles_mutex);
    }

    return enabled;
}

/**
 * Set the display orientation depending on the IMU orientation
 *
 * @param new_orientation New display orientation
 * @retval 0 Success
 * @retval 1 The mutex is not initialised yet
 * @retval 2 Invalid orientation provided
 * @retval 3 Unable to take the orientation mutex
 */
ErrorCode setDisplayOrientation(Orientation new_orientation) {
    if (!orientation_mutex) {
        return createErrorCode(kSetOrientation, 1, kErrorError);
    }

    if (new_orientation >= kNBorientations) {
        return createErrorCode(kSetOrientation, 2, kErrorError);
    }

    if (new_orientation == current_orientation) {
        return kSuccessCode;
    }

    if (xSemaphoreTake(orientation_mutex, kMutexMS) == pdFALSE) {
        return createErrorCode(kSetOrientation, 3, kErrorError);
    }

    current_orientation = new_orientation;
    (void)xSemaphoreGive(orientation_mutex);

    return kSuccessCode;
}

/**
 * Get the display orientation depending on the IMU orientation
 *
 * @param[out] orientation Display orientation
 * @retval 0 Success
 * @retval 1 The mutex is not initialised yet or orientation is NULL
 * @retval 2 Unable to take the orientation mutex
 */
ErrorCode getDisplayOrientation(Orientation* orientation) {
    if (!orientation_mutex || !orientation) {
        return createErrorCode(kgetOrientation, 1, kErrorError);
    }

    if (xSemaphoreTake(orientation_mutex, kMutexMS) == pdFALSE) {
        return createErrorCode(kgetOrientation, 2, kErrorError);
    }

    *orientation = current_orientation;
    (void)xSemaphoreGive(orientation_mutex);

    return kSuccessCode;
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
                if (anglesChanged()) {
                    triggerHardwareEvent(kEventAngle);
                }
                break;

            case kStateError:
                Error_Handler();
                break;

            case kFunctionTask:
            case kIgnoreSamples:
            case kSetOrientation:
            case kgetOrientation:
            default:
                result = createErrorCode(kFunctionTask, 0, kErrorCritical);
                break;
        }

        //if an error occurred, get to the error state
        if (isError(result)) {
            logSerial(result.level, "IMU Error %x", result.dword);
            imu_state = kStateError;
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
    logSerial(kErrorInfo, "IMU communication OK");

    result = IMUsoftReset();
    EXIT_ON_ERROR(result, kStateStartup, 2)
    vTaskDelay(pdMS_TO_TICKS(kStartupTimeMS));
    logSerial(kErrorInfo, "IMU soft-reset done");

    //attempt self-testing accelerometer and gyroscope
    uint8_t attempts = 5U;  // NOLINT(*-magic-numbers)
    do {
        result = selfTestAccelerometer();
        if (isError(result)) {
            continue;
        }
        logSerial(kErrorInfo, "Accemerometer self-test ok");

        result = selfTestGyroscope();
    } while (--attempts && isError(result));

    //if either self-test failed, error
    EXIT_ON_ERROR(result, kStateStartup, 3)

    logSerial(kErrorInfo, "Gyroscope self-test ok");
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

    //TODO allow for samples to be dropped in the LSM6DSO, thus using task notification
    logSerial(kErrorInfo, "IMU and Mahony filter initialisation OK");
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

    logSerial(kErrorInfo, "%u IMU samples ignored for synchronisation", getNbSamplesToIgnore());
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
        logSerial(kErrorDebug, "Mahony filter updated");
        (void)xSemaphoreGive(angles_mutex);
    }

    return kSuccessCode;
}
