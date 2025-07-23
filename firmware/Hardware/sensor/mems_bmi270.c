/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file mems_bmi270.c
 * @brief Implement the behavior of the BMI270 MEMS
 *
 * @author Gilles Henrard
 * @date 26/07/2025
 *
 * @details Datasheet : https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf 
 */

#include "mems_bmi270.h"

#include <FreeRTOS.h>
#include <bmi270.h>
#include <bmi2_defs.h>
#include <math.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_def.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_spi.h>
#include <task.h>

#include "errorstack.h"
#include "halspi.h"
#include "main.h"
#include "sensorfusion.h"
#include "st7735_initialisation.h"

enum {
    kStackSize = 180U,             ///< Amount of words in the task stack
    kTaskLowPriority = 8U,         ///< FreeRTOS number for a low priority task
    kStartupTimeMS = 2U,           ///< Number of milliseconds required after Power On Reset or soft reset
    kTimeoutMS = 1000U,            ///< Max number of milliseconds to wait for the device ID
    kConfigTimeoutMS = 20U,        ///< Max number of milliseconds after which the configuration should be ok
    kConfigFileSize = 8192U,       ///< Size of the config file in bytes
    kNBpositionRegisters = 16U,    ///< Number of registers holding Accelerometer and Gyroscope data
    kNBTemperatureRegisters = 2U,  ///< Number of registers holding Temperature data
    kTemperatureRefreshMS = 10U,   ///< Number of milliseconds between temperature value refreshes
    kConfigCheckAttempts = 3U,     ///< Maximum number of attempts to re-read a correct config file
};

/**
 * Enumeration of the IDs of the functions used by the BMI270 implementation
 */
typedef enum {
    kBMI270task = 1,                 ///< taskBMI270() : Function running the BMI270 state machine
    kBMI270readRegisters = 2,        ///< readRegisters() : Function reading several registers
    kBMI270writeRegisters = 3,       ///< burstWriteRegisters() : Function writing multiple registers
    kBMI270stateStartup = 4,         ///< stateStartup() : State in which the startup time is waited for
    kBMI270stateInitialising = 5,    ///< stateInitialising() : State in which the BMI270 is being initialised
    kBMI270stateError = 6,           ///< stateError() : Error state
    kBMI270checkCommunication = 7,   ///< checkSPICommunication() : Check the SPI communication with the BMI270
    kBMI270checkConfiguration = 8,   ///< checkConfigurationFile() : Check if written configuration file is correct
    kBMI270stateConfiguring = 9,     ///< stateConfiguring() : State in which the BMI270 is configured for the app.
    kBMI270stateMeasuring = 10,      ///< stateMeasuring() : State in which measurements are received from the BMI270
    kBMI270readTemperature = 11,     ///< readTemperature() : Function reading the BMI270 internal temp. registers
    kBMI270stateSelfTestAccel = 12,  ///< stateSelfTestingAccelerometer() : State in which the chip self-tests the acc.
    kBMI270stateSelfTestGyro = 13,   ///< stateSelfTestingGyro() : State in which the BMI270 self-tests the gyroscope
    kBMI270runAccelSelfTest = 14,    ///< runAccelerometerSelfTest() : Run the self-test command on the BMI270
    kBMI270configureRegisters = 15,  ///< configureRegisters() : Write an array of register number/value pairs
} FunctionCode;

/**
 * @brief Union regrouping 8-bits and 16-bits arrays
 * @details
 *  This allows reading all the accelerometer and gyroscope values at once, and convert them to 16 bit instantly
 */
typedef union {
    uint8_t registers8bits[kNBpositionRegisters];                   ///< 8-bits registers array
    int16_t values16bits[((uint8_t)(kNBpositionRegisters) >> 1U)];  ///< 16-bits values array
} RawValues;

_Static_assert(((uint8_t)kNBpositionRegisters & (kNBpositionRegisters - 1U)) == 0,
               "NB_POSITION_REGISTERS must be an even number");

typedef uint8_t BMI270register;  ///< type definition for BMI270 registers

// Read/Write bit value
static const BMI270register kBMIwrite = 0x00U;              ///< Address byte value for a write operation
static const BMI270register kBMIread = 0x80U;               ///< Address byte value for a read operation
static const float kRadiansToDegreesTenths = 572.957795F;   ///< One radian in tenths of degrees (= 10 * (180°/PI))
static const float kDegreesTenthsToRadians = 0.001745329F;  ///< One tenth of degree in radians (= (180°/PI) / 10)
const uint32_t kMaxSensorTimeTicks = 0xFFFFFFU;             ///< The maximum tick value before it wraps around to 0
const float kSensorTimeResolutionSeconds = 0.0000390625F;   ///< How many seconds a tick lasts

// Utility functions
static ErrorCode checkSPICommunication(void);
static ErrorCode writeConfigurationFile(void);
static ErrorCode checkConfigurationFile(void);
static ErrorCode readTemperature(float* temperature_celsius);
static ErrorCode runAccelerometerSelfTest(RawValues* values_read, uint8_t positive_sign);
static ErrorCode runGyroscopeSelfTest(void);
static inline uint32_t toTicks(const uint8_t registers[3]);
static void applyTemperatureDrift(float temperature_celcius, float accelerometer_g[3], float gyroscope_radps[3]);
static ErrorCode configureRegisters(const BMI270register registers[][2], uint8_t nb_registers);

// State machine functions
static void taskBMI270(void* argument);
static ErrorCode stateStartup(void);
static ErrorCode stateSelfTestingAccelerometer(void);
static ErrorCode stateSelfTestingGyroscope(void);
static ErrorCode stateInitialising(void);
static ErrorCode stateConfiguring(void);
static ErrorCode stateMeasuring(void);
static void stateError(void);

/**
 * Accelerometer nominal sensitivity ratio from LSB to G, at +-2G range and 25°C
 * @details Equation :
 * Value(G) = Value(LSB) / 16384(LSB/G)
 *
 * See datasheet p. 12, section "Sensitivity"
 */
static const float kNominalAccelLSBto2G = (1.0F / (float)BMI2_ACC_FOC_2G_REF);

/**
 * Accelerometer nominal sensitivity ratio from LSB to G, at +-16G range and 25°C
 * @details Equation :
 * Value(G) = Value(LSB) / 2048(LSB/G)
 *
 * See datasheet p. 12, section "Sensitivity"
 */
static const float kNominalAccelLSBto16G = (1.0F / (float)BMI2_ACC_FOC_16G_REF);

/**
 * Gyroscope nominal sensitivity ratio from LSB to rad/s, at +-125°/s range and 25°C
 * @details Equation :
 * Value(rad/s) = (Value(LSB) / 262,144(LSB/dps)) * (PI/180°)
 *
 * See datasheet p. 14, section "Sensitivity"
 */
static const float kNominalGyroLSBtoRadps = (1.0F / 262.144F) * 0.017453293F;

/**
 * Accelerometer temperature drift in [%/K], divided by 100 because percents
 */
static const float kAccelTemperatureDriftPercentPerKelvin = (0.004F / 100.0F);

/**
 * Gyroscope temperature drift in [%/K], divided by 100 because percents
 */
static const float kGyroTemperatureDriftPercentPerKelvin = (0.02F / 100.0F);

/**
 * Temperature in [°C] at which the internal temperature value read is 0x0000
 */
static const float kReferenceTemperatureCelsius = 23.0F;

//external variables
// NOLINTBEGIN(readability-identifier-naming)
extern const uint8_t bmi270_config_file[];  ///< BMI270 config file provided by Bosch Sensortec, declared in bmi270.c
// NOLINTEND(readability-identifier-naming)

//state variables
static volatile TaskHandle_t task_handle = NULL;  ///< handle of the FreeRTOS task
static SemaphoreHandle_t angles_mutex = NULL;     ///< handle of the mutex used to protect angles measurements
static FunctionCode state = kBMI270stateStartup;  ///< State machine current state
static uint8_t reset_occurred = 1;                ///< Flag used to ensure config is written only once after reset
static ErrorCode result;                          ///< Variable used to store error codes
static MahonyContext filter_context = {
    .ki = kIntegralGain,
    .kp = kProportionalGain,
    .dt = {.max_tick = kMaxSensorTimeTicks,
           .resolution_seconds = kSensorTimeResolutionSeconds}};  ///< Current Mahony filter context
static float angles_zeroing_rad[kNBaxis - 1] = {0, 0};            ///< Latest angles measured and filtered in [rad]
static SPI spi_descriptor = {.handle = SPI1,
                             .cs_port = BMI270_CS_GPIO_Port,
                             .pin = BMI270_CS_Pin,
                             .highest_register_number = BMI2_CMD_REG_ADDR,
                             .read_mask = kBMIread,
                             .write_mask = kBMIwrite};  ///< Descriptor of the SPI port used
static volatile uint8_t task_notifiable = 0;  ///< Flag used to avoid notifying the BMI270 task if it's not ready

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * @brief BMI270 INT1 and INT2 GPIO pins interrupt handler
 * 
 * @param interrupt_pin Number of the pin triggered (1 or 2)
 */
void bmi270InterruptTriggered(uint8_t interrupt_pin) {
    BaseType_t has_woken = 0;

    // vPortEnterCritical();

    //if INT1, notify the BMI270 task
    // (the event of concurrent access to bmi270configured will not be of consequence)
    if (task_notifiable && (interrupt_pin == 1)) {
        vTaskNotifyGiveFromISR(task_handle, &has_woken);
    }

    // vPortExitCritical();

    portYIELD_FROM_ISR(has_woken);
}

/**
 * Create a BMI270 FreeRTOS static task
 */
void createBMI270Task(void) {
    static StackType_t task_stack[kStackSize] = {0};     ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};                ///< Task state variables
    static StaticSemaphore_t measure_mutex_state = {0};  ///< ADC value mutex state variables

    LL_SPI_Enable(spi_descriptor.handle);

    //create the static task
    task_handle =
        xTaskCreateStatic(taskBMI270, "BMI270 task", kStackSize, NULL, kTaskLowPriority, task_stack, &task_state);
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
 * Check the SPI communication by reading the Chip ID
 * @details This function reads the chip ID in a loop until either the ID is correct or timeout occurs
 * 
 * @retval 0 Success
 * @retval 1 Error while reading the Chip ID register
 * @retval 2 Could not read a correct chip ID within the allocated time
 */
static ErrorCode checkSPICommunication(void) {
    BMI270register device_id = 0;

    //attempt to read a correct device ID for max. 1 second
    uint32_t first_tick = HAL_GetTick();
    do {
        //store the bitwise-NOT value of the chip ID to make sure every bit is changed by the read command
        device_id = (BMI270register) ~((BMI270register)BMI270_CHIP_ID);

        //read the register
        result = readRegisters(&spi_descriptor, BMI2_CHIP_ID_ADDR, &device_id, 1);
        if (isError(result)) {
            return (pushErrorCode(result, kBMI270checkCommunication, 1));
        }
    } while ((device_id != BMI270_CHIP_ID) && !timeout(first_tick, kTimeoutMS));

    //check the device ID
    if (device_id != BMI270_CHIP_ID) {
        return (createErrorCode(kBMI270checkCommunication, 2, kErrorCritical));
    }

    return kSuccessCode;
}

/**
 * Write the configuration file to the BMI270 and check it is correct
 *
 * @retval 0 Success
 * @retval 1 Error while writing the config file
 * @retval 2 The Config file was not written correctly
 */
static ErrorCode writeConfigurationFile(void) {
    //Write the config file provided by Bosch Sensortec
    result = writeRegisters(&spi_descriptor, BMI2_INIT_DATA_ADDR, bmi270_config_file, kConfigFileSize);
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateInitialising, 1);
    }

    //re-read the configuration file written to make sure it is correct (max. 3 attempts)
    uint8_t attempts = kConfigCheckAttempts;
    do {
        result = checkConfigurationFile();
    } while ((--attempts) && isError(result));
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateInitialising, 2);
    }

    return result;
}

/**
 * Read the configuration file stored in the BMI270 and make sure it corresponds to the one provided by Bosch
 * 
 * @retval 0 Success
 * @retval 1 Timeout while reading the configuration file
 * @retval 2 Data difference between configuration files
 */
static ErrorCode checkConfigurationFile(void) {
    const uint8_t spi_rx_filler = 0xFFU;  ///< Value to send as a filler while receiving multiple bytes

    //set timeout timer and enable CS
    uint32_t spi_start_tick = HAL_GetTick();
    LL_GPIO_ResetOutputPin(BMI270_CS_GPIO_Port, BMI270_CS_Pin);

    //send the config data read request and receive a dummy byte
    (void)receiveSPIbyte(&spi_descriptor, BMI2_INIT_DATA_ADDR, spi_start_tick);
    (void)receiveSPIbyte(&spi_descriptor, spi_rx_filler, spi_start_tick);

    //receive the bytes to read
    uint8_t failure = 0;
    uint16_t byte_index = 0;
    do {
        BMI270register value = receiveSPIbyte(&spi_descriptor, spi_rx_filler, spi_start_tick);
        failure = (value != bmi270_config_file[byte_index]);
        byte_index++;
    } while (!failure && (byte_index < kConfigFileSize) && !timeout(spi_start_tick, kSPItimeoutMS));

    //wait for transaction to be finished and clear Overrun flag
    while (LL_SPI_IsActiveFlag_BSY(spi_descriptor.handle) && !timeout(spi_start_tick, kSPItimeoutMS)) {
    };
    LL_SPI_ClearFlag_OVR(spi_descriptor.handle);

    //disable CS
    LL_GPIO_SetOutputPin(BMI270_CS_GPIO_Port, BMI270_CS_Pin);

    //if timeout, error
    if (timeout(spi_start_tick, kSPItimeoutMS)) {
        return (createErrorCode(kBMI270checkConfiguration, 1, kErrorWarning));
    }

    if (failure || (byte_index < kConfigFileSize)) {
        return (createErrorCode(kBMI270checkConfiguration, 2, kErrorWarning));
    }

    return kSuccessCode;
}

/**
 * Read the BMI270 internal temperature
 * 
 * @param[out] temperature_celsius Temperature in [°C] to update if the temperature read is valid
 * @retval 0 Success
 * @retval 1 Error while reading the registers
 */
static ErrorCode readTemperature(float* temperature_celsius) {
    int16_t read_buffer = 0x0000;

    //read temperature value
    result = readRegisters(&spi_descriptor, BMI2_TEMPERATURE_0_ADDR, (uint8_t*)&read_buffer, kNBTemperatureRegisters);
    if (isError(result)) {
        return (pushErrorCode(result, kBMI270readTemperature, 1));
    }

    //check if the temperature LSB read is valid
    const int16_t invalid_temperature_lsb = (int16_t)0x8000;
    if (read_buffer == invalid_temperature_lsb) {
        return kSuccessCode;
    }

    //transform temperature LSB to °C
    const float kelvin_per_lsb = 0.001953125F;
    *temperature_celsius = kReferenceTemperatureCelsius + ((float)read_buffer * kelvin_per_lsb);

    return kSuccessCode;
}

/**
 * Send the self-test request to the BMI270 and read its results
 * 
 * @param[out] values_read Buffer to which save the read self-test values 
 * @param positive_sign 1 if self-test sign is positive, 0 if negative
 * @retval 0 Success
 * @retval 1 Error while sending the self-test request
 * @retval 2 Error while reading the self-test results
 */
static ErrorCode runAccelerometerSelfTest(RawValues* values_read, uint8_t positive_sign) {
    //prepare the register value to write to start the self-testing
    BMI270register value = (uint8_t)(positive_sign << BMI2_ACC_SELF_TEST_SIGN_POS);
    value |= (1U << BMI2_ACC_SELF_TEST_AMP_POS);
    value |= BMI2_ACC_SELF_TEST_EN_MASK;

    //write the self-test request and wait for it to finish
    result = writeRegisters(&spi_descriptor, BMI2_ACC_SELF_TEST_ADDR, &value, 1U);
    if (isError(result)) {
        return pushErrorCode(result, kBMI270runAccelSelfTest, 1);
    }
    vTaskDelay(pdMS_TO_TICKS(51U));

    //read all accelerometer/gyroscope values
    result = readRegisters(&spi_descriptor, BMI2_ACC_X_LSB_ADDR, values_read->registers8bits, kNBpositionRegisters);
    if (isError(result)) {
        return (pushErrorCode(result, kBMI270runAccelSelfTest, 2));
    }

    return kSuccessCode;
}

/**
 * Apply the gyroscope self-test procedure
 *
 * @retval 0 Success
 * @retval 1 Error while sending the gyro test trigger command
 * @retval 2 Could not get a self-test done signal in time
 * @retval 3 Error while retrieving the self-test status
 * @retval 4 At least one axis failed the self-test
 */
static ErrorCode runGyroscopeSelfTest(void) {
    BMI270register value = 0;

    //set the gyro self-test trigger command
    value = BMI2_G_TRIGGER_CMD;
    result = writeRegisters(&spi_descriptor, BMI2_CMD_REG_ADDR, &value, 1);
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateSelfTestGyro, 1);  // NOLINT(*-magic-numbers)
    }

    //wait for the self-test done flag
    uint32_t start_tick = HAL_GetTick();
    do {
        result = readRegisters(&spi_descriptor, BMI2_GYR_SELF_TEST_AXES_ADDR, &value, 1);
    } while (!isError(result) && !timeout(start_tick, kTimeoutMS) &&
             ((value & BMI2_ACC_SELF_TEST_DONE_MASK) != BMI2_ACC_SELF_TEST_DONE_MASK));

    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateSelfTestGyro, 2);  // NOLINT(*-magic-numbers)
    }

    //check if all axis are flagged as ok
    const BMI270register gyro_gain_status_registernumber = 0x38;
    const BMI270register all_gyroscope_ok =
        (BMI2_ACC_X_OK_MASK | BMI2_ACC_Y_OK_MASK | BMI2_ACC_Z_OK_MASK | BMI2_ACC_SELF_TEST_DONE_MASK);
    if ((value & all_gyroscope_ok) != all_gyroscope_ok) {
        BMI270register trigger_status = 0;
        result = readRegisters(&spi_descriptor, gyro_gain_status_registernumber, &trigger_status, 1);
        if (isError(result)) {
            return pushErrorCode(result, kBMI270stateSelfTestGyro, 3);  // NOLINT(*-magic-numbers)
        }
        state = kBMI270stateError;
        result = createErrorCodeLayer1(kBMI270stateSelfTestGyro, value, trigger_status, kErrorCritical);
        return pushErrorCode(result, kBMI270stateSelfTestGyro, 4);  // NOLINT(*-magic-numbers)
    }

    return kSuccessCode;
}

/**
 * @brief Transpose a measurement to an angle in tenths of degrees with the Z axis
 *
 * @param axis Axis for which get the angle with the Z axis
 * @return Angle with the Z axis
 */
int16_t getAngleDegreesTenths(Axis axis) {
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kSPItimeoutMS)) == pdFALSE) {
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
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kSPItimeoutMS)) == pdFALSE) {
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
void bmi270ZeroDown(void) {
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kSPItimeoutMS)) == pdFALSE) {
        return;
    }

    angles_zeroing_rad[kXaxis] = -angleAlongAxis(&filter_context, kXaxis);
    angles_zeroing_rad[kYaxis] = -angleAlongAxis(&filter_context, kYaxis);

    xSemaphoreGive(angles_mutex);
}

/**
 * @brief Set the measurements in absolute mode (no zeroing compensation)
 */
void bmi270CancelZeroing(void) {
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kSPItimeoutMS)) == pdFALSE) {
        return;
    }

    angles_zeroing_rad[kXaxis] = 0.0F;
    angles_zeroing_rad[kYaxis] = 0.0F;

    xSemaphoreGive(angles_mutex);
}

/**
 * Assemble sensor time registers into tick value
 *
 * @param registers Sensor time registers
 * @return Tick value
 */
static inline uint32_t toTicks(const uint8_t registers[3]) {
    return (registers[0] | (uint32_t)(registers[1U] << 8U)  //NOLINT (*-magic-numbers)
            | (uint32_t)(registers[2] << 16U));             //NOLINT (*-magic-numbers)
}

/**
 * Apply temperature drifts to the measurements
 *
 * @param temperature_celcius Internal temperature in [°C]
 * @param[out] accelerometer_g Accelerometer values in [G] (9.81 m/s²)
 * @param[out] gyroscope_radps Gyroscope values in [rad/s]
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
static void applyTemperatureDrift(const float temperature_celcius, float accelerometer_g[3], float gyroscope_radps[3]) {
    const float delta_celsius = temperature_celcius - kReferenceTemperatureCelsius;
    const float accel_drift_percent = (delta_celsius * kAccelTemperatureDriftPercentPerKelvin);
    const float gyro_drift_percent = (delta_celsius * kGyroTemperatureDriftPercentPerKelvin);

    for (uint8_t axis = 0; axis < (uint8_t)kNBaxis; axis++) {
        accelerometer_g[axis] += (accel_drift_percent * accelerometer_g[axis]);
        gyroscope_radps[axis] += (gyro_drift_percent * gyroscope_radps[axis]);
    }
}

/**
 * Write an array of register number/value pairs in the BMI270 
 *
 * @param registers Registers number/value pairs to write
 * @param nb_registers Number of registers to write
 * @return Result of the operation
 */
static ErrorCode configureRegisters(const BMI270register registers[][2], const uint8_t nb_registers) {
    BMI270register reg = 0;
    do {
        result = writeRegisters(&spi_descriptor, registers[reg][0], &registers[reg][1], 1);
        reg++;
    } while ((reg < nb_registers) && !isError(result));
    if (isError(result)) {
        result = pushErrorCode(result, kBMI270configureRegisters, 1);
    }

    return result;
}

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Run the BMI270 state machine
 * 
 * @param argument Unused
 */
static void taskBMI270(void* argument) {
    // #lizard forgives
    UNUSED(argument);

    while (1) {
        switch (state) {
            case kBMI270stateStartup:
                result = stateStartup();
                break;

            case kBMI270stateSelfTestAccel:
                result = stateSelfTestingAccelerometer();
                break;

            case kBMI270stateSelfTestGyro:
                result = stateSelfTestingGyroscope();
                break;

            case kBMI270stateInitialising:
                result = stateInitialising();
                break;

            case kBMI270stateConfiguring:
                result = stateConfiguring();
                break;

            case kBMI270stateMeasuring:
                result = stateMeasuring();
                break;

            case kBMI270stateError:
                stateError();
                break;

            case kBMI270readRegisters:
            case kBMI270writeRegisters:
            case kBMI270checkCommunication:
            case kBMI270checkConfiguration:
            case kBMI270readTemperature:
            case kBMI270task:
            case kBMI270runAccelSelfTest:
            case kBMI270configureRegisters:
            default:
                result = createErrorCode(kBMI270task, 0, kErrorCritical);
                break;
        }

        if (isError(result)) {
            state = kBMI270stateError;
            Error_Handler();
        }
    }
}

/**
 * State in which the BMI270 is resetted and the communication is checked
 * 
 * @retval 0 Success
 * @retval 1 Error while checking the communication
 * @retval 2 Error while requesting the soft reset
 * @retval 3 Error while re-checking the communication
 */
static ErrorCode stateStartup(void) {
    BMI270register value = 0;

    //wait for a while for the sensor to boot properly, and enable SPI
    task_notifiable = 0;
    vTaskDelay(pdMS_TO_TICKS(kStartupTimeMS));

    result = checkSPICommunication();
    if (isError(result)) {
        return (pushErrorCode(result, kBMI270stateStartup, 1));
    }

    //issue a soft reset command to reset all registers to their default value
    // (useful when debugging and the supply voltage is not interrupted)
    value = BMI2_SOFT_RESET_CMD;
    result = writeRegisters(&spi_descriptor, BMI2_CMD_REG_ADDR, &value, 1);
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateStartup, 2);
    }

    //wait for a while for the sensor to reset properly, and read a register to enable SPI
    vTaskDelay(pdMS_TO_TICKS(kStartupTimeMS));
    result = checkSPICommunication();
    if (isError(result)) {
        return (pushErrorCode(result, kBMI270stateStartup, 3));
    }

    //Disable advanced power mode
    value = 0x00;
    result = writeRegisters(&spi_descriptor, BMI2_PWR_CONF_ADDR, &value, 1);
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateInitialising, 1);
    }

    //Getting out of advanced power mode takes up to 450us
    vTaskDelay(pdMS_TO_TICKS(1U));

    reset_occurred = 1;
    state = kBMI270stateInitialising;
    return kSuccessCode;
}

/**
 * State in which the BMI270 is initialised, following the POR procedure in the datasheet
 *
 * @details This follows the procedure descripted in the datasheet :
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf#%5B%7B%22num%22%3A64%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C33%2C771%2C0%5D
 * 
 * @retval 0 Success
 * @retval 1 Error while requesting config initialisation start
 * @retval 2 Error while writing the config file
 * @retval 3 Error while requesting config initialisation finish
 * @retval 4 Error while reading the internal status
 * @retval 5 Invalid internal status
 */
static ErrorCode stateInitialising(void) {
    const BMI270register start_configfile_load = 0x00U;
    const BMI270register finish_configfile_load = 0x01U;
    BMI270register value = 0;

    //make sure config file is written only once after POR or soft reset
    if (!reset_occurred) {
        state = kBMI270stateSelfTestAccel;
        return kSuccessCode;
    }
    reset_occurred = 0;

    //Request configuration start
    value = start_configfile_load;
    result = writeRegisters(&spi_descriptor, BMI2_INIT_CTRL_ADDR, &value, 1);
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateInitialising, 1);
    }

    result = writeConfigurationFile();
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateInitialising, 2);
    }

    //Request configuration finish
    value = finish_configfile_load;
    result = writeRegisters(&spi_descriptor, BMI2_INIT_CTRL_ADDR, &value, 1);
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateInitialising, 3);
    }

    //Applying the new config takes up to 20ms
    vTaskDelay(pdMS_TO_TICKS(kConfigTimeoutMS));

    //Wait for the initialisation status register to show a success or an error
    uint32_t config_start_tick = HAL_GetTick();
    do {
        result = readRegisters(&spi_descriptor, BMI2_INTERNAL_STATUS_ADDR, &value, 1);
        if (isError(result)) {
            return pushErrorCode(result, kBMI270stateInitialising, 4);
        }
    } while ((value == BMI2_NOT_INIT) && !timeout(config_start_tick, kConfigTimeoutMS));

    //check if an initialisation error occurred
    if (value != BMI2_INIT_OK) {
        return createErrorCodeLayer1(kBMI270stateInitialising, 5, value, kErrorCritical);  // NOLINT(*-magic-numbers)
    }

    state = kBMI270stateSelfTestAccel;
    return kSuccessCode;
}

/**
 * State in which the BMI270 self-tests the accelerometer
 *
 * @details This follows the procedure descripted in the datasheet :
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf#%5B%7B%22num%22%3A172%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C33%2C750%2C0%5D
 * 
 * @retval 0 Success
 * @retval 1 Error while writing preliminary self-test configuration
 * @retval 2 Error while performing the positive-sign self-test
 * @retval 3 Error while performing the negative-sign self-test
 * @retval 4 Error while disabling the self-test
 * @retval 5 Self-test value deltas do not match the ones stated in the datasheet
 */
static ErrorCode stateSelfTestingAccelerometer(void) {
    const Register accel_range_register_number = 0x41U;  ///< Accelerometer range value register number
    BMI270register value = 0;

    const BMI270register configuration[][2] = {
        {BMI2_PWR_CTRL_ADDR, (BMI2_ACC_EN_MASK)},           //enable temp, gyroscope and accel.
        {accel_range_register_number, BMI2_ACC_RANGE_16G},  //set the accel. range to +-16G
        {BMI2_ACC_CONF_ADDR, (1U << BMI2_ACC_FILTER_PERF_MODE_POS) |
                                 (BMI270register)(BMI2_ACC_NORMAL_AVG4 << BMI2_ACC_BW_PARAM_POS) | BMI2_ACC_ODR_1600HZ},
    };

    //send the configuration array
    result = configureRegisters(configuration, (sizeof(configuration) / sizeof(configuration)[0]));
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateSelfTestAccel, 1);
    }

    //wait for 2ms
    vTaskDelay(pdMS_TO_TICKS(3U));

    RawValues lsb_posivite = {0};  ///< Buffer holding the LSB values read from the IC
    RawValues lsb_negative = {0};  ///< Buffer holding the LSB values read from the IC

    result = runAccelerometerSelfTest(&lsb_posivite, 1);
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateSelfTestAccel, 2);
    }
    result = runAccelerometerSelfTest(&lsb_negative, 0);
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateSelfTestAccel, 3);
    }

    //disable self-testing
    value = 0x00;
    result = writeRegisters(&spi_descriptor, BMI2_ACC_SELF_TEST_ADDR, &value, 1U);
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateSelfTestAccel, 4U);  // NOLINT(*-magic-numbers)
    }

    //transform the values read to values in [G] (9.81 m/s²)
    const float delta_x_g =
        (float)(lsb_posivite.values16bits[kXaxis] - lsb_negative.values16bits[kXaxis]) * kNominalAccelLSBto16G;
    const float delta_y_g =
        (float)(lsb_posivite.values16bits[kYaxis] - lsb_negative.values16bits[kYaxis]) * kNominalAccelLSBto16G;
    const float delta_z_g =
        (float)(lsb_posivite.values16bits[kZaxis] - lsb_negative.values16bits[kZaxis]) * kNominalAccelLSBto16G;

    //check if the minimum delta given in the datasheet is respected
    const float min_delta_x_g = 16.0F;
    const float min_delta_y_g = -15.0F;
    const float min_delta_z_g = 10.0F;
    if ((delta_x_g <= min_delta_x_g) || (delta_y_g >= min_delta_y_g) || (delta_z_g <= min_delta_z_g)) {
        return createErrorCode(kBMI270stateSelfTestAccel, 5, kErrorCritical);  // NOLINT(*-magic-numbers)
    }

    state = kBMI270stateSelfTestGyro;
    return kSuccessCode;
}

/**
 * State in which the BMI270 self-tests the gyroscope
 *
 * @details This follows the procedure descripted in the datasheet :
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf#%5B%7B%22num%22%3A172%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C33%2C750%2C0%5D
 * 
 * @retval 0 Success
 * @retval 1 Error while requesting the self-test triggering
 * @retval 2 Error while reading the self-test result
 * @retval 3 Error while reading the self-test error code
 * @retval 4 Gyroscope self-test internal error @note Internal error code stored one layer lower
 */
static ErrorCode stateSelfTestingGyroscope(void) {
    uint8_t attempts = 3;
    do {
        result = runGyroscopeSelfTest();
    } while (--attempts && isError(result));

    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateSelfTestGyro, 1);  // NOLINT(*-magic-numbers)
    }

    if (!attempts) {
        return pushErrorCode(result, kBMI270stateSelfTestGyro, 2);  // NOLINT(*-magic-numbers)
    }

    state = kBMI270stateConfiguring;
    return kSuccessCode;
}

/**
 * State in which the BMI270 is configured for the application
 * 
 * @retval 0 Success
 * @retval 1 Error while writing a register
 */
static ErrorCode stateConfiguring(void) {
    const Register accel_range_register_number = 0x41U;  ///< Accelerometer range value register number
    const Register gyro_range_register_number = 0x43U;   ///< Gyroscope range value register number
    const Register no_fifo = 0x00;                       ///< Value used to disable the FIFO

    const BMI270register configuration[][2] = {
        {BMI2_PWR_CTRL_ADDR,
         (BMI2_GYR_EN_MASK | BMI2_ACC_EN_MASK | BMI2_TEMP_EN_MASK)},  //enable temp, gyroscope and accel.
        {BMI2_ACC_CONF_ADDR, (BMI2_ACC_FILTER_PERF_MODE_MASK | (BMI2_ACC_CIC_AVG8 << 4U) | BMI2_ACC_ODR_1600HZ)},  //
        {accel_range_register_number, BMI2_ACC_RANGE_2G},  //set the accel. range to +-2G
        {BMI2_GYR_CONF_ADDR, (BMI2_GYR_FILTER_PERF_MODE_MASK | BMI2_GYR_NOISE_PERF_MODE_MASK |
                              (BMI2_GYR_NORMAL_MODE << 4U) | BMI2_GYR_ODR_1600HZ)},
        {gyro_range_register_number, BMI2_GYR_RANGE_125},  //set the gyroscope range to +-500°/s
        {BMI2_FIFO_CONFIG_1_ADDR, no_fifo},                //disable the FIFO (streaming mode)
        {BMI2_INT_MAP_DATA_ADDR, BMI2_DRDY_INT},           //enable Data Ready interrupt on INT1
        {BMI2_INT1_IO_CTRL_ADDR, BMI2_INT_OUTPUT_EN_MASK | BMI2_INT_OPEN_DRAIN_MASK |
                                     BMI2_INT_LEVEL_MASK},  //enable INT1 active high, open drain
        {BMI2_PWR_CONF_ADDR, 0x00},                         //disable advanced power mode
    };

    //send the configuration array
    result = configureRegisters(configuration, (sizeof(configuration) / sizeof(configuration)[0]));
    if (isError(result)) {
        return pushErrorCode(result, kBMI270stateConfiguring, 1);
    }

    resetMahonyFilter(&filter_context);
    task_notifiable = 1;
    state = kBMI270stateMeasuring;
    return kSuccessCode;
}

/**
 * State in which measurements are received from the BMI270
 * 
 * @retval 0 Success
 * @retval 1 Did not receive any data ready interrupt
 * @retval 2 Error while reading the data registers
 * @retval 3 Error while reading the temperature registers
 */
static ErrorCode stateMeasuring(void) {
    float accelerometer_g[kNBaxis];    ///< Accelerometer values in [G] (9.81 m/s²)
    float gyroscope_radps[kNBaxis];    ///< Gyroscope values in [rad/s]
    RawValues lsb_values = {0};        ///< Buffer holding the LSB values read from the IC
    float temperature_celsius = 0.0F;  ///< Latest internal temperature in [°C]

    //wait for measurements to be ready
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kTimeoutMS)) == pdFALSE) {
        return (createErrorCode(kBMI270stateMeasuring, 1, kErrorCritical));
    }

    //read all accelerometer/gyroscope values
    result = readRegisters(&spi_descriptor, BMI2_ACC_X_LSB_ADDR, lsb_values.registers8bits, kNBpositionRegisters);
    if (isError(result)) {
        return (pushErrorCode(result, kBMI270stateMeasuring, 2));
    }

    //apply the nominal sensitivity to the measurements to change them to usable units
    for (uint8_t axis = 0; axis < (uint8_t)kNBaxis; axis++) {
        accelerometer_g[axis] = (float)(lsb_values.values16bits[axis]) * kNominalAccelLSBto2G;
        gyroscope_radps[axis] = (float)(lsb_values.values16bits[axis + kNBaxis]) * kNominalGyroLSBtoRadps;
    }

    //read the BMI270 temperature every 10ms
    static TickType_t latest_temperature_tick = 0;
    if (timeout(latest_temperature_tick, kTemperatureRefreshMS)) {
        latest_temperature_tick = HAL_GetTick();

        result = readTemperature(&temperature_celsius);
        if (isError(result)) {
            return (pushErrorCode(result, kBMI270stateMeasuring, 3));
        }
    }

    //take temperature drift into account with sensitivity
    applyTemperatureDrift(temperature_celsius, accelerometer_g, gyroscope_radps);

    //update the current sensor time ticks
    const uint8_t nb_measure_registers = (uint8_t)kNBaxis << 2U;
    filter_context.dt.current_tick = toTicks(&lsb_values.registers8bits[nb_measure_registers]);

    //apply sensor fusion to the measurements
    if (xSemaphoreTake(angles_mutex, pdMS_TO_TICKS(kSPItimeoutMS)) == pdTRUE) {
        updateMahonyFilter(&filter_context, accelerometer_g, gyroscope_radps);
        xSemaphoreGive(angles_mutex);
    }

    return kSuccessCode;
}

/**
 * State in which BMI270 is in error
 */
static void stateError(void) {}
