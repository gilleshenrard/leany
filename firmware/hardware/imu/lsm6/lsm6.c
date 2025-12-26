/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file lsm6.c
 * @brief Implement the LSM6 MEMS sensor communication
 * @author Gilles Henrard
 *
 * @note Additional information can be found in :
 *   - Datasheet : https://www.st.com/resource/en/datasheet/lsm6dsr.pdf
 *   - AN5358 (LSM6DSR: always-on 6-axis IMU) : https://www.st.com/resource/en/application_note/an5358-lsm6dsr-alwayson-6axis-imu-inertial-measurement-unit-stmicroelectronics.pdf
 *   - AN5226 (Finite State Machine) : https://www.st.com/resource/en/application_note/an5226-lsm6dso-finite-state-machine-stmicroelectronics.pdf
 *   - DT0058 (Design tip) : https://www.st.com/resource/en/design_tip/dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.pdf
 */

//used in Doxygen
#if IMU_MODEL == IMU_LSM6

#include <main.h>
#include <projdefs.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_ll_spi.h>
#include <task.h>

#include "errorstack.h"
#include "hal_spi.h"
#include "imu.h"
#include "lsm6_registers.inc"
#include "lsm6dsr_registers.inc"
#include "sensorfusion.h"

enum {
    kBootTimeMS = 100U,          ///< Number of milliseconds to wait for the MEMS to boot
    kTimeoutMS = 1000U,          ///< Max number of milliseconds to wait for the device ID
    kRegisterValueAlign = 8,     ///< Memory alignment of the RegisterValue struct
    kNbRegistersToRead = 14U,    ///< Numbers of data registers to read
    kNbTickRegisters = 4U,       ///< Number of sensor tick registers
    kDriftAlignment = 32U,       ///< Memory alignment of the TemperatureDrift struct
    kNbSelfTestRegisters = 10U,  ///< Number of registers to write during self-test
};

/**
 * @brief Enumeration of all the function ID used in errors
 */
typedef enum {
    kResetting = 1,              ///< IMUsoftReset() function
    kCheckDeviceID = 2,          ///< stateStartup() state
    kConfiguring = 3,            ///< stateConfiguring() state
    kDropping = 4,               ///< stateIgnoringSamples() state
    kMeasuring = 5,              ///< stateMeasuring() state
    kConfigureRegisters = 6,     ///< configureRegisters() function
    kSelfTestAccelerometer = 7,  ///< selfTestAccelerometer() function
    kSelfTestGyroscope = 8,      ///< selfTestGyroscope() function
    kWaitAndRead = 9,            ///< waitAndRead() function
    kAverageMeasures = 10,       ///< getAverageMeasures() function
} FunctionCode;

/**
 * @brief Structure representing a value to write at a specific register
 */
typedef struct {
    LSM6register register_id;  ///< Register ID to which write the value
    uint8_t value;             ///< Value to write
} __attribute__((aligned(kRegisterValueAlign))) RegisterValue;

/**
 * @brief Union regrouping 8-bits and 16-bits arrays
 * @details
 *  This allows reading all the accelerometer and gyroscope values at once, and convert them to 16 bit instantly
 */
typedef union {
    uint8_t registers8bits[kNbRegistersToRead];     ///< 8-bits registers array
    int16_t values16bits[kNbRegistersToRead / 2U];  ///< 16-bits values array
} RawValues;

/**
 * Structure defining the internal IMU temperature, and drifts to be applied
 */
typedef struct {
    float current_celsius;           ///< Current internal temperature in [°C]
    float gyro_scaled_sensitivity;   ///< Gyroscope sensitivity, scaled with temperature, in [rad/s / LSB]
    float accel_scaled_sensitivity;  ///< Accelerometer sensitivity, scaled with temperature, in [G/LSB]
    float gyro_bias;                 ///< Gyroscope measurements bias due to temperature, in [rad/s]
    float accel_bias;                ///< Accelerometer measurements bias due to temperature, in [G]
} __attribute__((aligned(kDriftAlignment))) Temperature;

//machine state
static void updateTemperature(int16_t lsb_value);
static ErrorCode getAverageMeasures(SPIregister first_register, SPIregister available_mask, int16_t measures[kNBaxis]);
static ErrorCode waitAndRead(SPIregister first_register, SPIregister available_mask, const int16_t measures[kNBaxis]);
static uint8_t isAccelSelfTestValid(const int16_t self_test_off[kNBaxis], const int16_t self_test_on[kNBaxis]);
static uint8_t isGyroSelfTestValid(const int16_t self_test_off[kNBaxis], const int16_t self_test_on[kNBaxis]);

/**
 * Temperature at which the LSM6 temperature reading will give 0
 */
static const float kBASE_TEMPERATURE = 25.0F;

/**
 * Temperature sensitivity = 256 [LSB/°C] = 0.00390625 [°C/LSB] + LSB = 0 @ 25°C
 */
static const float kTemperatureSensitivity = 0.00390625F;

/**
 * Gyroscope sensitivity at 125°/s = 4.375[mdps/LSB]
 * to rad/s : (sensitivity / 1000[mdps/dps]) * (PI/180°) = 0.000076358155
 */
static const float kGyroSensitivity125dps = 0.000076358155F;

/**
 * Gyroscope sensitivity at 2000°/s = 70[mdps/LSB] = 0.07[dps/LSB]
 */
static const float kGyroSensitivity2000dps = 0.07F;

/**
 * Gyroscope temperature bias = 0.005 [dps/°C]
 * to radps/°C : sensitivity * (PI/180°) = 8.72664e-5
 */
static const float kGyroTemperatureBias = 8.726646e-5F;

/**
 * Gyroscope sensitivity drift due to temperature = 0.007 [%/°C]
 */
static const float kGyroTemperatureSensitivity = 0.00007F;

/**
 * Accelerometer sensitivity at 2G = 0.061 [mG/LSB] = 0.000061 [G/LSB] (9.81m/s²)
 */
static const float kAccelSensitivity2g = 0.000061F;

/**
 * Accelerometer sensitivity at 4G = 0.122 [mG/LSB]
 */
static const float kAccelSensitivity4g_mg = 0.122F;

/**
 * Accelerometer sensitivity drift due to temperature = 0.01 [%/°C]
 */
static const float kAccelTemperatureSensitivity = 0.0001F;

/**
 * Accelerometer temperature bias = 0.1 [mG/°C] = 0.0001 [G/°C]
 */
static const float kAccelTemperatureBias = 0.0001F;

//state variables
static ErrorCode result;                                                  ///< Variables used to store error codes
static Temperature temperature = {.current_celsius = kBASE_TEMPERATURE};  ///< Current temperature and biases
static SPI spi_descriptor = {.handle = SPI1,
                             .cs_port = IMU_CS_GPIO_Port,
                             .pin = IMU_CS_Pin,
                             .highest_register_number = kMAX_REGISTER,
                             .read_mask = kLSM6_READ,
                             .write_mask = kLSM6_WRITE,
                             .read_dummy = 0};  ///< Descriptor of the SPI port used

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Check whether the correct device ID can be retrieved
 *
 * @retval 0 Success
 * @retval 1 Error while reading the WHO_AM_I register
 * @retval 2 Incorrect device ID
 */
ErrorCode IMUcheckDeviceID(void) {
    uint8_t device_id = 0;

    LL_SPI_Enable(spi_descriptor.handle);

    //attempt to read a correct device ID for max. 1 second
    uint32_t first_tick = HAL_GetTick();
    do {
        //store the bitwise-NOT value of the chip ID to make sure every bit is changed by the read command
        // clang-format off
        device_id = (uint8_t)~((uint8_t)LSM6_WHOAMI);
        // clang-format on

        //read the register
        result = readRegisters(&spi_descriptor, kWHO_AM_I, &device_id, 1);
        EXIT_ON_ERROR(result, kCheckDeviceID, 1)
    } while ((device_id != LSM6_WHOAMI) && !timeout(first_tick, kTimeoutMS));

    //check the device ID
    if (device_id != LSM6_WHOAMI) {
        return (createErrorCode(kCheckDeviceID, 2, kErrorCritical));
    }

    return kSuccessCode;
}

/**
 * Request a device soft reset
 *
 * @retval 0 Success
 * @retval 1 Error while requesting request
 */
ErrorCode IMUsoftReset(void) {
    const SPIregister value = (kLSM6_SOFTWARE_RESET | kLSM6_INT_ACTIVE_LOW);  //NOLINT(hicpp-signed-bitwise)
    result = writeRegisters(&spi_descriptor, kCTRL3_C, &value, 1);
    EXIT_ON_ERROR(result, kResetting, 1)

    return kSuccessCode;
}

/**
 * Run the accelerometer self-test procedure
 *
 * @retval 0 Success
 * @retval 1 Error while writing self-test configuration
 * @retval 2 Error while reading the data with self-testing off
 * @retval 3 Error while enabling self-testing
 * @retval 4 Error while reading the data with self-testing on
 * @retval 5 Self-test results invalid
 * @retval 6 Error while disabling self-test
 */
ErrorCode selfTestAccelerometer(void) {
    const uint8_t selftest_config_registers[kNbSelfTestRegisters][2] = {
        // NOLINTBEGIN(misc-redundant-expression,hicpp-signed-bitwise)
        {kCTRL1_XL, kODR_XL_52HZ | kFS_XL_4G | kLPF2_XL_DISABLE},
        {kCTRL2_G, kGYR_POWER_DOWN},
        {kCTRL3_C,
         kLSM6_REBOOT_NORMAL | kLSM6_BDU_WAITBLOCK | kLSM6_INT_ACTIVE_HIGH | kLSM6_SPI_4WIRE | kLSM6_F_INC_AUTO},
        {kCTRL4_C, kGYR_SLEEP_DISABLE | kLSM6_DRDY_DISABLE | kLSM6_I2C_DISABLE | kGYR_LPF1_DISABLE},
        {kCTRL5_C, kGYR_SELFTEST_DISABLE | kAXL_SELFTEST_DISABLE},
        {kCTRL6_C, kLSM6_DEN_DISABLE | kAXL_HM_MODE},
        {kCTRL7_G, kGYR_HM_ENABLE | kGYR_HPF_DISABLE},
        {kCTRL8_XL, kAXL_NO_HP_FILTER | k6D_NO_LP_FILTER},
        {kCTRL9_XL, kAXL_DEN_DISABLE | kLSM6_I3C_ENABLE},
        {kCTRL10_C, kLSM6_TIMESTAMP_DISABLE}
        // NOLINTEND(misc-redundant-expression,hicpp-signed-bitwise)
    };

    //initialise and turn on sensor, then wait for a while
    result = writeRegistersBunch(&spi_descriptor, selftest_config_registers, kNbSelfTestRegisters);
    EXIT_ON_ERROR(result, kSelfTestAccelerometer, 1)
    vTaskDelay(pdMS_TO_TICKS(kBootTimeMS));

    //gather accelerometer values with self-test disabled
    int16_t self_test_off_measures[kNBaxis] = {0};
    result = getAverageMeasures(kOUTX_L_A, kLSM6_AXL_DATA_AVAIL, self_test_off_measures);
    EXIT_ON_ERROR(result, kSelfTestAccelerometer, 2)

    //enable self-testing with positive polarity and wait for a while
    SPIregister value = kAXL_SELFTEST_ENABLE_POS;
    result = writeRegisters(&spi_descriptor, kCTRL5_C, &value, 1U);
    EXIT_ON_ERROR(result, kSelfTestAccelerometer, 3)
    vTaskDelay(pdMS_TO_TICKS(kBootTimeMS));

    //gather accelerometer values with self-test enabled
    int16_t self_test_on_measures[kNBaxis] = {0};
    result = getAverageMeasures(kOUTX_L_A, kLSM6_AXL_DATA_AVAIL, self_test_on_measures);
    EXIT_ON_ERROR(result, kSelfTestAccelerometer, 4)

    //check if the self-test is valid
    if (!isAccelSelfTestValid(self_test_off_measures, self_test_on_measures)) {
        return createErrorCode(kSelfTestAccelerometer, 5, kErrorCritical);  //NOLINT(*-magic-numbers)
    }

    //disable self-test and the accelerometer
    const SPIregister disable_registers[][2] = {
        {kCTRL5_C, kAXL_SELFTEST_DISABLE},
        {kCTRL1_XL, kAXL_POWER_DOWN},
    };
    result = writeRegistersBunch(&spi_descriptor, disable_registers, 2U);
    EXIT_ON_ERROR(result, kSelfTestAccelerometer, 6)

    return kSuccessCode;
}

/**
 * Run the gyroscope self-test procedure
 *
 * @retval 0 Success
 */
ErrorCode selfTestGyroscope(void) {
    const uint8_t selftest_config_registers[kNbSelfTestRegisters][2] = {
        // NOLINTBEGIN(misc-redundant-expression,hicpp-signed-bitwise)
        {kCTRL1_XL, kAXL_POWER_DOWN},
        {kCTRL2_G, kGYR_ODR_208HZ | kGYR_FS_2000_DPS},
        {kCTRL3_C,
         kLSM6_REBOOT_NORMAL | kLSM6_BDU_WAITBLOCK | kLSM6_INT_ACTIVE_HIGH | kLSM6_SPI_4WIRE | kLSM6_F_INC_AUTO},
        {kCTRL4_C, kGYR_SLEEP_DISABLE | kLSM6_DRDY_DISABLE | kLSM6_I2C_DISABLE | kGYR_LPF1_DISABLE},
        {kCTRL5_C, kGYR_SELFTEST_DISABLE | kAXL_SELFTEST_DISABLE},
        {kCTRL6_C, kLSM6_DEN_DISABLE | kAXL_HM_MODE},
        {kCTRL7_G, kGYR_HM_ENABLE | kGYR_HPF_DISABLE},
        {kCTRL8_XL, kAXL_NO_HP_FILTER | k6D_NO_LP_FILTER},
        {kCTRL9_XL, kAXL_DEN_DISABLE | kLSM6_I3C_ENABLE},
        {kCTRL10_C, kLSM6_TIMESTAMP_DISABLE}
        // NOLINTEND(misc-redundant-expression,hicpp-signed-bitwise)
    };

    //initialise and turn on sensor, then wait for a while
    result = writeRegistersBunch(&spi_descriptor, selftest_config_registers, kNbSelfTestRegisters);
    EXIT_ON_ERROR(result, kSelfTestGyroscope, 1)
    vTaskDelay(pdMS_TO_TICKS(kBootTimeMS));

    //gather accelerometer values with self-test disabled
    int16_t self_test_off_measures[kNBaxis] = {0};
    result = getAverageMeasures(kOUTX_L_G, 0x02U, self_test_off_measures);
    EXIT_ON_ERROR(result, kSelfTestGyroscope, 2)

    //enable self-testing with positive polarity and wait for a while
    SPIregister value = kGYR_SELFTEST_ENABLE_POS;
    result = writeRegisters(&spi_descriptor, kCTRL5_C, &value, 1U);
    EXIT_ON_ERROR(result, kSelfTestGyroscope, 3)
    vTaskDelay(pdMS_TO_TICKS(kBootTimeMS));

    //gather accelerometer values with self-test enabled
    int16_t self_test_on_measures[kNBaxis] = {0};
    result = getAverageMeasures(kOUTX_L_G, 0x02U, self_test_on_measures);
    EXIT_ON_ERROR(result, kSelfTestGyroscope, 4)

    //check if the self-test is valid
    if (!isGyroSelfTestValid(self_test_off_measures, self_test_on_measures)) {
        return createErrorCode(kSelfTestGyroscope, 5, kErrorCritical);  //NOLINT(*-magic-numbers)
    }

    //disable self-test and the accelerometer
    const SPIregister disable_registers[][2] = {
        {kCTRL5_C, kGYR_SELFTEST_DISABLE},
        {kCTRL2_G, kGYR_POWER_DOWN},
    };
    result = writeRegistersBunch(&spi_descriptor, disable_registers, 2U);
    EXIT_ON_ERROR(result, kSelfTestGyroscope, 6)

    return kSuccessCode;
}

/**
 * Process the device initialisation
 *
 * @return Success
 */
ErrorCode IMUinitialise(void) { return kSuccessCode; }

/**
 * Write the LSM6 config
 *
 * @retval 0 Success
 * @retval 1 Error while writing a register
 */
ErrorCode IMUconfigure(void) {
    const uint8_t configuration_array[][2] = {
        // NOLINTBEGIN(misc-redundant-expression,hicpp-signed-bitwise)
        {kFIFO_CTRL4, kFIFO_MODE_BYPASS},
        {kINT1_CTRL, kINT1_AXL_DATA_RDY},
        {kCTRL1_XL, kFS_XL_2G | kAXL_ODR_416HZ | kLSM6_AXL_LPF2_ENABLE},
        {kCTRL2_G, kAXL_ODR_416HZ | kGYR_FS_125_DPS},
        {kCTRL4_C, kGYR_LPF1_ENABLE | kLSM6_I2C_DISABLE},
        {kCTRL5_C, kGYR_SELFTEST_DISABLE | kAXL_SELFTEST_DISABLE},
        {kCTRL6_C, kGYR_LPF1_CUTOFF_120_3HZ},
        {kCTRL7_G, kGYR_HPF_ENABLE | kGYR_HPF_CUTOFF_65MHZ | kLSM6_OIS_DISABLE},
        {kCTRL8_XL, kAXL_NO_HP_FILTER | kAXL_LPF2_ODR_4},
        {kCTRL9_XL, kAXL_DEN_DISABLE | kLSM6_I3C_DISABLE},
        {kCTRL10_C, kLSM6_TIMESTAMP_ENABLE},
        // NOLINTEND(misc-redundant-expression,hicpp-signed-bitwise)
    };

    //write all registers values from the initialisation array
    const uint8_t nb_config_registers = sizeof(configuration_array) / 2U;
    result = writeRegistersBunch(&spi_descriptor, configuration_array, nb_config_registers);
    EXIT_ON_ERROR(result, kConfiguring, 1)

    return kSuccessCode;
}

/**
 * Initialise the time fields of the Mahony filter's context
 *
 * @param[out] filter_context Mahony filter's current context
 */
void IMUsetupTimebase(MahonyContext* filter_context) {
    //read the internal frequency register
    uint8_t frequency_fine = 0;
    result = readRegisters(&spi_descriptor, kINTERNAL_FREQ_FINE, &frequency_fine, 1);
    if (isError(result)) {
        return;
    }

    //apply the frequency difference to the nominal frequency (lsm6dsr datasheet p.82)
    const float nominal_tick_hz = 40000.0F;
    const float freq_diff_step_100percent = 0.0015F;
    float tick_frequency = (nominal_tick_hz + (freq_diff_step_100percent * (float)frequency_fine * nominal_tick_hz));

    //update the Mahony filter's context timebase
    filter_context->dt.max_tick = UINT32_MAX;
    filter_context->dt.tick_period_seconds = 1.0F / tick_frequency;
}

/**
 * @note LSM6 implementation uses SPI burst read and a latched timestamp.
 */
/** \cond */
ErrorCode IMUgetSample(IMUsample* sample) {
    RawValues lsb_values = {0};

    //read all temp/accelerometer/gyroscope values
    result = readRegisters(&spi_descriptor, kOUT_TEMP_L, lsb_values.registers8bits, kNbRegistersToRead);
    EXIT_ON_ERROR(result, kMeasuring, 1)

    //read the current sensor tick value
    result = readRegisters(&spi_descriptor, kTIMESTAMP0, (uint8_t*)&sample->latest_tick, kNbTickRegisters);
    EXIT_ON_ERROR(result, kMeasuring, 2)

    //convert the temperature LSB values to °C, and compute temperature bias
    // Sparingly calculated since the temperature readings max. freq. is 52Hz
    updateTemperature(lsb_values.values16bits[0]);

    //convert the gyroscope and accelerometer LSB values to rad/s and G
    for (uint8_t axis = 0; axis < (uint8_t)kNBaxis; axis++) {
        sample->gyroscope_radps[axis] =
            ((float)(lsb_values.values16bits[axis + 1U]) * temperature.gyro_scaled_sensitivity) - temperature.gyro_bias;
        sample->accelerometer_g[axis] =
            ((float)(lsb_values.values16bits[axis + 4U]) * temperature.accel_scaled_sensitivity) -
            temperature.accel_bias;
    }

    return (kSuccessCode);
}
/** \endcond */

/**
 * Get the number of samples to ignore after configuration
 *
 * @return Number of samples to ignore 
 */
uint8_t getNbSamplesToIgnore(void) { return 2U; }

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Convert the temperature LSB values to °C, and compute temperature bias
 * @details Sparingly calculated since the temperature readings max. freq. is 52Hz
 *
 * @param lsb_value Latest temperature raw 16-bit value read
 */
static void updateTemperature(int16_t lsb_value) {
    static int16_t previoustemp_lsb = 0;

    //if temperature did not change, do nothing
    if (lsb_value == previoustemp_lsb) {
        return;
    }

    //compute current temperature and delta
    temperature.current_celsius = kBASE_TEMPERATURE + ((float)(lsb_value)*kTemperatureSensitivity);
    float delta_celsius = (temperature.current_celsius - kBASE_TEMPERATURE);

    //compute gyroscope sensitivity and bias
    temperature.gyro_scaled_sensitivity =
        kGyroSensitivity125dps * (1.0F + (kGyroTemperatureSensitivity * delta_celsius));
    temperature.gyro_bias = (kGyroTemperatureBias * delta_celsius);

    //compute accelerometer sensitivity and bias
    temperature.accel_scaled_sensitivity =
        kAccelSensitivity2g * (1.0F + (kAccelTemperatureSensitivity * delta_celsius));
    temperature.accel_bias = (kAccelTemperatureBias * delta_celsius);

    //save current LSB value
    previoustemp_lsb = lsb_value;
}

/**
 * Read several sensor values and get their average
 *
 * @param[out] first_register Number of the first register to burst-read 
 * @param[out] available_mask Mask used to know if the sensor available bit is set
 * @param[out] measures Array to fill whith the average values 
 * @retval 0 Success
 * @retval 1 Error while reading the values to discard
 * @retval 2 Error while reading the values to average
 */
static ErrorCode getAverageMeasures(SPIregister first_register, SPIregister available_mask, int16_t measures[kNBaxis]) {
    //read and discard a first set of data
    result = waitAndRead(first_register, available_mask, measures);
    EXIT_ON_ERROR(result, kAverageMeasures, 1)

    //read 5 values for each axis
    const uint8_t nb_average = 5U;
    int16_t raw[nb_average][kNBaxis];
    for (uint8_t data_set = 0; data_set < nb_average; data_set++) {
        result = waitAndRead(first_register, available_mask, raw[data_set]);
        EXIT_ON_ERROR(result, kAverageMeasures, 2)
    }

    //average out the values
    for (uint8_t axis = 0; axis < (uint8_t)kNBaxis; axis++) {
        measures[axis] = (int16_t)(((int32_t)raw[0][axis] + (int32_t)raw[1][axis] + (int32_t)raw[2][axis] +
                                    (int32_t)raw[3][axis] + (int32_t)raw[4][axis]) /
                                   (int32_t)nb_average);
    }

    return kSuccessCode;
}

/**
 * Wait for Data Available bit to be set and read accelerometer data
 *
 * @param[out] first_register Number of the first register to burst-read 
 * @param[out] available_mask Mask used to know if the sensor available bit is set
 * @param[out] measures Array in which store the data read 
 * @retval 0 Success
 * @retval 1 Error while reading DATA_AXL_AVAIL
 * @retval 2 DATA_AXL_AVAIL never went up
 * @retval 3 Error while reading the data registers
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
static ErrorCode waitAndRead(SPIregister first_register, SPIregister available_mask, const int16_t measures[kNBaxis]) {
    //wait until DATA_AXL_AVAIL bit is up
    SPIregister status = 0;
    SPIregister available = 0;
    const uint32_t current_tick = HAL_GetTick();
    do {
        result = readRegisters(&spi_descriptor, kSTATUS_REG, &status, 1);
        EXIT_ON_ERROR(result, kWaitAndRead, 1)

        available = ((status & available_mask) != 0U);
    } while (!available && !timeout(current_tick, kBootTimeMS));

    if (!available) {
        return createErrorCode(kWaitAndRead, 2, kErrorError);
    }

    result = readRegisters(&spi_descriptor, first_register, (uint8_t*)measures, kNBaxis * 2U);
    EXIT_ON_ERROR(result, kWaitAndRead, 3)

    return kSuccessCode;
}

/**
 * Check if accelerometer self-test values are within a valid range
 *
 * @param self_test_off Measurements taken with Self-Test disabled
 * @param self_test_on Measurements taken with Self-Test enabled
 * @retval 1 Self-test valid
 * @retval 0 Self-test invalid
 */
static uint8_t isAccelSelfTestValid(const int16_t self_test_off[kNBaxis], const int16_t self_test_on[kNBaxis]) {
    const float min_selftest_value_mg = 40.0F;
    const float max_selftest_value_mg = 1700.0F;

    const float self_test_range_mg[kNBaxis] = {
        [kXaxis] = (float)(self_test_on[kXaxis] - self_test_off[kXaxis]) * kAccelSensitivity4g_mg,
        [kYaxis] = (float)(self_test_on[kYaxis] - self_test_off[kYaxis]) * kAccelSensitivity4g_mg,
        [kZaxis] = (float)(self_test_on[kZaxis] - self_test_off[kZaxis]) * kAccelSensitivity4g_mg,
    };

    return ((self_test_range_mg[kXaxis] >= min_selftest_value_mg) &&
            (self_test_range_mg[kXaxis] <= max_selftest_value_mg) &&
            (self_test_range_mg[kYaxis] >= min_selftest_value_mg) &&
            (self_test_range_mg[kYaxis] <= max_selftest_value_mg) &&
            (self_test_range_mg[kZaxis] >= min_selftest_value_mg) &&
            (self_test_range_mg[kZaxis] <= max_selftest_value_mg));
}

/**
 * Check if gyroscope self-test values are within a valid range
 *
 * @param self_test_off Measurements taken with Self-Test disabled
 * @param self_test_on Measurements taken with Self-Test enabled
 * @retval 1 Self-test valid
 * @retval 0 Self-test invalid
 */
static uint8_t isGyroSelfTestValid(const int16_t self_test_off[kNBaxis], const int16_t self_test_on[kNBaxis]) {
    const float min_selftest_value_dps = 150.0F;
    const float max_selftest_value_dps = 700.0F;

    const float self_test_range_dps[kNBaxis] = {
        [kXaxis] = (float)(self_test_on[kXaxis] - self_test_off[kXaxis]) * kGyroSensitivity2000dps,
        [kYaxis] = (float)(self_test_on[kYaxis] - self_test_off[kYaxis]) * kGyroSensitivity2000dps,
        [kZaxis] = (float)(self_test_on[kZaxis] - self_test_off[kZaxis]) * kGyroSensitivity2000dps,
    };

    return ((self_test_range_dps[kXaxis] >= min_selftest_value_dps) &&
            (self_test_range_dps[kXaxis] <= max_selftest_value_dps) &&
            (self_test_range_dps[kYaxis] >= min_selftest_value_dps) &&
            (self_test_range_dps[kYaxis] <= max_selftest_value_dps) &&
            (self_test_range_dps[kZaxis] >= min_selftest_value_dps) &&
            (self_test_range_dps[kZaxis] <= max_selftest_value_dps));
}
#endif
