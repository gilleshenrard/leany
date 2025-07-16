/**
 * @file memsBMI270.c
 * @author Gilles Henrard
 * @brief Implement the behavior of the BMI270 MEMS
 * @date 12/06/2025
 *
 * @details Datasheet : https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf 
 */

#include "memsBMI270.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "ST7735_initialisation.h"
#include "bmi270.h"
#include "bmi2_defs.h"
#include "errorstack.h"
#include "halspi.h"
#include "main.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "sensorfusion.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include "task.h"

enum {
    STACK_SIZE            = 180U,   ///< Amount of words in the task stack
    TASK_LOW_PRIORITY     = 8U,     ///< FreeRTOS number for a low priority task
    STARTUP_TIME_MS       = 2U,     ///< Number of milliseconds required after Power On Reset or soft reset
    TIMEOUT_MS            = 1000U,  ///< Max number of milliseconds to wait for the device ID
    CONFIG_TIMEOUT_MS     = 20U,    ///< Max number of milliseconds after which the configuration should be ok
    CONFIGFILE_SIZE       = 8192U,  ///< Size of the config file in bytes
    NB_POSITION_REGISTERS = 16U,    ///< Number of registers holding Accelerometer and Gyroscope data
    NB_TEMP_REGISTERS     = 2U,     ///< Number of registers holding Temperature data
    TEMP_REFRESH_MS       = 10U,    ///< Number of milliseconds between temperature value refreshes
};

/**
 * Enumeration of the IDs of the functions used by the BMI270 implementation
 */
typedef enum {
    BMI270_TASK                = 1,   ///< taskBMI270() : Function running the BMI270 state machine
    BMI270_READ_REGISTERS      = 2,   ///< readRegisters() : Function reading several registers
    BMI270_WRITE_REGISTERS     = 3,   ///< burstWriteRegisters() : Function writing multiple registers
    BMI270_STATE_STARTUP       = 4,   ///< stateStartup() : State in which the startup time is waited for
    BMI270_STATE_INITIALISING  = 5,   ///< stateInitialising() : State in which the BMI270 is being initialised
    BMI270_STATE_ERROR         = 6,   ///< stateError() : Error state
    BMI270_CHECK_COMMUNICATION = 7,   ///< checkSPICommunication() : Check the SPI communication with the BMI270
    BMI270_CHECK_CONFIGURATION = 8,   ///< checkConfigurationFile() : Check if written configuration file is correct
    BMI270_STATE_CONFIGURING   = 9,   ///< stateConfiguring() : State in which the BMI270 is configured for the app.
    BMI270_STATE_MEASURING     = 10,  ///< stateMeasuring() : State in which measurements are received from the BMI270
    BMI270_READ_TEMPERATURE    = 11,  ///< readTemperature() : Function reading the BMI270 internal temp. registers
    BMI270_STATE_SELFTEST_ACC  = 12,  ///< stateSelfTestingAccelerometer() : State in which the chip self-tests the acc.
    BMI270_STATE_SELFTEST_GYRO = 13,  ///< stateSelfTestingGyro() : State in which the BMI270 self-tests the gyroscope
    BMI270_RUN_ACCSELFTEST     = 14,  ///< runAccelerometerSelfTest() : Run the self-test command on the BMI270
} BMI270function_e;

/**
 * @brief Union regrouping 8-bits and 16-bits arrays
 * @details
 *  This allows reading all the accelerometer and gyroscope values at once, and convert them to 16 bit instantly
 */
typedef union {
    uint8_t registers8bits[NB_POSITION_REGISTERS];                   ///< 8-bits registers array
    int16_t values16bits[((uint8_t)(NB_POSITION_REGISTERS) >> 1U)];  ///< 16-bits values array
} rawValues_u;

_Static_assert(((uint8_t)NB_POSITION_REGISTERS & (NB_POSITION_REGISTERS - 1U)) == 0,
               "NB_POSITION_REGISTERS must be an even number");

typedef uint8_t BMI270register_t;  ///< type definition for BMI270 registers

// Read/Write bit value
static const BMI270register_t BMI_WRITE          = 0x00U;        ///< Address byte value for a write operation
static const BMI270register_t BMI_READ           = 0x80U;        ///< Address byte value for a read operation
static const float RADIANS_TO_DEGREES_TENTHS     = 572.957795F;  ///< One radian in tenths of degrees (= 10 * (180°/PI))
static const float DEGREES_TENTHS_TO_RADIANS     = 0.001745329F;  ///< One tenth of degree in radians (= (180°/PI) / 10)
const uint32_t     MAX_SENSORTIME_TICKS          = 0xFFFFFFU;  ///< The maximum tick value before it wraps around to 0
const float        SENSORTIME_RESOLUTION_SECONDS = 0.0000390625F;  ///< How many seconds a tick lasts

// Utility functions
static errorCode_u     checkSPICommunication(void);
static errorCode_u     checkConfigurationFile(void);
static errorCode_u     readTemperature(float* temperature_celsius);
static errorCode_u     runAccelerometerSelfTest(rawValues_u* valuesRead, uint8_t positiveSign);
static errorCode_u     runGyroscopeSelfTest(void);
static inline uint32_t toTicks(const uint8_t registers[3]);
static void applyTemperatureDrift(float temperature_celcius, float accelerometer_G[3], float gyroscope_radps[3]);

// State machine functions
static void        taskBMI270(void* argument);
static errorCode_u stateStartup(void);
static errorCode_u stateSelfTestingAccelerometer(void);
static errorCode_u stateSelfTestingGyroscope(void);
static errorCode_u stateInitialising(void);
static errorCode_u stateConfiguring(void);
static errorCode_u stateMeasuring(void);
static void        stateError(void);

/**
 * Accelerometer nominal sensitivity ratio from LSB to G, at 2G range and 25°C
 * @details Equation :
 * Value(G) = Value(LSB) / 16384(LSB/G)
 *
 * See datasheet p. 12, section "Sensitivity"
 */
static const float ACC_NOMINAL_LSB_TO_G = (1.0F / (float)BMI2_ACC_FOC_2G_REF);

/**
 * Accelerometer nominal sensitivity ratio from LSB to G, at 16G range and 25°C
 * @details Equation :
 * Value(G) = Value(LSB) / 2048(LSB/G)
 *
 * See datasheet p. 12, section "Sensitivity"
 */
static const float ACC_NOMINAL_LSB_TO_16G = (1.0F / (float)BMI2_ACC_FOC_16G_REF);

/**
 * Gyroscope nominal sensitivity ratio from LSB to rad/s, at 125°/s range and 25°C
 * @details Equation :
 * Value(rad/s) = (Value(LSB) / 262,144(LSB/dps)) * (PI/180°)
 *
 * See datasheet p. 14, section "Sensitivity"
 */
static const float GYR_NOMINAL_LSB_TO_RADPS = (1.0F / 262.144F) * 0.017453293F;

/**
 * Accelerometer temperature drift in [%/K], divided by 100 because percents
 */
static const float ACC_TEMPDRIFT_100PERCENT_PER_KELVIN = (0.004F / 100.0F);

/**
 * Gyroscope temperature drift in [%/K], divided by 100 because percents
 */
static const float GYR_TEMPDRIFT_100PERCENT_PER_KELVIN = (0.02F / 100.0F);

/**
 * Temperature in [°C] at which the internal temperature value read is 0x0000
 */
static const float REFERENCE_TEMPERATURE_CELSIUS = 23.0F;

//state variables
static volatile TaskHandle_t taskHandle    = NULL;  ///< handle of the FreeRTOS task
static SemaphoreHandle_t     anglesMutex   = NULL;  ///< handle of the mutex used to protect angles measurements
static BMI270function_e      state         = BMI270_STATE_STARTUP;  ///< State machine current state
static uint8_t               resetOccurred = 1;  ///< Flag used to ensure config is written only once after reset
static errorCode_u           result;             ///< Variable used to store error codes
static mahonycontext_t       filterContext = {
          .KI = INTEGRAL_GAIN,
          .KP = PROPORTIONAL_GAIN,
          .dt = {.maxTick           = MAX_SENSORTIME_TICKS,
                 .resolutionSeconds = SENSORTIME_RESOLUTION_SECONDS}
};  ///< Current Mahony filter context
extern const uint8_t    bmi270_config_file[];  ///< BMI270 config file provided by Bosch Sensortec, declared in bmi270.c
static float            anglesAtZeroing_rad[NB_AXIS - 1] = {0, 0};  ///< Latest angles measured and filtered in [rad]
static spi_t            spiDescriptor                    = {.handle                = SPI1,
                                                            .CSport                = BMI270_CS_GPIO_Port,
                                                            .pin                   = BMI270_CS_Pin,
                                                            .highestRegisterNumber = BMI2_CMD_REG_ADDR,
                                                            .readMask              = BMI_READ,
                                                            .writeMask             = BMI_WRITE};  ///< Descriptor of the SPI port used
static volatile uint8_t taskNotifiable = 0;  ///< Flag used to avoid notifying the BMI270 task if it's not ready

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * @brief BMI270 INT1 and INT2 GPIO pins interrupt handler
 * 
 * @param interruptPin Number of the pin triggered (1 or 2)
 */
void bmi270InterruptTriggered(uint8_t interruptPin) {
    BaseType_t hasWoken = 0;

    // vPortEnterCritical();

    //if INT1, notify the BMI270 task
    // (the event of concurrent access to bmi270configured will not be of consequence)
    if(taskNotifiable && (interruptPin == 1)) {
        vTaskNotifyGiveFromISR(taskHandle, &hasWoken);
    }

    // vPortExitCritical();

    portYIELD_FROM_ISR(hasWoken);
}

/**
 * Create a BMI270 FreeRTOS static task
 */
void createBMI270Task(void) {
    static StackType_t       taskStack[STACK_SIZE] = {0};  ///< Buffer used as the task stack
    static StaticTask_t      taskState             = {0};  ///< Task state variables
    static StaticSemaphore_t measureMutexState     = {0};  ///< ADC value mutex state variables

    LL_SPI_Enable(spiDescriptor.handle);

    //create the static task
    taskHandle =
        xTaskCreateStatic(taskBMI270, "BMI270 task", STACK_SIZE, NULL, TASK_LOW_PRIORITY, taskStack, &taskState);
    if(!taskHandle) {
        Error_Handler();
    }

    //create a semaphore to protect mearurements
    anglesMutex = xSemaphoreCreateMutexStatic(&measureMutexState);
    if(!anglesMutex) {
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
static errorCode_u checkSPICommunication(void) {
    BMI270register_t deviceID = 0;

    //attempt to read a correct device ID for max. 1 second
    uint32_t firstTick = HAL_GetTick();
    do {
        //store the bitwise-NOT value of the chip ID to make sure every bit is changed by the read command
        deviceID = (BMI270register_t) ~((BMI270register_t)BMI270_CHIP_ID);

        //read the register
        result = readRegisters(&spiDescriptor, BMI2_CHIP_ID_ADDR, &deviceID, 1);
        if(isError(result)) {
            return (pushErrorCode(result, BMI270_CHECK_COMMUNICATION, 1));
        }
    } while((deviceID != BMI270_CHIP_ID) && !timeout(firstTick, TIMEOUT_MS));

    //check the device ID
    if(deviceID != BMI270_CHIP_ID) {
        return (createErrorCode(BMI270_CHECK_COMMUNICATION, 2, ERR_CRITICAL));
    }

    return ERR_SUCCESS;
}

/**
 * Read the configuration file stored in the BMI270 and make sure it corresponds to the one provided by Bosch
 * 
 * @retval 0 Success
 * @retval 1 Timeout while reading the configuration file
 * @retval 2 Data difference between configuration files
 */
static errorCode_u checkConfigurationFile(void) {
    const uint8_t SPI_RX_FILLER = 0xFFU;  ///< Value to send as a filler while receiving multiple bytes

    //set timeout timer and enable CS
    uint32_t SPIstartTick = HAL_GetTick();
    LL_GPIO_ResetOutputPin(BMI270_CS_GPIO_Port, BMI270_CS_Pin);

    //send the config data read request and receive a dummy byte
    (void)receiveSPIbyte(&spiDescriptor, BMI2_INIT_DATA_ADDR, SPIstartTick);
    (void)receiveSPIbyte(&spiDescriptor, SPI_RX_FILLER, SPIstartTick);

    //receive the bytes to read
    uint8_t  failure   = 0;
    uint16_t byteIndex = 0;
    do {
        BMI270register_t value = receiveSPIbyte(&spiDescriptor, SPI_RX_FILLER, SPIstartTick);
        failure                = (value != bmi270_config_file[byteIndex]);
        byteIndex++;
    } while(!failure && (byteIndex < CONFIGFILE_SIZE) && !timeout(SPIstartTick, SPI_TIMEOUT_MS));

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(spiDescriptor.handle) && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {};
    LL_SPI_ClearFlag_OVR(spiDescriptor.handle);

    //disable CS
    LL_GPIO_SetOutputPin(BMI270_CS_GPIO_Port, BMI270_CS_Pin);

    //if timeout, error
    if(timeout(SPIstartTick, SPI_TIMEOUT_MS)) {
        return (createErrorCode(BMI270_CHECK_CONFIGURATION, 1, ERR_WARNING));
    }

    if(failure || (byteIndex < CONFIGFILE_SIZE)) {
        return (createErrorCode(BMI270_CHECK_CONFIGURATION, 2, ERR_WARNING));
    }

    return ERR_SUCCESS;
}

/**
 * Read the BMI270 internal temperature
 * 
 * @param[out] temperature_celsius Temperature in [°C] to update if the temperature read is valid
 * @retval 0 Success
 * @retval 1 Error while reading the registers
 */
static errorCode_u readTemperature(float* temperature_celsius) {
    int16_t readBuffer = 0x0000;

    //read temperature value
    result = readRegisters(&spiDescriptor, BMI2_TEMPERATURE_0_ADDR, (uint8_t*)&readBuffer, NB_TEMP_REGISTERS);
    if(isError(result)) {
        return (pushErrorCode(result, BMI270_READ_TEMPERATURE, 1));
    }

    //check if the temperature LSB read is valid
    const int16_t INVALID_TEMPERATURE_LSB = (int16_t)0x8000;
    if(readBuffer == INVALID_TEMPERATURE_LSB) {
        return ERR_SUCCESS;
    }

    //transform temperature LSB to °C
    const float KELVIN_PER_LSB = 0.001953125F;
    *temperature_celsius       = REFERENCE_TEMPERATURE_CELSIUS + ((float)readBuffer * KELVIN_PER_LSB);

    return ERR_SUCCESS;
}

/**
 * Send the self-test request to the BMI270 and read its results
 * 
 * @param[out] valuesRead Buffer to which save the read self-test values 
 * @param positiveSign 1 if self-test sign is positive, 0 if negative
 * @retval 0 Success
 * @retval 1 Error while sending the self-test request
 * @retval 2 Error while reading the self-test results
 */
static errorCode_u runAccelerometerSelfTest(rawValues_u* valuesRead, uint8_t positiveSign) {
    //prepare the register value to write to start the self-testing
    BMI270register_t value = (uint8_t)(positiveSign << BMI2_ACC_SELF_TEST_SIGN_POS);
    value |= (1U << BMI2_ACC_SELF_TEST_AMP_POS);
    value |= BMI2_ACC_SELF_TEST_EN_MASK;

    //write the self-test request and wait for it to finish
    result = writeRegisters(&spiDescriptor, BMI2_ACC_SELF_TEST_ADDR, &value, 1U);
    if(isError(result)) {
        return pushErrorCode(result, BMI270_RUN_ACCSELFTEST, 1);
    }
    vTaskDelay(pdMS_TO_TICKS(51U));

    //read all accelerometer/gyroscope values
    result = readRegisters(&spiDescriptor, BMI2_ACC_X_LSB_ADDR, valuesRead->registers8bits, NB_POSITION_REGISTERS);
    if(isError(result)) {
        return (pushErrorCode(result, BMI270_RUN_ACCSELFTEST, 2));
    }

    return ERR_SUCCESS;
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
static errorCode_u runGyroscopeSelfTest(void) {
    BMI270register_t value = 0;

    //set the gyro self-test trigger command
    value  = BMI2_G_TRIGGER_CMD;
    result = writeRegisters(&spiDescriptor, BMI2_CMD_REG_ADDR, &value, 1);
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_SELFTEST_GYRO, 1);  // NOLINT(*-magic-numbers)
    }

    //wait for the self-test done flag
    uint32_t startTick = HAL_GetTick();
    do {
        result = readRegisters(&spiDescriptor, BMI2_GYR_SELF_TEST_AXES_ADDR, &value, 1);
    } while(!isError(result) && !timeout(startTick, TIMEOUT_MS)
            && ((value & BMI2_ACC_SELF_TEST_DONE_MASK) != BMI2_ACC_SELF_TEST_DONE_MASK));

    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_SELFTEST_GYRO, 2);  // NOLINT(*-magic-numbers)
    }

    //check if all axis are flagged as ok
    const BMI270register_t GYR_GAIN_STATUS_ADDR = 0x38;
    const BMI270register_t allGyroscopeOK =
        (BMI2_ACC_X_OK_MASK | BMI2_ACC_Y_OK_MASK | BMI2_ACC_Z_OK_MASK | BMI2_ACC_SELF_TEST_DONE_MASK);
    if((value & allGyroscopeOK) != allGyroscopeOK) {
        BMI270register_t triggerStatus = 0;
        result                         = readRegisters(&spiDescriptor, GYR_GAIN_STATUS_ADDR, &triggerStatus, 1);
        if(isError(result)) {
            return pushErrorCode(result, BMI270_STATE_SELFTEST_GYRO, 3);  // NOLINT(*-magic-numbers)
        }
        state  = BMI270_STATE_ERROR;
        result = createErrorCodeLayer1(BMI270_STATE_SELFTEST_GYRO, value, triggerStatus, ERR_CRITICAL);
        return pushErrorCode(result, BMI270_STATE_SELFTEST_GYRO, 4);  // NOLINT(*-magic-numbers)
    }

    return ERR_SUCCESS;
}

/**
 * @brief Transpose a measurement to an angle in tenths of degrees with the Z axis
 *
 * @param axis Axis for which get the angle with the Z axis
 * @return Angle with the Z axis
 */
int16_t getAngleDegreesTenths(axis_e axis) {
    if(xSemaphoreTake(anglesMutex, pdMS_TO_TICKS(SPI_TIMEOUT_MS)) == pdFALSE) {
        return 0;
    }
    float currentAngle_rad = angleAlongAxis(&filterContext, axis);
    xSemaphoreGive(anglesMutex);

    float total = (currentAngle_rad + anglesAtZeroing_rad[axis]);
    return (int16_t)(total * RADIANS_TO_DEGREES_TENTHS);
}

/**
 * @brief Check if angle measurements have changed
 *
 * @retval 0 No new values available
 * @retval 1 New values are available
 */
uint8_t anglesChanged(void) {
    if(xSemaphoreTake(anglesMutex, pdMS_TO_TICKS(SPI_TIMEOUT_MS)) == pdFALSE) {
        return 0;
    }
    const float currentAngle_rad = getAttitudeAngle(&filterContext);
    xSemaphoreGive(anglesMutex);

    //check if angle changed above minimum threshold (0.1° in any direction)
    static float previousAngle_rad = 0.0F;
    uint8_t      changed           = (fabsf(previousAngle_rad - currentAngle_rad) > DEGREES_TENTHS_TO_RADIANS);
    if(changed) {
        previousAngle_rad = currentAngle_rad;
    }

    return (changed);
}

/**
 * @brief Set the measurements in relative mode and zero down the values
 */
void bmi270ZeroDown(void) {
    if(xSemaphoreTake(anglesMutex, pdMS_TO_TICKS(SPI_TIMEOUT_MS)) == pdFALSE) {
        return;
    }

    anglesAtZeroing_rad[X_AXIS] = -angleAlongAxis(&filterContext, X_AXIS);
    anglesAtZeroing_rad[Y_AXIS] = -angleAlongAxis(&filterContext, Y_AXIS);

    xSemaphoreGive(anglesMutex);
}

/**
 * @brief Set the measurements in absolute mode (no zeroing compensation)
 */
void bmi270CancelZeroing(void) {
    if(xSemaphoreTake(anglesMutex, pdMS_TO_TICKS(SPI_TIMEOUT_MS)) == pdFALSE) {
        return;
    }

    anglesAtZeroing_rad[X_AXIS] = 0.0F;
    anglesAtZeroing_rad[Y_AXIS] = 0.0F;

    xSemaphoreGive(anglesMutex);
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
 * @param[out] accelerometer_G Accelerometer values in [G]
 * @param[out] gyroscope_radps Gyroscope values in [rad/s]
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
static void applyTemperatureDrift(const float temperature_celcius, float accelerometer_G[3], float gyroscope_radps[3]) {
    const float delta_celsius      = temperature_celcius - REFERENCE_TEMPERATURE_CELSIUS;
    const float accelDrift_percent = (delta_celsius * ACC_TEMPDRIFT_100PERCENT_PER_KELVIN);
    const float gyroDrift_percent  = (delta_celsius * GYR_TEMPDRIFT_100PERCENT_PER_KELVIN);

    for(uint8_t axis = 0; axis < (uint8_t)NB_AXIS; axis++) {
        accelerometer_G[axis] += (accelDrift_percent * accelerometer_G[axis]);
        gyroscope_radps[axis] += (gyroDrift_percent * gyroscope_radps[axis]);
    }
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

    while(1) {
        switch(state) {
            case BMI270_STATE_STARTUP:
                result = stateStartup();
                break;

            case BMI270_STATE_SELFTEST_ACC:
                result = stateSelfTestingAccelerometer();
                break;

            case BMI270_STATE_SELFTEST_GYRO:
                result = stateSelfTestingGyroscope();
                break;

            case BMI270_STATE_INITIALISING:
                result = stateInitialising();
                break;

            case BMI270_STATE_CONFIGURING:
                result = stateConfiguring();
                break;

            case BMI270_STATE_MEASURING:
                result = stateMeasuring();
                break;

            case BMI270_STATE_ERROR:
                stateError();
                break;

            case BMI270_READ_REGISTERS:
            case BMI270_WRITE_REGISTERS:
            case BMI270_CHECK_COMMUNICATION:
            case BMI270_CHECK_CONFIGURATION:
            case BMI270_READ_TEMPERATURE:
            case BMI270_TASK:
            case BMI270_RUN_ACCSELFTEST:
            default:
                result = createErrorCode(BMI270_TASK, 0, ERR_CRITICAL);
                break;
        }

        if(isError(result)) {
            state = BMI270_STATE_ERROR;
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
static errorCode_u stateStartup(void) {
    BMI270register_t value = 0;

    //wait for a while for the sensor to boot properly, and enable SPI
    taskNotifiable = 0;
    vTaskDelay(pdMS_TO_TICKS(STARTUP_TIME_MS));

    result = checkSPICommunication();
    if(isError(result)) {
        return (pushErrorCode(result, BMI270_STATE_STARTUP, 1));
    }

    //issue a soft reset command to reset all registers to their default value
    // (useful when debugging and the supply voltage is not interrupted)
    value  = BMI2_SOFT_RESET_CMD;
    result = writeRegisters(&spiDescriptor, BMI2_CMD_REG_ADDR, &value, 1);
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_STARTUP, 2);
    }

    //wait for a while for the sensor to reset properly, and read a register to enable SPI
    vTaskDelay(pdMS_TO_TICKS(STARTUP_TIME_MS));
    result = checkSPICommunication();
    if(isError(result)) {
        return (pushErrorCode(result, BMI270_STATE_STARTUP, 3));
    }

    resetOccurred = 1;
    state         = BMI270_STATE_INITIALISING;
    return ERR_SUCCESS;
}

/**
 * State in which the BMI270 is initialised, following the POR procedure in the datasheet
 *
 * @details This follows the procedure descripted in the datasheet :
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf#%5B%7B%22num%22%3A64%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C33%2C771%2C0%5D
 * 
 * @retval 0 Success
 * @retval 1 Error while disabling advanced power mode
 * @retval 2 Error while requesting config initialisation start
 * @retval 3 Error while writing the config file
 * @retval 4 Error while checking the config file written
 * @retval 5 Error while requesting config initialisation finish
 * @retval 6 Error while reading the internal status
 * @retval 7 Invalid internal status
 */
static errorCode_u stateInitialising(void) {
    const BMI270register_t START_CONFIGFILE_LOAD  = 0x00U;
    const BMI270register_t FINISH_CONFIGFILE_LOAD = 0x01U;
    BMI270register_t       value                  = 0;

    //Disable advanced power mode
    value  = 0x00;
    result = writeRegisters(&spiDescriptor, BMI2_PWR_CONF_ADDR, &value, 1);
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_INITIALISING, 1);
    }

    //Getting out of advanced power mode takes up to 450us
    vTaskDelay(pdMS_TO_TICKS(1U));

    //make sure config file is written only once after POR or soft reset
    if(!resetOccurred) {
        state = BMI270_STATE_SELFTEST_ACC;
        return ERR_SUCCESS;
    }
    resetOccurred = 0;

    //Request configuration start
    value  = START_CONFIGFILE_LOAD;
    result = writeRegisters(&spiDescriptor, BMI2_INIT_CTRL_ADDR, &value, 1);
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_INITIALISING, 2);
    }

    //Write the config file provided by Bosch Sensortec
    result = writeRegisters(&spiDescriptor, BMI2_INIT_DATA_ADDR, bmi270_config_file, CONFIGFILE_SIZE);
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_INITIALISING, 3);
    }

    //re-read the configuration file written to make sure it is correct (max. 3 attempts)
    uint8_t attempts = 3;
    do {
        result = checkConfigurationFile();
    } while((--attempts) && isError(result));
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_INITIALISING, 4);  // NOLINT(*-magic-numbers)
    }

    //Request configuration finish
    value  = FINISH_CONFIGFILE_LOAD;
    result = writeRegisters(&spiDescriptor, BMI2_INIT_CTRL_ADDR, &value, 1);
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_INITIALISING, 5);  // NOLINT(*-magic-numbers)
    }

    //Applying the new config takes up to 20ms
    vTaskDelay(pdMS_TO_TICKS(CONFIG_TIMEOUT_MS));

    //Wait for the initialisation status register to show a success or an error
    uint32_t configStartTick = HAL_GetTick();
    do {
        result = readRegisters(&spiDescriptor, BMI2_INTERNAL_STATUS_ADDR, &value, 1);
        if(isError(result)) {
            return pushErrorCode(result, BMI270_STATE_INITIALISING, 6);  // NOLINT(*-magic-numbers)
        }
    } while((value == BMI2_NOT_INIT) && !timeout(configStartTick, CONFIG_TIMEOUT_MS));

    //check if an initialisation error occurred
    if(value != BMI2_INIT_OK) {
        return createErrorCodeLayer1(BMI270_STATE_INITIALISING, 7, value, ERR_CRITICAL);  // NOLINT(*-magic-numbers)
    }

    state = BMI270_STATE_SELFTEST_ACC;
    return ERR_SUCCESS;
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
static errorCode_u stateSelfTestingAccelerometer(void) {
    const register_t BMI2_ACC_RANGE_ADDR = 0x41U;  ///< Accelerometer range value register number
    BMI270register_t value               = 0;

    // clang-format off
    const BMI270register_t configuration[][2] = {
        {BMI2_PWR_CTRL_ADDR,                      (BMI2_ACC_EN_MASK)}, //enable temp, gyroscope and accel.
        {BMI2_ACC_RANGE_ADDR,                     BMI2_ACC_RANGE_16G}, //set the accel. range to +-16G
        {BMI2_ACC_CONF_ADDR,
            (1U << BMI2_ACC_FILTER_PERF_MODE_POS)
                | (BMI270register_t)(BMI2_ACC_NORMAL_AVG4 << BMI2_ACC_BW_PARAM_POS)
                | BMI2_ACC_ODR_1600HZ},
    };
    // clang-format on

    //write the preliminary self-test registers
    BMI270register_t reg         = 0;
    const uint8_t    nbRegisters = sizeof(configuration) / sizeof(configuration)[0];
    do {
        result = writeRegisters(&spiDescriptor, configuration[reg][0], &configuration[reg][1], 1);
        reg++;
    } while((reg < nbRegisters) && !isError(result));
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_SELFTEST_ACC, 1);
    }

    //wait for 2ms
    vTaskDelay(pdMS_TO_TICKS(3U));

    rawValues_u LSBvaluesPositive = {0};  ///< Buffer holding the LSB values read from the IC
    rawValues_u LSBvaluesNegative = {0};  ///< Buffer holding the LSB values read from the IC

    result = runAccelerometerSelfTest(&LSBvaluesPositive, 1);
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_SELFTEST_ACC, 2);
    }
    result = runAccelerometerSelfTest(&LSBvaluesNegative, 0);
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_SELFTEST_ACC, 3);
    }

    //disable self-testing
    value  = 0x00;
    result = writeRegisters(&spiDescriptor, BMI2_ACC_SELF_TEST_ADDR, &value, 1U);
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_SELFTEST_ACC, 4U);  // NOLINT(*-magic-numbers)
    }

    //transform the values read to values in [G]
    const float deltaX_G = (float)(LSBvaluesPositive.values16bits[X_AXIS] - LSBvaluesNegative.values16bits[X_AXIS])
                           * ACC_NOMINAL_LSB_TO_16G;
    const float deltaY_G = (float)(LSBvaluesPositive.values16bits[Y_AXIS] - LSBvaluesNegative.values16bits[Y_AXIS])
                           * ACC_NOMINAL_LSB_TO_16G;
    const float deltaZ_G = (float)(LSBvaluesPositive.values16bits[Z_AXIS] - LSBvaluesNegative.values16bits[Z_AXIS])
                           * ACC_NOMINAL_LSB_TO_16G;

    //check if the minimum delta given in the datasheet is respected
    const float MIN_DELTAX_G = 16.0F;
    const float MIN_DELTAY_G = -15.0F;
    const float MIN_DELTAZ_G = 10.0F;
    if((deltaX_G <= MIN_DELTAX_G) || (deltaY_G >= MIN_DELTAY_G) || (deltaZ_G <= MIN_DELTAZ_G)) {
        return createErrorCode(BMI270_STATE_SELFTEST_ACC, 5, ERR_CRITICAL);  // NOLINT(*-magic-numbers)
    }

    state = BMI270_STATE_SELFTEST_GYRO;
    return ERR_SUCCESS;
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
static errorCode_u stateSelfTestingGyroscope(void) {
    uint8_t attempts = 3;
    do {
        result = runGyroscopeSelfTest();
    } while(--attempts && isError(result));

    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_SELFTEST_GYRO, 1);  // NOLINT(*-magic-numbers)
    }

    if(!attempts) {
        return pushErrorCode(result, BMI270_STATE_SELFTEST_GYRO, 2);  // NOLINT(*-magic-numbers)
    }

    state = BMI270_STATE_CONFIGURING;
    return ERR_SUCCESS;
}

/**
 * State in which the BMI270 is configured for the application
 * 
 * @retval 0 Success
 * @retval 1 Error while writing a register
 */
static errorCode_u stateConfiguring(void) {
    const register_t BMI2_ACC_RANGE_ADDR = 0x41U;  ///< Accelerometer range value register number
    const register_t BMI2_GYR_RANGE_ADDR = 0x43U;  ///< Gyroscope range value register number
    const register_t NO_FIFO             = 0x00;   ///< Value used to disable the FIFO

    // clang-format off
    const BMI270register_t configuration[][2] = {
        {BMI2_PWR_CTRL_ADDR,                          (BMI2_GYR_EN_MASK | BMI2_ACC_EN_MASK | BMI2_TEMP_EN_MASK)}, //enable temp, gyroscope and accel.
        {BMI2_ACC_CONF_ADDR, (BMI2_ACC_FILTER_PERF_MODE_MASK | (BMI2_ACC_CIC_AVG8 << 4U) | BMI2_ACC_ODR_1600HZ)}, //
        {BMI2_ACC_RANGE_ADDR,                                                                 BMI2_ACC_RANGE_2G}, //set the accel. range to +-2G
        {BMI2_GYR_CONF_ADDR, (BMI2_GYR_FILTER_PERF_MODE_MASK
                                        | BMI2_GYR_NOISE_PERF_MODE_MASK
                                        | (BMI2_GYR_NORMAL_MODE << 4U)
                                        | BMI2_GYR_ODR_1600HZ)},
        {BMI2_GYR_RANGE_ADDR,                                                                BMI2_GYR_RANGE_125}, //set the gyroscope range to +-500°/s
        {BMI2_FIFO_CONFIG_1_ADDR,                                                                       NO_FIFO}, //disable the FIFO (streaming mode)
        {BMI2_INT_MAP_DATA_ADDR,                                                                  BMI2_DRDY_INT}, //enable Data Ready interrupt on INT1
        {BMI2_INT1_IO_CTRL_ADDR,       BMI2_INT_OUTPUT_EN_MASK | BMI2_INT_OPEN_DRAIN_MASK | BMI2_INT_LEVEL_MASK}, //enable INT1 active high, open drain
        {BMI2_PWR_CONF_ADDR,                                                                               0x00}, //disable advanced power mode
    };
    // clang-format on

    //write all the configuration registers
    BMI270register_t reg         = 0;
    const uint8_t    nbRegisters = sizeof(configuration) / sizeof(configuration)[0];
    do {
        result = writeRegisters(&spiDescriptor, configuration[reg][0], &configuration[reg][1], 1);
        reg++;
    } while((reg < nbRegisters) && !isError(result));

    //if error, exit
    if(isError(result)) {
        return pushErrorCode(result, BMI270_STATE_CONFIGURING, 1);
    }

    resetMahonyFilter(&filterContext);
    taskNotifiable = 1;
    state          = BMI270_STATE_MEASURING;
    return ERR_SUCCESS;
}

/**
 * State in which measurements are received from the BMI270
 * 
 * @retval 0 Success
 * @retval 1 Did not receive any data ready interrupt
 * @retval 2 Error while reading the data registers
 * @retval 3 Error while reading the temperature registers
 */
static errorCode_u stateMeasuring(void) {
    float       accelerometer_G[NB_AXIS];    ///< Accelerometer values in [G]
    float       gyroscope_radps[NB_AXIS];    ///< Gyroscope values in [rad/s]
    rawValues_u LSBvalues           = {0};   ///< Buffer holding the LSB values read from the IC
    float       temperature_celsius = 0.0F;  ///< Latest internal temperature in [°C]

    //wait for measurements to be ready
    if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TIMEOUT_MS)) == pdFALSE) {
        return (createErrorCode(BMI270_STATE_MEASURING, 1, ERR_CRITICAL));
    }

    //read all accelerometer/gyroscope values
    result = readRegisters(&spiDescriptor, BMI2_ACC_X_LSB_ADDR, LSBvalues.registers8bits, NB_POSITION_REGISTERS);
    if(isError(result)) {
        return (pushErrorCode(result, BMI270_STATE_MEASURING, 2));
    }

    //apply the nominal sensitivity to the measurements to change them to usable units
    for(uint8_t axis = 0; axis < (uint8_t)NB_AXIS; axis++) {
        accelerometer_G[axis] = (float)(LSBvalues.values16bits[axis]) * ACC_NOMINAL_LSB_TO_G;
        gyroscope_radps[axis] = (float)(LSBvalues.values16bits[axis + NB_AXIS]) * GYR_NOMINAL_LSB_TO_RADPS;
    }

    //read the BMI270 temperature every 10ms
    static TickType_t latestTemperatureTick = 0;
    if(timeout(latestTemperatureTick, TEMP_REFRESH_MS)) {
        latestTemperatureTick = HAL_GetTick();

        result = readTemperature(&temperature_celsius);
        if(isError(result)) {
            return (pushErrorCode(result, BMI270_STATE_MEASURING, 3));
        }
    }

    //take temperature drift into account with sensitivity
    applyTemperatureDrift(temperature_celsius, accelerometer_G, gyroscope_radps);

    //update the current sensor time ticks
    const uint8_t NB_MEASURE_REGISTERS = (uint8_t)NB_AXIS << 2U;
    filterContext.dt.currentTick       = toTicks(&LSBvalues.registers8bits[NB_MEASURE_REGISTERS]);

    //apply sensor fusion to the measurements
    if(xSemaphoreTake(anglesMutex, pdMS_TO_TICKS(SPI_TIMEOUT_MS)) == pdTRUE) {
        updateMahonyFilter(&filterContext, accelerometer_G, gyroscope_radps);
        xSemaphoreGive(anglesMutex);
    }

    return ERR_SUCCESS;
}

/**
 * State in which BMI270 is in error
 */
static void stateError(void) {
}
