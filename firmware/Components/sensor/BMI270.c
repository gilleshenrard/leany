/**
 * @file BMI270.c
 * @author Gilles Henrard
 * @brief Implement the behavior of the BMI270 MEMS
 * @date 23/02/2025
 *
 * @details Datasheet : https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf 
 */

#include "BMI270.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "ST7735_initialisation.h"
#include "bmi270.h"
#include "bmi2_defs.h"
#include "errorstack.h"
#include "main.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include "task.h"

enum {
    STACK_SIZE           = 128U,   ///< Amount of words in the task stack
    TASK_LOW_PRIORITY    = 8U,     ///< FreeRTOS number for a low priority task
    STARTUP_TIME_MS      = 2U,     ///< Number of milliseconds required after Power On Reset or soft reset
    SPI_TIMEOUT_MS       = 100U,   ///< Number of milliseconds beyond which SPI is in timeout
    TIMEOUT_MS           = 1000U,  ///< Max number of milliseconds to wait for the device ID
    CONFIG_TIMEOUT_MS    = 20U,    ///< Max number of milliseconds after which the configuration should be ok
    CONFIGFILE_SIZE      = 8192U,  ///< Size of the config file in bytes
    NB_REGISTERS_TO_READ = 12U,    ///< Numbers of data registers to read
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
} BMI270function_e;

/**
 * @brief Union regrouping 8-bits and 16-bits arrays
 * @details
 *  This allows reading all the accelerometer and gyroscope values at once, and convert them to 16 bit instantly
 */
typedef union {
    uint8_t registers8bits[NB_REGISTERS_TO_READ];                   ///< 8-bits registers array
    int16_t values16bits[((uint8_t)(NB_REGISTERS_TO_READ) >> 1U)];  ///< 16-bits values array
} rawValues_u;

typedef uint8_t BMI270register_t;

// Read/Write bit value
static const BMI270register_t BMI_WRITE = 0x00U;  ///< Address byte value for a write operation
static const BMI270register_t BMI_READ  = 0x80U;  ///< Address byte value for a read operation

// Utility functions
static __attribute__((always_inline)) inline uint8_t receiveSPIbyte(uint8_t byteToTransmit, uint32_t txStartTick);
static errorCode_u readRegisters(BMI270register_t firstRegister, BMI270register_t value[], size_t size);
static errorCode_u writeRegisters(BMI270register_t registerNumber, const BMI270register_t value[], size_t size);
static errorCode_u checkSPICommunication(void);
static errorCode_u checkConfigurationFile(void);
static void        complementaryFilter(const float accelerometer_mG[], const float gyroscope_radps[],
                                       float filteredAngles_rad[]);

// State machine functions
static void        taskBMI270(void* argument);
static errorCode_u stateStartup(void);
static errorCode_u stateInitialising(void);
static errorCode_u stateConfiguring(void);
static errorCode_u stateMeasuring(void);
static void        stateError(void);

/**
 * Accelerometer sensitivity ratio from LSB to mG, at 2G range
 * @details Equation :
 * Value(mG) = (Value(LSB) / 16384(LSB/mG)) * 1000(mG/G)
 *
 * See datasheet p. 12, section "Sensitivity"
 */
static const float ACC_LSB_TO_MG = (1.0F / (float)BMI2_ACC_FOC_2G_REF) * 1000.0F;

/**
 * Gyroscope sensitivity ratio from LSB to rad/s, at 500°/s range
 * @details Equation :
 * Value(rad/s) = (Value(LSB) / 65,536(LSB/dps)) * (PI/180°)
 *
 * See datasheet p. 14, section "Sensitivity"
 */
static const float GYR_LSB_TO_RADPS = (1.0F / 65.536F) * 0.017453293F;

//state variables
static volatile TaskHandle_t taskHandle  = NULL;      ///< handle of the FreeRTOS task
static SemaphoreHandle_t     anglesMutex = NULL;      ///< handle of the mutex used to protect angles measurements
static SPI_TypeDef*          spiHandle   = (void*)0;  ///< SPI handle used by the LSM6DSO device
static BMI270function_e      state       = BMI270_STATE_STARTUP;  ///< State machine current state
static errorCode_u           result;                              ///< Variable used to store error codes
extern const uint8_t bmi270_config_file[];  ///< BMI270 config file provided by Bosch Sensortec, declared in bmi270.c
static float         latestAngles_rad[NB_AXIS - 1] = {0.0F, 0.0F};  ///< Latest angles measured and filtered in [rad]

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Create a BMI270 FreeRTOS static task
 *
 * @param handle SPI handle used by the BMI270
 */
void createBMI270Task(const SPI_TypeDef* handle) {
    static StackType_t       taskStack[STACK_SIZE] = {0};  ///< Buffer used as the task stack
    static StaticTask_t      taskState             = {0};  ///< Task state variables
    static StaticSemaphore_t measureMutexState     = {0};  ///< ADC value mutex state variables

    //save the SPI handle used by LSM6DSO and disable SPI1
    spiHandle = (SPI_TypeDef*)handle;
    LL_SPI_Enable(spiHandle);

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
 * Receive a byte from the SPI bus
 * 
 * @param byteToTransmit Byte to transmit either as a command or as a filler to keep the SPI clock running
 * @param txStartTick Tick at which the transmission started
 * @return uint8_t Received byte
 */
static __attribute__((always_inline)) inline uint8_t receiveSPIbyte(uint8_t byteToTransmit, uint32_t txStartTick) {
    //send the byte to transmit and wait for the reception to be complete
    LL_SPI_TransmitData8(spiHandle, byteToTransmit);
    while((!LL_SPI_IsActiveFlag_RXNE(spiHandle)) && !timeout(txStartTick, SPI_TIMEOUT_MS)) {};

    //read the received byte (clears the rx buffer) and return it
    return LL_SPI_ReceiveData8(spiHandle);
}

/**
 * Burst read registers on the BMI270
 *
 * @param firstRegister Number of the first register to read
 * @param[out] value Registers value array
 * @param size Number of registers to read
 * @return   Success
 * @retval 1 SPI handle or value buffer NULL
 * @retval 2 Timeout
 */
static errorCode_u readRegisters(BMI270register_t firstRegister, BMI270register_t value[], size_t size) {
    static const uint8_t SPI_RX_FILLER = 0xFFU;  ///< Value to send as a filler while receiving multiple bytes

    //if no bytes to read, success
    if(!size) {
        return ERR_SUCCESS;
    }

    //make sure neither the handle nor the buffer are NULL
    if(!spiHandle || !value) {
        return (createErrorCode(BMI270_READ_REGISTERS, 1, ERR_CRITICAL));
    }

    //set timeout timer and enable CS
    uint32_t SPIstartTick = HAL_GetTick();
    LL_GPIO_ResetOutputPin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin);

    //send the read request and a dummy byte to synchronize the SPI clock
    (void)receiveSPIbyte(BMI_READ | (uint8_t)firstRegister, SPIstartTick);
    (void)receiveSPIbyte(SPI_RX_FILLER, SPIstartTick);

    //receive the bytes to read
    do {
        *value = receiveSPIbyte(SPI_RX_FILLER, SPIstartTick);
        value++;
        size--;
    } while(size && !timeout(SPIstartTick, SPI_TIMEOUT_MS));

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(spiHandle) && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {};
    LL_SPI_ClearFlag_OVR(spiHandle);

    //disable CS
    LL_GPIO_SetOutputPin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin);

    //if timeout, error
    if(timeout(SPIstartTick, SPI_TIMEOUT_MS)) {
        return (createErrorCode(BMI270_READ_REGISTERS, 2, ERR_WARNING));
    }

    return (ERR_SUCCESS);
}

/**
 * Burst write a single register on the BMI270
 *
 * @param registerNumber Register number
 * @param[in] value Value to assign to the register
 * @param size Number of bytes (i.e. register values or data bytes to a single register) to write
 * @return	 Success
 * @retval 1 No SPI handle specified
 * @retval 2 Register number out of range
 * @retval 3 Timeout
 */
static errorCode_u writeRegisters(BMI270register_t registerNumber, const BMI270register_t value[], size_t size) {
    //if no bytes to write, success
    if(!size) {
        return ERR_SUCCESS;
    }

    //if handle not specified, error
    if(!spiHandle) {
        return (createErrorCode(BMI270_WRITE_REGISTERS, 1, ERR_WARNING));
    }

    //if register number above known or within the reserved range, error
    if(registerNumber > BMI2_CMD_REG_ADDR) {
        return (createErrorCode(BMI270_WRITE_REGISTERS, 2, ERR_WARNING));
    }

    //set timeout timer and enable CS
    uint32_t SPIstartTick = HAL_GetTick();
    LL_GPIO_ResetOutputPin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin);

    //send the write instruction
    LL_SPI_TransmitData8(spiHandle, BMI_WRITE | registerNumber);

    //write the value data
    do {
        while(!LL_SPI_IsActiveFlag_TXE(spiHandle) && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {};
        LL_SPI_TransmitData8(spiHandle, *value);
        value++;
        size--;
    } while(size && !timeout(SPIstartTick, SPI_TIMEOUT_MS));

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(spiHandle) && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {};
    LL_SPI_ClearFlag_OVR(spiHandle);

    //disable CS
    LL_GPIO_SetOutputPin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin);

    //if timeout, error
    if(timeout(SPIstartTick, SPI_TIMEOUT_MS)) {
        return (createErrorCode(BMI270_WRITE_REGISTERS, 3, ERR_WARNING));
    }

    return (ERR_SUCCESS);
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
        result = readRegisters(BMI2_CHIP_ID_ADDR, &deviceID, 1);
        if(isError(result)) {
            return (pushErrorCode(result, BMI270_CHECK_COMMUNICATION, 1));
        }
    } while((deviceID != BMI270_CHIP_ID) && !timeout(firstTick, TIMEOUT_MS));

    //check the device ID
    if(deviceID != BMI270_CHIP_ID) {
        state = BMI270_STATE_ERROR;
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
    static const uint8_t SPI_RX_FILLER = 0xFFU;  ///< Value to send as a filler while receiving multiple bytes

    //set timeout timer and enable CS
    uint32_t SPIstartTick = HAL_GetTick();
    LL_GPIO_ResetOutputPin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin);

    //send the config data read request and receive a dummy byte
    (void)receiveSPIbyte(BMI_READ | (uint8_t)BMI2_INIT_DATA_ADDR, SPIstartTick);
    (void)receiveSPIbyte(SPI_RX_FILLER, SPIstartTick);

    //receive the bytes to read
    uint8_t  failure   = 0;
    uint16_t byteIndex = 0;
    do {
        BMI270register_t value = receiveSPIbyte(SPI_RX_FILLER, SPIstartTick);
        failure                = (value != bmi270_config_file[byteIndex]);
        byteIndex++;
    } while(!failure && (byteIndex < CONFIGFILE_SIZE) && !timeout(SPIstartTick, SPI_TIMEOUT_MS));

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(spiHandle) && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {};
    LL_SPI_ClearFlag_OVR(spiHandle);

    //disable CS
    LL_GPIO_SetOutputPin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin);

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
 * @brief Compute a complementary filter on accelerometer/gyroscope values
 * 
 * @param[in] accelerometer_mG    Array of acceleration values in [mG] on all axis
 * @param[in] gyroscope_radps       Array of gyroscope values  in [rad/s] on X and Y axis
 * @param[out] filteredAngles_rad     Array of final angle values in [rad] on X and Y axis
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void complementaryFilter(const float accelerometer_mG[], const float gyroscope_radps[], float filteredAngles_rad[]) {
    static const float alpha          = 0.02F;  ///< Proportion applied to the gyro. and accel. in the final result
    static const float dtPeriod_sec   = 0.00240385F;  ///< Time period between two updates (LSM6DSO config. at 416Hz)
    static const float GRAVITATION_MG = 1000.0F;      ///< Grativation value in mG
    float              AccelEstimatedX_rad = 0.0F;    ///< Estimated accelerator angle on the X axis in [rad]
    float              AccelEstimatedY_rad = 0.0F;    ///< Estimated accelerator angle on the Y axis in [rad]
    float eulerAngleRateX_radps = 0.0F;  ///< Euler angle rate (with reference to Earth) around X axis in rad/s
    float eulerAngleRateY_radps = 0.0F;  ///< Euler angle rate (with reference to Earth) around Y axis in rad/s

    //calculate the accelerometer angle estimations
    AccelEstimatedX_rad = asinf(accelerometer_mG[X_AXIS] / GRAVITATION_MG);
    AccelEstimatedY_rad = atanf(accelerometer_mG[Y_AXIS] / accelerometer_mG[Z_AXIS]);

    //take the measurements mutex before updating angle values
    if(xSemaphoreTake(anglesMutex, pdMS_TO_TICKS(SPI_TIMEOUT_MS)) == pdFALSE) {
        return;
    }

    //Transform gyroscope rates (reference is the solid body) to Euler rates (reference is Earth)
    eulerAngleRateX_radps =
        gyroscope_radps[X_AXIS]
        + (sinf(filteredAngles_rad[X_AXIS]) * tanf(filteredAngles_rad[Y_AXIS]) * gyroscope_radps[Y_AXIS])
        + (cosf(filteredAngles_rad[X_AXIS]) * tanf(filteredAngles_rad[Y_AXIS]) * gyroscope_radps[Z_AXIS]);

    eulerAngleRateY_radps = (cosf(filteredAngles_rad[X_AXIS]) * gyroscope_radps[Y_AXIS])
                            - (sinf(filteredAngles_rad[X_AXIS]) * gyroscope_radps[Z_AXIS]);

    //combine accelerometer estimates with Euler angle rates estimates
    filteredAngles_rad[X_AXIS] =
        ((1.0F - alpha) * (filteredAngles_rad[X_AXIS] + (eulerAngleRateX_radps * dtPeriod_sec)))
        + (alpha * AccelEstimatedX_rad);
    filteredAngles_rad[Y_AXIS] =
        ((1.0F - alpha) * (filteredAngles_rad[Y_AXIS] + (eulerAngleRateY_radps * dtPeriod_sec)))
        + (alpha * AccelEstimatedY_rad);

    //release the mutex
    xSemaphoreGive(anglesMutex);
}

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Run the BMI270 state machine
 * 
 * @param argument Unused
 */
static void taskBMI270(void* argument) {
    UNUSED(argument);

    while(1) {
        switch(state) {
            case BMI270_STATE_STARTUP:
                result = stateStartup();
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
            case BMI270_TASK:
            default:
                result = createErrorCode(BMI270_TASK, 0, ERR_CRITICAL);
                break;
        }

        if(isError(result)) {
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
    vTaskDelay(pdMS_TO_TICKS(STARTUP_TIME_MS));

    result = checkSPICommunication();
    if(isError(result)) {
        return (pushErrorCode(result, BMI270_STATE_STARTUP, 1));
    }

    //issue a soft reset command to reset all registers to their default value
    // (useful when debugging and the supply voltage is not interrupted)
    value  = BMI2_SOFT_RESET_CMD;
    result = writeRegisters(BMI2_CMD_REG_ADDR, &value, 1);
    if(isError(result)) {
        state = BMI270_STATE_ERROR;
        return pushErrorCode(result, BMI270_STATE_STARTUP, 2);
    }

    //wait for a while for the sensor to reset properly, and enable SPI
    vTaskDelay(pdMS_TO_TICKS(STARTUP_TIME_MS));

    result = checkSPICommunication();
    if(isError(result)) {
        return (pushErrorCode(result, BMI270_STATE_STARTUP, 3));
    }

    state = BMI270_STATE_INITIALISING;
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
    static const BMI270register_t START_CONFIGFILE_LOAD  = 0x00U;
    static const BMI270register_t FINISH_CONFIGFILE_LOAD = 0x01U;
    BMI270register_t              value                  = 0;

    //Disable advanced power mode
    value  = 0x00;
    result = writeRegisters(BMI2_PWR_CONF_ADDR, &value, 1);
    if(isError(result)) {
        state = BMI270_STATE_ERROR;
        return pushErrorCode(result, BMI270_STATE_INITIALISING, 1);
    }

    //Getting out of advanced power mode takes up to 450us
    vTaskDelay(pdMS_TO_TICKS(1U));

    //Request configuration start
    value  = START_CONFIGFILE_LOAD;
    result = writeRegisters(BMI2_INIT_CTRL_ADDR, &value, 1);
    if(isError(result)) {
        state = BMI270_STATE_ERROR;
        return pushErrorCode(result, BMI270_STATE_INITIALISING, 2);
    }

    //Write the config file provided by Bosch Sensortec
    result = writeRegisters(BMI2_INIT_DATA_ADDR, bmi270_config_file, CONFIGFILE_SIZE);
    if(isError(result)) {
        state = BMI270_STATE_ERROR;
        return pushErrorCode(result, BMI270_STATE_INITIALISING, 3);
    }

    //re-read the configuration file written to make sure it is correct
    result = checkConfigurationFile();
    if(isError(result)) {
        state = BMI270_STATE_ERROR;
        return pushErrorCode(result, BMI270_STATE_INITIALISING, 4);  // NOLINT(*-magic-numbers)
    }

    //Request configuration finish
    value  = FINISH_CONFIGFILE_LOAD;
    result = writeRegisters(BMI2_INIT_CTRL_ADDR, &value, 1);
    if(isError(result)) {
        state = BMI270_STATE_ERROR;
        return pushErrorCode(result, BMI270_STATE_INITIALISING, 5);  // NOLINT(*-magic-numbers)
    }

    //Applying the new config takes up to 20ms
    vTaskDelay(pdMS_TO_TICKS(CONFIG_TIMEOUT_MS));

    //Wait for the initialisation status register to show a success or an error
    uint32_t configStartTick = HAL_GetTick();
    do {
        result = readRegisters(BMI2_INTERNAL_STATUS_ADDR, &value, 1);
        if(isError(result)) {
            state = BMI270_STATE_ERROR;
            return pushErrorCode(result, BMI270_STATE_INITIALISING, 6);  // NOLINT(*-magic-numbers)
        }
    } while((value == BMI2_NOT_INIT) && !timeout(configStartTick, CONFIG_TIMEOUT_MS));

    //check if an initialisation error occurred
    if(value != BMI2_INIT_OK) {
        state = BMI270_STATE_ERROR;
        return createErrorCodeLayer1(BMI270_STATE_INITIALISING, 7, value, ERR_CRITICAL);  // NOLINT(*-magic-numbers)
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
    static const register_t BMI2_ACC_RANGE_ADDR = 0x41U;
    static const register_t BMI2_GYR_RANGE_ADDR = 0x43U;

    // clang-format off
    static const BMI270register_t configuration[][2] = {
        {BMI2_PWR_CTRL_ADDR, (BMI2_GYR_EN_MASK | BMI2_ACC_EN_MASK | BMI2_TEMP_EN_MASK)},
        {BMI2_ACC_CONF_ADDR, (BMI2_ACC_FILTER_PERF_MODE_MASK | (BMI270register_t)(BMI2_ACC_NORMAL_AVG4 << 4U) | BMI2_ACC_ODR_100HZ)},
        {BMI2_ACC_RANGE_ADDR, BMI2_ACC_RANGE_2G},
        {BMI2_GYR_CONF_ADDR, (BMI2_GYR_FILTER_PERF_MODE_MASK | (BMI270register_t)(BMI2_GYR_NORMAL_MODE << 4U) | BMI2_GYR_ODR_100HZ)},
        {BMI2_GYR_RANGE_ADDR, BMI2_GYR_RANGE_500},
        {BMI2_PWR_CONF_ADDR, BMI2_FIFO_SELF_WAKE_UP_MASK},
    };
    // clang-format on

    //write all the configuration registers
    BMI270register_t reg         = 0;
    const uint8_t    nbRegisters = sizeof(configuration) / (sizeof(BMI270register_t) << 1U);
    do {
        result = writeRegisters(configuration[reg][0], &configuration[reg][1], 1);
        reg++;
    } while((reg < nbRegisters) && !isError(result));

    //if error, exit
    if(isError(result)) {
        state = BMI270_STATE_ERROR;
        return pushErrorCode(result, BMI270_STATE_CONFIGURING, 1);
    }

    state = BMI270_STATE_MEASURING;
    return ERR_SUCCESS;
}

/**
 * State in which measurements are received from the BMI270
 * 
 * @return Success
 */
static errorCode_u stateMeasuring(void) {
    float       accelerometer_mG[NB_AXIS];  ///< Accelerometer values in [mG]
    float       gyroscope_radps[NB_AXIS];   ///< Gyroscope values in [rad/s]
    rawValues_u LSBvalues = {0};
    vTaskDelay(pdMS_TO_TICKS(10U));

    //read all temp/accelerometer/gyroscope values
    result = readRegisters(BMI2_ACC_X_LSB_ADDR, LSBvalues.registers8bits, NB_REGISTERS_TO_READ);
    if(isError(result)) {
        state = BMI270_STATE_ERROR;
        return (pushErrorCode(result, BMI270_STATE_MEASURING, 2));
    }

    accelerometer_mG[0] = (float)(LSBvalues.values16bits[0]) * ACC_LSB_TO_MG;
    accelerometer_mG[1] = (float)(LSBvalues.values16bits[1]) * ACC_LSB_TO_MG;
    accelerometer_mG[2] = (float)(LSBvalues.values16bits[2]) * ACC_LSB_TO_MG;

    gyroscope_radps[0] = (float)(LSBvalues.values16bits[3]) * GYR_LSB_TO_RADPS;
    gyroscope_radps[1] = (float)(LSBvalues.values16bits[4]) * GYR_LSB_TO_RADPS;
    gyroscope_radps[2] = (float)(LSBvalues.values16bits[5]) * GYR_LSB_TO_RADPS;  // NOLINT

    //apply a complementary filter on read values
    complementaryFilter(accelerometer_mG, gyroscope_radps, latestAngles_rad);

    return ERR_SUCCESS;
}

/**
 * State in which BMI270 is in error
 */
static void stateError(void) {
}
