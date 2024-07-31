/**
 * @file LSM6DSO.c
 * @brief Implement the LSM6DSO MEMS sensor communication
 * @author Gilles Henrard
 * @date 31/07/2024
 *
 * @note Additional information can be found in :
 *   - Datasheet : https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 *   - AN5192 (always-on 3-axis accelerometer and 3-axis gyroscope) : https://www.st.com/resource/en/application_note/an5192-lsm6dso-alwayson-3axis-accelerometer-and-3axis-gyroscope-stmicroelectronics.pdf
 *   - AN5226 (Finite State Machine) : https://www.st.com/resource/en/application_note/an5226-lsm6dso-finite-state-machine-stmicroelectronics.pdf
 *   - DT0058 (Design tip) : https://www.st.com/resource/en/design_tip/dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.pdf
 */
#include "LSM6DSO.h"
#include <math.h>
#include <stdint.h>
#include "LSM6DSO_registers.h"
#include "errorstack.h"
#include "main.h"
#include "stm32f103xb.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"

static const float ANGLE_DELTA_MINIMUM = 0.05F;  ///< Minimum value for angle differences to be noticed
enum {
    BOOT_TIME_MS         = 10U,    ///< Number of milliseconds to wait for the MEMS to boot
    SPI_TIMEOUT_MS       = 10U,    ///< Number of milliseconds beyond which SPI is in timeout
    TIMEOUT_MS           = 1000U,  ///< Max number of milliseconds to wait for the device ID
    REGISTER_VALUE_ALIGN = 8,      ///< Alignment of the registerValue_t struct
};

/**
 * @brief Enumeration of all the function ID used in errors
 */
typedef enum {
    READ_REGISTERS = 1,  ///< readRegisters() function
    WRITE_REGISTER,      ///< writeRegister() function
    CHECK_DEVICE_ID,     ///< stateWaitingDeviceID() state
    CONFIGURING,         ///< stateConfiguring() state
    DROPPING,            ///< stateIgnoringSamples() state
    MEASURING,           ///< stMeasuring() state
} LSM6DSOfunction_e;

/**
 * @brief Structure representing a value to write at a specific register
 */
typedef struct {
    LSM6DSOregister_e registerID;  ///< Register ID to which write the value
    uint8_t           value;       ///< Value to write
} __attribute__((aligned(REGISTER_VALUE_ALIGN))) registerValue_t;

/**
 * @brief Union regrouping 8-bits and 16-bits arrays
 * @details
 *  This allows reading all the accelerometer and gyroscope values at once, and convert them to 16 bit instantly
 */
typedef union {
    uint8_t registers8bits[LSM6_NB_OUT_REGISTERS];        ///< 8-bits registers array
    int16_t values16bits[(LSM6_NB_OUT_REGISTERS >> 1U)];  ///< 16-bits values array
} rawValues_u;

/**
 * @brief State machine state prototype
 *
 * @return Error code of the state
 */
typedef errorCode_u (*lsm6dsoState)();

//machine state
static errorCode_u stateWaitingBoot();
static errorCode_u stateWaitingDeviceID();
static errorCode_u stateConfiguring();
static errorCode_u stateIgnoringSamples();
static errorCode_u stateMeasuring();
static errorCode_u stateError();

//registers read/write functions
static errorCode_u writeRegister(LSM6DSOregister_e registerNumber, uint8_t value);
static errorCode_u readRegisters(LSM6DSOregister_e firstRegister, uint8_t value[], uint8_t size);

static inline uint8_t dataReady(void);
static void           complementaryFilter(const float accelerometer_mG[], const float gyroscope_radps[],
                                          float filteredAngles_deg[]);

//global variables
volatile uint16_t lsm6dsoTimer_ms    = BOOT_TIME_MS;  ///< Timer used in various states of the LSM6DSO (in ms)
volatile uint16_t lsm6dsoSPITimer_ms = 0;             ///< Timer used to make sure SPI does not time out (in ms)

//state variables
static SPI_TypeDef* spiHandle                   = (void*)0;          ///< SPI handle used by the LSM6DSO device
static lsm6dsoState state                       = stateWaitingBoot;  ///< State machine current state
static uint8_t     accelerometerSamplesToIgnore = 0;  ///< Number of samples to ignore after change of ODR or power mode
static errorCode_u result;                            ///< Variables used to store error codes
static float       zeroValues[NB_AXIS];               ///< Accelerometer values at time of zeroing in [m/s²]
static float       latestAngles_deg[NB_AXIS - 1] = {0.0F, 0.0F};

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Initialise the LSM6DSO
 *
 * @param handle	SPI handle used
 * @returns 		Success
 */
errorCode_u LSM6DSOinitialise(const SPI_TypeDef* handle) {
    spiHandle = (SPI_TypeDef*)handle;
    LL_SPI_Disable(spiHandle);

    return (ERR_SUCCESS);
}

/**
 * @brief Run the LSM6DSO state machine
 * @returns Current state return code
 */
errorCode_u LSM6DSOupdate() {
    return ((*state)());
}

/**
 * @brief Read several registers on the LSM6DSO
 *
 * @param firstRegister Number of the first register to read
 * @param[out] value Registers value array
 * @param size Number of registers to read
 * @return   Success
 * @retval 1 SPI handle or value buffer NULL
 * @retval 2 Timeout
 */
static errorCode_u readRegisters(LSM6DSOregister_e firstRegister, uint8_t value[], uint8_t size) {
    static const uint8_t SPI_RX_FILLER = 0xFFU;  ///< Value to send as a filler while receiving multiple bytes

    //if no bytes to read, success
    if(!size) {
        return ERR_SUCCESS;
    }

    //make sure neither the handle nor the buffer are NULL
    if(!spiHandle || !value) {
        return (createErrorCode(READ_REGISTERS, 1, ERR_CRITICAL));
    }

    //set timeout timer and enable SPI
    lsm6dsoSPITimer_ms = SPI_TIMEOUT_MS;
    LL_SPI_Enable(spiHandle);
    uint8_t* iterator = value;

    //send the read request and ignore the first byte received (reply to the write request)
    LL_SPI_TransmitData8(spiHandle, LSM6_READ | (uint8_t)firstRegister);
    while((!LL_SPI_IsActiveFlag_RXNE(spiHandle)) && lsm6dsoSPITimer_ms) {};
    *iterator = LL_SPI_ReceiveData8(spiHandle);

    //receive the bytes to read
    do {
        //send a filler byte to keep the SPI clock running, to receive the next byte
        LL_SPI_TransmitData8(spiHandle, SPI_RX_FILLER);

        //wait for data to be available, and read it
        while((!LL_SPI_IsActiveFlag_RXNE(spiHandle)) && lsm6dsoSPITimer_ms) {};
        *iterator = LL_SPI_ReceiveData8(spiHandle);

        iterator++;
        size--;
    } while(size && lsm6dsoSPITimer_ms);

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(spiHandle) && lsm6dsoSPITimer_ms) {};
    LL_SPI_ClearFlag_OVR(spiHandle);

    //disable SPI
    LL_SPI_Disable(spiHandle);

    //if timeout, error
    if(!lsm6dsoSPITimer_ms) {
        return (createErrorCode(READ_REGISTERS, 2, ERR_WARNING));
    }

    return (ERR_SUCCESS);
}

/**
 * @brief Write a single register on the LSM6DSO
 *
 * @param registerNumber Register number
 * @param value Register value
 * @return	 Success
 * @retval 1 No SPI handle specified
 * @retval 2 Register number out of range
 * @retval 3 Timeout
 */
static errorCode_u writeRegister(LSM6DSOregister_e registerNumber, uint8_t value) {
    //if handle not specified, error
    if(!spiHandle) {
        return (createErrorCode(WRITE_REGISTER, 1, ERR_WARNING));
    }

    //if register number above known or within the reserved range, error
    if(registerNumber > MAX_REGISTER) {
        return (createErrorCode(WRITE_REGISTER, 2, ERR_WARNING));
    }

    //set timeout timer and enable SPI
    lsm6dsoSPITimer_ms = SPI_TIMEOUT_MS;
    LL_SPI_Enable(spiHandle);

    //send the write instruction
    LL_SPI_TransmitData8(spiHandle, LSM6_WRITE | (uint8_t)registerNumber);

    //wait for TX buffer to be ready and send value to write
    while(!LL_SPI_IsActiveFlag_TXE(spiHandle) && lsm6dsoSPITimer_ms) {};
    if(lsm6dsoSPITimer_ms) {
        LL_SPI_TransmitData8(spiHandle, value);
    }

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(spiHandle) && lsm6dsoSPITimer_ms) {};
    LL_SPI_ClearFlag_OVR(spiHandle);

    //disable SPI
    LL_SPI_Disable(spiHandle);

    //if timeout, error
    if(!lsm6dsoSPITimer_ms) {
        return (createErrorCode(WRITE_REGISTER, 3, ERR_WARNING));
    }

    return (ERR_SUCCESS);
}

/**
 * @brief Check if angle measurements have changed
 *
 * @retval 0 No new values available
 * @retval 1 New values are available
 */
uint8_t LSM6DSOhasChanged(axis_e axis) {
    static float previousAccelerometerValues[NB_AXIS - 1] = {0.0F};
    uint8_t      tmp                                      = 0;

    if(latestAngles_deg[axis] > previousAccelerometerValues[axis]) {
        tmp = ((latestAngles_deg[axis] - previousAccelerometerValues[axis]) > ANGLE_DELTA_MINIMUM);
    } else {
        tmp = ((previousAccelerometerValues[axis] - latestAngles_deg[axis]) > ANGLE_DELTA_MINIMUM);
    }

    previousAccelerometerValues[axis] = latestAngles_deg[axis];

    return (tmp);
}

/**
 * @brief Transpose a measurement to an angle in tenths of degrees with the Z axis
 *
 * @param axis Axis for which get the angle with the Z axis
 * @return Angle with the Z axis
 */
int16_t getAngleDegreesTenths(axis_e axis) {
    const float ANGLE_TO_TENTHS = 10.0F;
    return ((int16_t)(latestAngles_deg[axis] * ANGLE_TO_TENTHS));
}

/**
 * @brief Set the measurements in relative mode and zero down the values
 */
void LSM6DSOzeroDown(void) {
    zeroValues[X_AXIS] = -latestAngles_deg[X_AXIS];
    zeroValues[Y_AXIS] = -latestAngles_deg[Y_AXIS];
}

/**
 * @brief Set the measurements in absolute mode (no zeroing compensation)
 */
void LSM6DSOcancelZeroing(void) {
    for(uint8_t axis = 0; axis < (uint8_t)NB_AXIS; axis++) {
        zeroValues[axis] = 0;
    }
}

/**
 * @brief Compute a complementary filter on accelerometer/gyroscope values
 * 
 * @param[in] accelerometer_mG    Array of acceleration values in [mG] on all axis
 * @param[in] gyroscope_radps       Array of gyroscope values  in [rad/s] on X and Y axis
 * @param[out] filteredAngles_deg     Array of final angle values in [°] on X and Y axis
 */
void complementaryFilter(const float accelerometer_mG[], const float gyroscope_radps[], float filteredAngles_deg[]) {
    const float alpha                 = 0.02F;  ///< Proportion applied to the gyro. and accel. in the final result
    const float dtPeriod_sec          = 0.00240385F;  ///< Time period between two updates (LSM6DSO config. at 416Hz)
    const float RADIANS_TO_DEGREES    = 57.2957795F;  ///< Ratio between radians and degrees (= 180°/PI)
    const float GRAVITATION_MG        = 1000.0F;      ///< Grativation value in mG
    float       AccelEstimatedX_deg   = 0.0F;         ///< Estimated accelerator angle on the X axis in [°]
    float       AccelEstimatedY_deg   = 0.0F;         ///< Estimated accelerator angle on the Y axis in [°]
    float       eulerAngleRateX_radps = 0.0F;  ///< Euler angle rate (with reference to Earth) around X axis in rad/s
    float       eulerAngleRateY_radps = 0.0F;  ///< Euler angle rate (with reference to Earth) around Y axis in rad/s

    //calculate the accelerometer angle estimations in °
    AccelEstimatedX_deg = asinf(accelerometer_mG[X_AXIS] / GRAVITATION_MG) * RADIANS_TO_DEGREES;
    AccelEstimatedY_deg = atanf(accelerometer_mG[Y_AXIS] / accelerometer_mG[Z_AXIS]) * RADIANS_TO_DEGREES;

    //Transform gyroscope rates (reference is the solid body) to Euler rates (reference is Earth)
    eulerAngleRateX_radps =
        gyroscope_radps[X_AXIS]
        + (sinf(filteredAngles_deg[X_AXIS]) * tanf(filteredAngles_deg[Y_AXIS]) * gyroscope_radps[Y_AXIS])
        + (cosf(filteredAngles_deg[X_AXIS]) * tanf(filteredAngles_deg[Y_AXIS]) * gyroscope_radps[Z_AXIS]);

    eulerAngleRateY_radps = (cosf(filteredAngles_deg[X_AXIS]) * gyroscope_radps[Y_AXIS])
                            - (sinf(filteredAngles_deg[X_AXIS]) * gyroscope_radps[Z_AXIS]);

    //combine accelerometer estimates with Euler angle rates estimates
    filteredAngles_deg[X_AXIS] =
        ((1.0F - alpha) * (filteredAngles_deg[X_AXIS] + (eulerAngleRateX_radps * dtPeriod_sec * RADIANS_TO_DEGREES)))
        + (alpha * AccelEstimatedX_deg);
    filteredAngles_deg[Y_AXIS] =
        ((1.0F - alpha) * (filteredAngles_deg[Y_AXIS] + (eulerAngleRateY_radps * dtPeriod_sec * RADIANS_TO_DEGREES)))
        + (alpha * AccelEstimatedY_deg);
}

/**
 * @brief Check if an INT1 event occurred
 * 
 * @retval 0 INT1 did not occur
 * @retval 1 INT1 occurred
 */
static inline uint8_t dataReady(void) {
    return (uint8_t)LL_GPIO_IsInputPinSet(LSM6DSO_INT1_GPIO_Port, LSM6DSO_INT1_Pin);
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief State in which the program waits for the LSM6DSO to boot up
 * 
 * @return Success
 */
static errorCode_u stateWaitingBoot() {
    //if timer elapsed, reset it and get to next state
    if(!lsm6dsoTimer_ms) {
        lsm6dsoTimer_ms = TIMEOUT_MS;
        state           = stateWaitingDeviceID;
    }

    return (ERR_SUCCESS);
}

/**
 * @brief State during which the Who Am I ID is checked
 * @attention This takes the 10ms boot time into account
 * @note If no correct manufacturer ID is read within 1 second, a timeout occurs
 * 
 * @retval 0 Success
 * @retval 1 Timeout while reading the manufacturer ID
 * @retval 2 Error while sending the read request
 */
static errorCode_u stateWaitingDeviceID() {
    uint8_t deviceID = 0;

    //if 1s elapsed without reading the correct vendor ID, go error
    if(!lsm6dsoTimer_ms) {
        state = stateError;
        return (createErrorCode(CHECK_DEVICE_ID, 1, ERR_CRITICAL));
    }

    //if unable to read device ID, error
    result = readRegisters(WHO_AM_I, &deviceID, 1);
    if(isError(result)) {
        return (pushErrorCode(result, CHECK_DEVICE_ID, 2));
    }

    //if invalid device ID, exit
    if(deviceID != LSM6_WHOAMI) {
        return (ERR_SUCCESS);
    }

    //reset timeout timer and get to next state
    state = stateConfiguring;
    return (ERR_SUCCESS);
}

/**
 * @brief State in which the registers are configured in the LSM6DSO
 * 
 * @retval 0 Success
 * @retval 1 Error while writing a register
 */
static errorCode_u stateConfiguring() {
#define NB_INIT_REG 9U
    const uint8_t         AXL_SAMPLES_TO_IGNORE = 2U;  ///< Number of samples to drop (see stateIgnoringSamples())
    const registerValue_t initialisationArray[NB_INIT_REG] = {
        {   CTRL3_C, LSM6_SOFTWARE_RESET | LSM6_INT_ACTIVE_LOW}, //reboot MEMS memory and reset software
        {FIFO_CTRL4,                          FIFO_MODE_BYPASS}, //disable the FIFO (bypass mode)
        { INT1_CTRL,                         INT1_AXL_DATA_RDY}, //enable the accelerometer DATA READY interrupt on INT1
        {  CTRL8_XL,         AXL_NO_HP_FILTER | AXL_LPF2_ODR_4}, //disable accererometer HP filter and set LP2 cutoff to ODR/4
        {  CTRL1_XL,     LSM6_ODR_416HZ | LSM6_AXL_LPF2_ENABLE}, //set accelerometer in high-perf. mode + enable LPF 2
        {   CTRL7_G,     GYR_HPF_ENABLE | GYR_HPF_CUTOFF_65MHZ}, //enable the gyroscope HP filter with 16mHz cutoff freq.
        {   CTRL4_C,                           GYR_LPF1_ENABLE}, //enable the gyroscope LP1 filter
        {   CTRL6_C,                   GYR_LPF1_CUTOFF_120_3HZ}, //set the gyroscope LPF1 cutoff frequency to 136.6Hz
        {   CTRL2_G,                            LSM6_ODR_416HZ}, //set the gyroscope in high-performance mode
    };

    //write all registers values from the initialisation array
    for(uint8_t i = 0; i < NB_INIT_REG; i++) {
        result = writeRegister(initialisationArray[i].registerID, initialisationArray[i].value);
        if(isError(result)) {
            state = stateError;
            return (pushErrorCode(result, CONFIGURING, 1));
        }
    }

    //set the number of samples to ignore after changing ODR and power mode
    accelerometerSamplesToIgnore = AXL_SAMPLES_TO_IGNORE;

    state = stateIgnoringSamples;
    return (ERR_SUCCESS);
}

/**
 * @brief State in which the first few samples measured are dropped
 * @note This is recommended in the document AN5192, Table 12
 * 
 * @retval 0 Success
 * @retval 1 Error while reading a register
 */
errorCode_u stateIgnoringSamples() {
    //if no interrupt occurred, exit
    if(!dataReady()) {
        return (ERR_SUCCESS);
    }

    //read an accelerometer value to reset the latched interrupt
    uint8_t dummyValue = 0;
    result             = readRegisters(OUTX_H_A, &dummyValue, 1);
    if(isError(result)) {
        state = stateError;
        return (pushErrorCode(result, DROPPING, 1));
    }

    //decrement the remaining amount to ignore and exit if still remaining
    accelerometerSamplesToIgnore--;
    if(accelerometerSamplesToIgnore) {
        return (ERR_SUCCESS);
    }

    //get to measuring state
    state = stateMeasuring;
    return (ERR_SUCCESS);
}

/**
 * @brief State in which the measurements are pulled from the sensor
 * 
 * @retval 0 Success
 * @retval 1 Error while reading the status register value
 */
static errorCode_u stateMeasuring() {
    rawValues_u LSBvalues = {0};            ///< Buffer in which read values will be stored
    float       accelerometer_mG[NB_AXIS];  ///< Accelerometer values in [mG]
    float       gyroscope_radps[NB_AXIS];   ///< Gyroscope values in [rad/s]
    int16_t*    valueIterator = (void*)0;

    //if no interrupt occurred, exit
    if(!dataReady()) {
        return (ERR_SUCCESS);
    }

    //read all accelerometer/gyroscope values (they are synchronised, as stated in AN5192, section 3)
    result = readRegisters(OUTX_L_G, LSBvalues.registers8bits, LSM6_NB_OUT_REGISTERS);
    if(isError(result)) {
        state = stateError;
        return (pushErrorCode(result, MEASURING, 2));
    }

    //prepare iterator values
    uint8_t axis  = 0;
    valueIterator = LSBvalues.values16bits;

    //convert the gyroscope LSB values to rad/s
    //datasheet p.9 : gyroscope sensitivity at 125°/s = 4.375[mdps/LSB]
    //      to rad/s : (sensitivity / 1000[mdps/dps]) * (PI/180°) = 0.000076358155
    const float GYR_SENSITIVITY_125DPS_RPS = 0.000076358155F;
    for(axis = 0; axis < (uint8_t)NB_AXIS; axis++) {
        gyroscope_radps[axis] = (float)(*valueIterator) * GYR_SENSITIVITY_125DPS_RPS;
        valueIterator++;
    }

    //then convert the accelerometer LSB values to mG
    //datasheet p.9 : accelerometer sensitivity at 2G = 0.061 [mG/LSB]
    const float AXL_SENSITIVITY_2G = 0.061F;
    for(axis = 0; axis < (uint8_t)NB_AXIS; axis++) {
        accelerometer_mG[axis] = (float)(*valueIterator) * AXL_SENSITIVITY_2G;
        valueIterator++;
    }

    //apply a complementary filter on read values
    complementaryFilter(accelerometer_mG, gyroscope_radps, latestAngles_deg);

    return (ERR_SUCCESS);
}

static errorCode_u stateError() {
    return (ERR_SUCCESS);
}
