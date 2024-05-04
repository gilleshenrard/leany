/**
 * @file LSM6DSO.c
 * @brief Implement the LSM6DSO MEMS sensor communication
 * @author Gilles Henrard
 * @date 04/05/2024
 *
 * @note Additional information can be found in :
 *   - Datasheet : https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 *   - AN5192 (always-on 3-axis accelerometer and 3-axis gyroscope) : https://www.st.com/resource/en/application_note/an5192-lsm6dso-alwayson-3axis-accelerometer-and-3axis-gyroscope-stmicroelectronics.pdf
 *   - AN5226 (Finite State Machine) : https://www.st.com/resource/en/application_note/an5226-lsm6dso-finite-state-machine-stmicroelectronics.pdf
 *   - DT0058 (Design tip) : https://www.st.com/resource/en/design_tip/dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.pdf
 */
#include "LSM6DSO.h"
#include "LSM6DSO_registers.h"
#include "LSM6DSO_config_script.h"

static const uint8_t BOOT_TIME_MS = 10U;
static const uint8_t SPI_TIMEOUT_MS = 10U;
static const uint16_t TIMEOUT_MS = 1000U;

/**
 * @brief Enumeration of all the function ID used in errors
 */
typedef enum{
    READ_REGISTERS = 1,     ///< readRegisters() function
    WRITE_REGISTER,         ///< writeRegister() function
    CHECK_DEVICE_ID,        ///< stateWaitingDeviceID() state
    CONFIGURING,            ///< stateConfiguring() state
    MEASURING,              ///< stMeasuring() state
}LSM6DSOfunction_e;

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
static errorCode_u stateMeasuring();
static errorCode_u stateError();

//registers read/write functions
static errorCode_u writeRegister(LSM6DSOregister_e registerNumber, uint8_t value);
static errorCode_u readRegisters(LSM6DSOregister_e firstRegister, uint8_t value[], uint8_t size);
static inline int16_t twoComplement(const uint8_t bytes[2]);

//global variables
volatile uint16_t	lsm6dsoTimer_ms = BOOT_TIME_MS; ///< Timer used in various states of the LSM6DSO (in ms)
volatile uint16_t	lsm6dsoSPITimer_ms = 0;         ///< Timer used to make sure SPI does not time out (in ms)

//state variables
static SPI_TypeDef* _spiHandle = (void*)0;          ///< SPI handle used by the LSM6DSO device
static lsm6dsoState _state = stateWaitingBoot;      ///< State machine current state
static errorCode_u 	_result;                        ///< Variables used to store error codes
int16_t             accelerometerValues[NB_AXIS];
int16_t             gyroscopeValues[NB_AXIS];


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


/**
 * @brief Initialise the LSM6DSO
 *
 * @param handle	SPI handle used
 * @returns 		Success
 */
errorCode_u LSM6DSOinitialise(const SPI_TypeDef* handle){
    _spiHandle = (SPI_TypeDef*)handle;
    LL_SPI_Disable(_spiHandle);

    return (ERR_SUCCESS);
}

/**
 * @brief Run the LSM6DSO state machine
 * @returns Current state return code
 */
errorCode_u LSM6DSOupdate(){
    return ( (*_state)() );
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
static errorCode_u readRegisters(LSM6DSOregister_e firstRegister, uint8_t value[], uint8_t size){
    static const uint8_t SPI_RX_FILLER = 0xFFU;	///< Value to send as a filler while receiving multiple bytes

    //if no bytes to read, success
    if(!size)
        return ERR_SUCCESS;

    //make sure neither the handle nor the buffer are NULL
    if(!_spiHandle || !value)
        return (createErrorCode(READ_REGISTERS, 1, ERR_CRITICAL));

    //set timeout timer and enable SPI
    lsm6dsoSPITimer_ms = SPI_TIMEOUT_MS;
    LL_SPI_Enable(_spiHandle);
    uint8_t* iterator = value;

    //send the read request and ignore the first byte received (reply to the write request)
    LL_SPI_TransmitData8(_spiHandle, LSM6_READ | firstRegister);
    while((!LL_SPI_IsActiveFlag_RXNE(_spiHandle)) && lsm6dsoSPITimer_ms);
    *iterator = LL_SPI_ReceiveData8(_spiHandle);

    //receive the bytes to read
    do{
        //send a filler byte to keep the SPI clock running, to receive the next byte
        LL_SPI_TransmitData8(_spiHandle, SPI_RX_FILLER);

        //wait for data to be available, and read it
        while((!LL_SPI_IsActiveFlag_RXNE(_spiHandle)) && lsm6dsoSPITimer_ms);
        *iterator = LL_SPI_ReceiveData8(_spiHandle);
        
        iterator++;
        size--;
    }while(size && lsm6dsoSPITimer_ms);

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(_spiHandle) && lsm6dsoSPITimer_ms);
    LL_SPI_ClearFlag_OVR(_spiHandle);

    //disable SPI
    LL_SPI_Disable(_spiHandle);

    //if timeout, error
    if(!lsm6dsoSPITimer_ms)
        return (createErrorCode(READ_REGISTERS, 2, ERR_WARNING));

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
static errorCode_u writeRegister(LSM6DSOregister_e registerNumber, uint8_t value){
    //if handle not specified, error
    if(!_spiHandle)
        return (createErrorCode(WRITE_REGISTER, 1, ERR_WARNING));

    //if register number above known or within the reserved range, error
    if(registerNumber > MAX_REGISTER)
        return (createErrorCode(WRITE_REGISTER, 2, ERR_WARNING));

    //set timeout timer and enable SPI
    lsm6dsoSPITimer_ms = SPI_TIMEOUT_MS;
    LL_SPI_Enable(_spiHandle);

    //send the write instruction
    LL_SPI_TransmitData8(_spiHandle, LSM6_WRITE | registerNumber);

    //wait for TX buffer to be ready and send value to write
    while(!LL_SPI_IsActiveFlag_TXE(_spiHandle) && lsm6dsoSPITimer_ms);
    if(lsm6dsoSPITimer_ms)
        LL_SPI_TransmitData8(_spiHandle, value);

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(_spiHandle) && lsm6dsoSPITimer_ms);
    LL_SPI_ClearFlag_OVR(_spiHandle);

    //disable SPI
    LL_SPI_Disable(_spiHandle);

    //if timeout, error
    if(!lsm6dsoSPITimer_ms)
        return (createErrorCode(WRITE_REGISTER, 3, ERR_WARNING));

    return (ERR_SUCCESS);
}

/**
 * @brief Reassemble a two's complement int16_t from two bytes
 * 
 * @param bytes		Array of two bytes to reassemble
 * @return int16_t	16 bit resulting number
 */
static inline int16_t twoComplement(const uint8_t bytes[2]){
    return (((int16_t)bytes[1] << 8) | (int16_t)bytes[0]);
}


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


/**
 * @brief State in which the program waits for the LSM6DSO to boot up
 * 
 * @return Success
 */
static errorCode_u stateWaitingBoot(){
    //if timer elapsed, reset it and get to next state
    if(!lsm6dsoTimer_ms){
        lsm6dsoTimer_ms = TIMEOUT_MS;
        _state = stateWaitingDeviceID;
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
static errorCode_u stateWaitingDeviceID(){
    uint8_t deviceID = 0;

    //if 1s elapsed without reading the correct vendor ID, go error
    if(!lsm6dsoTimer_ms){
        _state = stateError;
        return (createErrorCode(CHECK_DEVICE_ID, 1, ERR_CRITICAL));
    }

    //if unable to read device ID, error
    _result = readRegisters(WHO_AM_I, &deviceID, 1);
    if(isError(_result))
        return (pushErrorCode(_result, CHECK_DEVICE_ID, 2));

    //if invalid device ID, exit
    if(deviceID != LSM6_WHOAMI)
        return (ERR_SUCCESS);

    //reset timeout timer and get to next state
    _state = stateConfiguring;
    return (ERR_SUCCESS);
}

/**
 * @brief State in which the registers are configured in the LSM6DSO
 * 
 * @retval 0 Success
 * @retval 1 Error while writing a register
 */
static errorCode_u stateConfiguring(){
    const registerValue_t* iterator = initialisationArray;

    //write all registers values from the initialisation array
    for(uint8_t i = 0 ; i < NB_INIT_REG ; i++){
        _result = writeRegister(iterator->registerID, iterator->value);
        if(isError(_result)){
            _state = stateError;
            return (pushErrorCode(_result, CONFIGURING, 1));
        }

        iterator++;
    }

    lsm6dsoTimer_ms = SPI_TIMEOUT_MS;
    _state = stateMeasuring;
    return (ERR_SUCCESS);
}

/**
 * @brief State in which the measurements are pulled from the sensor
 * 
 * @retval 0 Success
 * @retval 1 Error while reading the status register value
 */
static errorCode_u stateMeasuring(){
    //if timer not elapsed, exit
    if(lsm6dsoTimer_ms)
        return (ERR_SUCCESS);

    //reset the timer
    lsm6dsoTimer_ms = SPI_TIMEOUT_MS;

    //retrieve the status register value
    uint8_t status;
    _result = readRegisters(STATUS_REG, &status, 1);
    if(isError(_result)){
        _state = stateError;
        return (pushErrorCode(_result, MEASURING, 1));
    }

    //declare the measurements buffer
    uint8_t buffer[LSM6_NB_OUT_REGISTERS];

    //if accelerometer data available, retrieve and format it
    if(status & LSM6_AXL_DATA_AVAIL){
        _result = readRegisters(OUTX_L_A, buffer, LSM6_NB_OUT_REGISTERS);
        if(isError(_result)){
            _state = stateError;
            return (pushErrorCode(_result, MEASURING, 2));
        }

        accelerometerValues[X_AXIS] = twoComplement(&buffer[X_AXIS << 1]);
        accelerometerValues[Y_AXIS] = twoComplement(&buffer[Y_AXIS << 1]);
        accelerometerValues[Z_AXIS] = twoComplement(&buffer[Z_AXIS << 1]);
    }

    //if gyroscope data available, retrieve and format it
    if(status & LSM6_GYR_DATA_AVAIL){
        _result = readRegisters(OUTX_L_G, buffer, LSM6_NB_OUT_REGISTERS);
        if(isError(_result)){
            _state = stateError;
            return (pushErrorCode(_result, MEASURING, 3));
        }
        gyroscopeValues[X_AXIS] = twoComplement(&buffer[X_AXIS << 1]);
        gyroscopeValues[Y_AXIS] = twoComplement(&buffer[Y_AXIS << 1]);
        gyroscopeValues[Z_AXIS] = twoComplement(&buffer[Z_AXIS << 1]);
    }
    
    return (ERR_SUCCESS);
}

static errorCode_u stateError(){
    return (ERR_SUCCESS);
}
