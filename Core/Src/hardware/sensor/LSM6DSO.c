/**
 * @file LSM6DSO.c
 * @brief Implement the LSM6DSO MEMS sensor communication
 * @author Gilles Henrard
 * @date 14/03/2024
 *
 * @note Additional information can be found in :
 *   - Datasheet : https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 *   - AN5192 (always-on 3-axis accelerometer and 3-axis gyroscope) : https://www.st.com/resource/en/application_note/an5192-lsm6dso-alwayson-3axis-accelerometer-and-3axis-gyroscope-stmicroelectronics.pdf
 *   - AN5226 (Finite State Machine) : https://www.st.com/resource/en/application_note/an5226-lsm6dso-finite-state-machine-stmicroelectronics.pdf
 *   - DT0058 (Design tip) : https://www.st.com/resource/en/design_tip/dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.pdf
 */
#include "LSM6DSO.h"
#include "LSM6DSO_registers.h"

static const uint8_t SPI_TIMEOUT_MS = 10U;
static const uint16_t TIMEOUT_MS = 1000U;

/**
 * @brief Enumeration of all the function ID used in errors
 */
typedef enum{
    READ_REGISTERS = 1,     ///< readRegisters() function
    CHECK_DEVICE_ID         ///< stWaitingDeviceID() state
}LSM6DSOfunction_e;

/**
 * @brief State machine state prototype
 *
 * @return Error code of the state
 */
typedef errorCode_u (*lsm6dsoState)();

//machine state
static errorCode_u stWaitingDeviceID();
static errorCode_u stIdle();
static errorCode_u stError();

//registers read/write functions
static errorCode_u readRegisters(LSM6DSOregister_e firstRegister, uint8_t value[], uint8_t size);

//global variables
volatile uint16_t	lsm6dsoTimer_ms = TIMEOUT_MS;   ///< Timer used in various states of the LSM6DSO (in ms)
volatile uint16_t	lsm6dsoSPITimer_ms = 0;         ///< Timer used to make sure SPI does not time out (in ms)

//state variables
static SPI_TypeDef* _spiHandle = (void*)0;          ///< SPI handle used by the LSM6DSO device
static lsm6dsoState _state = stWaitingDeviceID;     ///< State machine current state
static errorCode_u 	_result;                        ///< Variables used to store error codes


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


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


/**
 * @brief State during which the Who Am I ID is checked
 * @attention This takes the 10ms boot time into account
 * @note If no correct manufacturer ID is read within 1 second, a timeout occurs
 * 
 * @retval 0 Success
 * @retval 1 Timeout while reading the manufacturer ID
 * @retval 2 Error while sending the read request
 */
static errorCode_u stWaitingDeviceID(){
    uint8_t deviceID = 0;

    //if 1s elapsed without reading the correct vendor ID, go error
    if(!lsm6dsoTimer_ms){
        _state = stError;
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
    _state = stIdle;
    return (ERR_SUCCESS);
}

static errorCode_u stIdle(){
    return (ERR_SUCCESS);
}

static errorCode_u stError(){
    return (ERR_SUCCESS);
}
