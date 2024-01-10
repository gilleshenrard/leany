/**
 * @brief Implement the ADXL345 accelerometer communication
 * @author Gilles Henrard
 * @date 10/01/2024
 *
 * @note Additional information can be found in :
 *   - ADXL345 datasheet : https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
 *   - AN-1025 (FIFO application note) document : https://www.analog.com/media/en/technical-documentation/application-notes/AN-1025.pdf
 *
 */
#include "ADXL345.h"
#include "ADXL345registers.h"
#include "main.h"
#include <math.h>

//definitions
#define SPI_TIMEOUT_MS		10U				///< SPI direct transmission timeout span in milliseconds
#define INT_TIMEOUT_MS		1000U			///< Maximum number of milliseconds before watermark int. timeout
#define ST_WAIT_MS			25U				///< Maximum number of milliseconds before watermark int. timeout
#define NB_REG_INIT			5U				///< Number of registers configured at initialisation
#define ADXL_AVG_SAMPLES	ADXL_SAMPLES_32	///< Amount of samples to integrate in the ADXL
#define ADXL_AVG_SHIFT		5U				///< Number used to shift the samples sum in order to divide it during integration

//assertions
static_assert((ADXL_AVG_SAMPLES >> ADXL_AVG_SHIFT) == 1, "TADXL_AVG_SHIFT does not divide all the samples configured with ADXL_AVG_NB");

//type definitions
/**
 * @brief Enumeration of the function IDs of the ADXL345
 */
typedef enum _ADXLfunctionCodes_e{
	INIT = 0,      		///< ADXL345initialise()
	SELF_TESTING_OFF,	///< stSelfTestingOFF()
	SELF_TEST_ENABLE,	///< stEnablingST()
	SELF_TEST_WAIT,		///< stWaitingForSTenabled()
	SELF_TESTING_ON,	///< stSelfTestingON()
	MEASURE,        	///< stMeasuring()
	CHK_MEASURES,  		///< ADXL345hasNewMeasurements()
	WRITE_REGISTER,		///< ADXL345writeRegister()
	READ_REGISTERS,		///< ADXL345readRegisters()
	GET_X_ANGLE,		///< ADXL345getXangleDegrees()
	GET_Y_ANGLE,		///< ADXL345getYangleDegrees()
	INTEGRATE,			///< integrateFIFO()
	STARTUP				///< stStartup()
}ADXLfunctionCodes_e;

/**
 * @brief SPI CS pin status enumeration
 */
typedef enum{
	DISABLED = 0,
	ENABLED,
}spiStatus_e;

/**
 * @brief Structure used to hold axis measurement values
 */
typedef struct{
	int16_t current;	///< Last known value
	int16_t previous;	///< Value held last time an update was checked on an axis
}adxlValues_t;

/**
 * @brief State machine state prototype
 *
 * @return Error code of the state
 */
typedef errorCode_u (*adxlState)();

//machine state
static errorCode_u stStartup();
static errorCode_u stConfiguring();
static errorCode_u stSelfTestingOFF();
static errorCode_u stEnablingST();
static errorCode_u stWaitingForSTenabled();
static errorCode_u stSelfTestingON();
static errorCode_u stMeasuring();
static errorCode_u stError();

//manipulation functions
static errorCode_u writeRegister(adxl345Registers_e registerNumber, uint8_t value);
static errorCode_u readRegisters(adxl345Registers_e firstRegister, uint8_t* value, uint8_t size);
static errorCode_u integrateFIFO(int16_t* xValue, int16_t* yValue, int16_t* zValue);

//tool functions
static inline void setSPIstatus(spiStatus_e value);
static inline float atanDegrees(int16_t direction, int16_t axisZ);

/**
 * @brief Array of all the registers/values to write at initialisation
 * @note Two values are written in FIFO_CONTROL to clear the FIFO at startup
 */
static const uint8_t initialisationArray[NB_REG_INIT][2] = {
	{BANDWIDTH_POWERMODE,	ADXL_POWER_NORMAL | ADXL_RATE_200HZ},
	{FIFO_CONTROL,			ADXL_MODE_BYPASS},
	{FIFO_CONTROL,			ADXL_MODE_FIFO | ADXL_TRIGGER_INT1 | (ADXL_AVG_SAMPLES - 1)},
	{INTERRUPT_ENABLE,		ADXL_INT_WATERMARK},
	{POWER_CONTROL,			ADXL_MEASURE_MODE},
};

// Default data format (register 0x31) value
static const uint8_t dataFormatDefault = (ADXL_NO_SELF_TEST | ADXL_SPI_4WIRE | ADXL_INT_ACTIV_LOW | ADXL_RANGE_16G);

//global variables
volatile uint8_t			adxlINT1occurred = 0;		///< Flag used to indicate the ADXL triggered an interrupt
volatile uint16_t			adxlTimer_ms = 0;			///< Timer used in various states of the ADXL (in ms)
volatile uint16_t			adxlSPITimer_ms = 0;		///< Timer used to make sure SPI does not time out (in ms)

//state variables
static SPI_TypeDef*		_spiHandle = NULL;			///< SPI handle used with the ADXL345
static GPIO_TypeDef*	_CSport = NULL;				///< GPIO port used to control the ADXL345 CS signal
static uint16_t			_CSpin = 0;					///< GPIO pin used to control the ADXL345 CS signal
static adxlState		_state = stStartup;			///< State machine current state
static uint8_t			_measurementsUpdated = 0;	///< Flag used to indicate new integrated measurements are ready within the ADXL345
static adxlValues_t		_finalValues[NB_AXIS];		///< Array of axis values
static errorCode_u 		_result;					///< Variables used to store error codes


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


/**
 * @brief Initialise the ADXL345
 *
 * @param handle		SPI handle used
 * @param CSGPIOport	GPIO port used to control CS
 * @param CSGPIOpin		GPIO pin used to control CS
 * @returns 			Success
 */
errorCode_u ADXL345initialise(const SPI_TypeDef* handle, const GPIO_TypeDef* CSGPIOport, uint16_t CSGPIOpin){
	_spiHandle = (SPI_TypeDef*)handle;
	LL_SPI_Enable(_spiHandle);

	_CSport = (GPIO_TypeDef*)CSGPIOport;
	_CSpin = CSGPIOpin;
	setSPIstatus(DISABLED);

	return (ERR_SUCCESS);
}

/**
 * @brief Run the ADXL state machine
 *
 * @return Current machine state return value
 */
errorCode_u ADXL345update(){
	return ( (*_state)() );
}

/**
 * @brief Check if new measurements have been updated
 *
 * @retval 0 No new values available
 * @retval 1 New values are available
 */
uint8_t ADXL345hasChanged(axis_e axis){
	uint8_t tmp = (_finalValues[axis].current != _finalValues[axis].previous);
	_finalValues[axis].previous = _finalValues[axis].current;

	return (tmp);
}

/**
 * @brief Write a single register on the ADXL345
 *
 * @param registerNumber Register number
 * @param value Register value
 * @retval 0 Success
 * @retval 1 No SPI handle set
 * @retval 2 Register number out of range
 * @retval 3 Attempted to access a reserved register
 * @retval 4 Error while writing the command
 * @retval 5 Error while writing the value
 */
errorCode_u writeRegister(adxl345Registers_e registerNumber, uint8_t value){
	errorCode_u result = ERR_SUCCESS;

	//if handle not set, error
	if(_spiHandle == NULL)
		return (createErrorCode(WRITE_REGISTER, 1, ERR_CRITICAL));

	//if SPI busy, error
	if(LL_SPI_IsActiveFlag_BSY(_spiHandle))
		return (createErrorCode(WRITE_REGISTER, 2, ERR_CRITICAL));

	//if register number above known, error
	if(registerNumber > ADXL_NB_REGISTERS)
		return (createErrorCode(WRITE_REGISTER, 3, ERR_WARNING));

	//if register number between 0x01 and 0x1C included, error
	if((uint8_t)(registerNumber - 1) < ADXL_HIGH_RESERVED_REG)
		return (createErrorCode(WRITE_REGISTER, 4, ERR_WARNING)); 	// @suppress("Avoid magic numbers")

	setSPIstatus(ENABLED);

	//format and transmit the two bytes command
	uint16_t command = ((ADXL_WRITE | ADXL_SINGLE | registerNumber) << 8) | value;
	LL_SPI_TransmitData16(_spiHandle, command);

	//wait for the SPI TX start to be confirmed
	adxlSPITimer_ms = SPI_TIMEOUT_MS;
	while(!LL_SPI_IsActiveFlag_TXE(_spiHandle) && adxlSPITimer_ms);
	if(!adxlSPITimer_ms){
		setSPIstatus(DISABLED);
		return (createErrorCode(WRITE_REGISTER, 5, ERR_WARNING));
	}

	//wait for the SPI to finish sending
	adxlSPITimer_ms = SPI_TIMEOUT_MS;
	while(!LL_SPI_IsActiveFlag_BSY(_spiHandle) && adxlSPITimer_ms);
	if(!adxlSPITimer_ms){
		setSPIstatus(DISABLED);
		return (createErrorCode(WRITE_REGISTER, 6, ERR_WARNING));
	}

	setSPIstatus(DISABLED);
	return (result);
}

/**
 * @brief Read several registers on the ADXL345
 *
 * @param firstRegister Number of the first register to read
 * @param[out] value Registers value array
 * @param size Number of registers to read
 * @retval 0 Success
 * @retval 1 No SPI handle set
 * @retval 2 Register number out of range
 * @retval 3 Error while writing the command
 * @retval 4 Error while reading the values
 */
errorCode_u readRegisters(adxl345Registers_e firstRegister, uint8_t* value, uint8_t size){
	errorCode_u result = ERR_SUCCESS;
	uint8_t* iterator = value;

	//if no bytes to read, success
	if(!size)
		return ERR_SUCCESS;

	//if handle not set or value buffer not provided, error
	if((_spiHandle == NULL) || (value == NULL))
		return (createErrorCode(READ_REGISTERS, 1, ERR_CRITICAL));

	//if SPI busy, error
	if(LL_SPI_IsActiveFlag_BSY(_spiHandle))
		return (createErrorCode(WRITE_REGISTER, 2, ERR_CRITICAL));

	//if register numbers above known, error
	if(firstRegister > ADXL_NB_REGISTERS)
		return (createErrorCode(READ_REGISTERS, 3, ERR_WARNING));

	setSPIstatus(ENABLED);

	//send the read request
	LL_SPI_TransmitData8(_spiHandle, ADXL_READ | ADXL_MULTIPLE | firstRegister);

	//wait for the SPI TX start to be confirmed
	adxlSPITimer_ms = SPI_TIMEOUT_MS;
	while(!LL_SPI_IsActiveFlag_TXE(_spiHandle) && adxlSPITimer_ms);
	if(!adxlSPITimer_ms){
		setSPIstatus(DISABLED);
		return (createErrorCode(READ_REGISTERS, 4, ERR_WARNING));
	}

	adxlSPITimer_ms = SPI_TIMEOUT_MS;

	//read bytes two by two first
	while(size >= 2 && adxlSPITimer_ms){
		//wait for RX flag to be up
		while(LL_SPI_IsActiveFlag_RXNE(_spiHandle) && adxlSPITimer_ms);

		//retrieve two bytes from SPI
		*((uint16_t*)iterator) = LL_SPI_ReceiveData16(_spiHandle);
		iterator += 2;
		size -= 2;
	}

	//read the remaining byte (if any)
	if(size && adxlSPITimer_ms){
		//wait for RX flag to be up
		while(!LL_SPI_IsActiveFlag_RXNE(_spiHandle) && adxlSPITimer_ms);

		//retrieve the remaining byte
		*iterator = LL_SPI_ReceiveData8(_spiHandle);
	}

	//if timeout, error
	if(!adxlSPITimer_ms){
		setSPIstatus(DISABLED);
		return (createErrorCode(READ_REGISTERS, 5, ERR_WARNING));
	}

	setSPIstatus(DISABLED);
	return (result);
}

/**
 * @brief Get the last known integrated measurements for an axis
 *
 * @param axis Axis of which get the measurement
 * @return Last known integrated measurement
 */
int16_t ADXL345getValue(axis_e axis){
	if(axis >= NB_AXIS)
		axis = X_AXIS;

	return (_finalValues[axis].current);
}

/**
 * @brief Transpose a measurement to an angle in degrees with the Z axis
 *
 * @param axisValue Measurement to transpose
 * @return Angle with the Z axis
 */
float measureToAngleDegrees(int16_t axisValue){
	return (atanDegrees(axisValue, _finalValues[Z_AXIS].current));
}

/**
 * @brief Compute the angle (in degrees) between any axis and the Z axis
 *
 * @param direction Value (in G) of an axis
 * @param axisZ Value (in G) of the Z axis
 * @return Angle between direction and the Z axis
 */
static inline float atanDegrees(int16_t direction, int16_t axisZ){
	static const float DEGREES_180 = 180.0f;	///< Value representing a flat angle

	if(!axisZ)
		return (0.0f);

	return ((atanf((float)direction / (float)axisZ) * DEGREES_180) / (float)M_PI);
}

/**
 * brief Set the SPI CS pin to enable/disable a SPI transmission
 *
 * @param value New CS pin status
 */
static inline void setSPIstatus(spiStatus_e value){
	if(value == ENABLED)
		LL_GPIO_ResetOutputPin(_CSport, _CSpin);
	else
		LL_GPIO_SetOutputPin(_CSport, _CSpin);
}

/**
 * @brief Retrieve and average the values held in the ADXL FIFOs
 *
 * @param[out] xValue Integrated X axis value
 * @param[out] yValue Integrated Y axis value
 * @param[out] zValue Integrated Z axis value
 * @retval 0 Success
 * @retval 1 Error while retrieving values from the FIFO
 */
errorCode_u integrateFIFO(int16_t* xValue, int16_t* yValue, int16_t* zValue){
	static const uint8_t BYTE_OFFSET = 8U;	///< Number of bits to offset a byte
	static const uint8_t X_INDEX_MSB = 1U;	///< Index of the X MSB in the measurements
	static const uint8_t X_INDEX_LSB = 0U;	///< Index of the X LSB in the measurements
	static const uint8_t Y_INDEX_MSB = 3U;	///< Index of the Y MSB in the measurements
	static const uint8_t Y_INDEX_LSB = 2U;	///< Index of the Y LSB in the measurements
	static const uint8_t Z_INDEX_MSB = 5U;	///< Index of the Z MSB in the measurements
	static const uint8_t Z_INDEX_LSB = 4U;	///< Index of the Z LSB in the measurements	
	static uint8_t buffer[ADXL_NB_DATA_REGISTERS];

	*xValue = *yValue = *zValue = 0;

	//for eatch of the 16 samples to read
	for(uint8_t i = 0 ; i < ADXL_AVG_SAMPLES ; i++){
		//read all data registers for 1 sample
		_result = readRegisters(DATA_X0, buffer, ADXL_NB_DATA_REGISTERS);
		if(IS_ERROR(_result)){
			_state = stError;
			return (pushErrorCode(_result, INTEGRATE, 1));
		}

		//add the measurements (formatted from a two's complement) to their final value buffer
		*xValue += (int16_t)(((uint16_t)(buffer[X_INDEX_MSB]) << BYTE_OFFSET) | (uint16_t)(buffer[X_INDEX_LSB]));
		*yValue += (int16_t)(((uint16_t)(buffer[Y_INDEX_MSB]) << BYTE_OFFSET) | (uint16_t)(buffer[Y_INDEX_LSB]));
		*zValue += (int16_t)(((uint16_t)(buffer[Z_INDEX_MSB]) << BYTE_OFFSET) | (uint16_t)(buffer[Z_INDEX_LSB]));
	}

	//divide the buffers to average out
	*xValue >>= ADXL_AVG_SHIFT;
	*yValue >>= ADXL_AVG_SHIFT;
	*zValue >>= ADXL_AVG_SHIFT;

	return (ERR_SUCCESS);
}


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


/**
 * @brief Begin state of the state machine
 *
 * @retval 0 Success
 * @retval 1 No SPI handle has been specified
 * @retval 2 Unable to read device ID
 * @retval 3 Device ID invalid
 */
errorCode_u stStartup(){
	uint8_t deviceID = 0;

	//if no handle specified, go error
	if(_spiHandle == NULL){
		_state = stError;
		return (createErrorCode(STARTUP, 1, ERR_CRITICAL));
	}

	//if unable to read device ID, go error
	adxlTimer_ms = INT_TIMEOUT_MS;
	_result = readRegisters(DEVICE_ID, &deviceID, 1);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, STARTUP, 2));
	}

	//if invalid device ID, go error
	if(deviceID != ADXL_DEVICE_ID)
		return (createErrorCode(STARTUP, 3, ERR_CRITICAL)); 	// @suppress("Avoid magic numbers")

	_state = stConfiguring;
	return (ERR_SUCCESS);
}

/**
 * @brief State in which the registers of the ADXL are configured
 *
 * @retval 0 Success
 * @retval 1 Error while writing a register
 */
errorCode_u stConfiguring(){
	//write the default data format
	_result = writeRegister(DATA_FORMAT, dataFormatDefault);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, INIT, 1));
	}

	//write all registers values from the initialisation array
	for(uint8_t i = 0 ; i < NB_REG_INIT ; i++){
		_result = writeRegister(initialisationArray[i][0], initialisationArray[i][1]);
		if(IS_ERROR(_result)){
			_state = stError;
			return (pushErrorCode(_result, INIT, 2));
		}
	}

	//reset the timer and get to next state
	adxlTimer_ms = INT_TIMEOUT_MS;
	_state = stSelfTestingOFF;
	return (_result);
}

/**
 * @brief State in which the ADXL does some measurements with self-test OFF
 * @note p. 22, 31 and 32 of the datasheet
 *
 * @retval 0 Success
 * @retval 1 Timeout while waiting for measurements
 * @retval 2 Error while integrating the FIFOs
 */
errorCode_u stSelfTestingOFF(){
	//if timeout, go error
	if(!adxlTimer_ms){
		_state = stError;
		return (createErrorCode(SELF_TESTING_OFF, 1, ERR_ERROR));
	}

	//if watermark interrupt not fired, exit
	if(!adxlINT1occurred)
		return (ERR_SUCCESS);

	//retrieve the integrated measurements
	_result = integrateFIFO(&_finalValues[X_AXIS].current, &_finalValues[Y_AXIS].current, &_finalValues[Z_AXIS].current);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TESTING_OFF, 2));
	}

	//get to next state
	_state = stEnablingST;
	return (ERR_SUCCESS);
}

/**
 * @brief State in which the self-test mode is enabled
 *
 * @return 0 Success
 * @return 1 Error while enabling self-test
 * @return 2 Error while clearing the FIFO
 */
errorCode_u stEnablingST(){
	//Enable the self-test
	_result = writeRegister(DATA_FORMAT, dataFormatDefault | ADXL_SELF_TEST);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TEST_ENABLE, 1)); 	// @suppress("Avoid magic numbers")
	}

	//clear the FIFOs
	_result = writeRegister(FIFO_CONTROL, ADXL_MODE_BYPASS);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TEST_ENABLE, 2)); 	// @suppress("Avoid magic numbers")
	}

	//reset timer and get to next state
	adxlTimer_ms = ST_WAIT_MS;
	_state = stWaitingForSTenabled;
	return (ERR_SUCCESS);
}

/**
 * @brief State in which the ADXL waits for a while before restarting measurements
 *
 * @return 0 Success
 * @return 1 Error while re-enabling FIFOs
 */
errorCode_u stWaitingForSTenabled(){
	if(!adxlTimer_ms)
		return (ERR_SUCCESS);

	//enable FIFOs
	adxlINT1occurred = 0;
	_result = writeRegister(FIFO_CONTROL, ADXL_MODE_FIFO | ADXL_TRIGGER_INT1 | (ADXL_AVG_SAMPLES - 1));
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TEST_WAIT, 1)); 	// @suppress("Avoid magic numbers")
	}

	//reset timer and get to next state
	adxlTimer_ms = INT_TIMEOUT_MS;
	_state = stSelfTestingON;
	return (ERR_SUCCESS);
}

/**
 * @brief State in which the ADXL measures while in self-test mode
 *
 * @retval 0 Success
 * @retval 1 Timeout while waiting for measurements
 * @retval 2 Error while integrating the FIFOs
 * @retval 3 Error while resetting the data format
 * @retval 4 Self-test values out of range
 */
errorCode_u stSelfTestingON(){
	int16_t _finalXSTon = 0;
	int16_t _finalYSTon = 0;
	int16_t _finalZSTon = 0;

	//if timeout, go error
	if(!adxlTimer_ms){
		_state = stError;
		return (createErrorCode(SELF_TESTING_ON, 1, ERR_ERROR));
	}

	//if watermark interrupt not fired, exit
	if(!adxlINT1occurred)
		return (ERR_SUCCESS);

	//integrate the FIFOs
	adxlINT1occurred = 0;
	_result = integrateFIFO(&_finalXSTon, &_finalYSTon, &_finalZSTon);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TESTING_ON, 2));
	}

	//restore the default data format
	_result = writeRegister(DATA_FORMAT, dataFormatDefault | ADXL_FULL_RESOL);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TESTING_ON, 3)); 	// @suppress("Avoid magic numbers")
	}

	//compute the self-test deltas
	_finalXSTon -= _finalValues[X_AXIS].current;
	_finalYSTon -= _finalValues[Y_AXIS].current;
	_finalZSTon -= _finalValues[Z_AXIS].current;

	//if self-test values out of range, error
	if((_finalXSTon <= ADXL_ST_MINX_33_16G) || (_finalXSTon >= ADXL_ST_MAXX_33_16G)
		|| (_finalYSTon <= ADXL_ST_MINY_33_16G) || (_finalYSTon >= ADXL_ST_MAXY_33_16G)
		|| (_finalZSTon <= ADXL_ST_MINZ_33_16G) || (_finalZSTon >= ADXL_ST_MAXZ_33_16G))
	{
		_state = stError;
		return (pushErrorCode(_result, SELF_TESTING_ON, 4)); 	// @suppress("Avoid magic numbers")
	}

	//reset timer and get to next state
	adxlTimer_ms = INT_TIMEOUT_MS;
	_state = stMeasuring;
	return (ERR_SUCCESS);
}

/**
 * @brief State in which the ADXL measures accelerations
 *
 * @retval 0 Success
 * @retval 1 Timeout occurred while waiting for watermark interrupt
 * @retval 2 Error occurred while integrating the FIFOs
 */
errorCode_u stMeasuring(){
	//if timeout, go error
	if(!adxlTimer_ms){
		_state = stError;
		return (createErrorCode(MEASURE, 1, ERR_ERROR));
	}

	//if watermark interrupt not fired, exit
	if(!adxlINT1occurred)
		return (ERR_SUCCESS);

	//reset flags
	adxlTimer_ms = INT_TIMEOUT_MS;
	adxlINT1occurred = 0;

	//integrate the FIFOs
	_result = integrateFIFO(&_finalValues[X_AXIS].current, &_finalValues[Y_AXIS].current, &_finalValues[Z_AXIS].current);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, MEASURE, 2));
	}

	_measurementsUpdated = 1;
	return (ERR_SUCCESS);
}

/**
 * @brief State in which the ADXL stays in an error state forever
 *
 * @return Success
 */
errorCode_u stError(){
	return (ERR_SUCCESS);
}
