/**
 * @brief Implement the ADXL345 accelerometer communication
 * @author Gilles Henrard
 * @date 15/02/2024
 *
 * @note Additional information can be found in :
 *   - ADXL345 datasheet : https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
 *   - AN-1077 (Quick Start Guide) : https://www.analog.com/media/en/technical-documentation/application-notes/AN-1077.pdf
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
#define NB_REG_INIT			6U				///< Number of registers configured at initialisation
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
	SELF_TESTING_OFF,	///< stMeasuringST_OFF()
	SELF_TEST_WAIT,		///< stWaitingForSTenabled()
	SELF_TESTING_ON,	///< stMeasuringST_ON()
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
static errorCode_u stMeasuringST_OFF();
static errorCode_u stWaitingForSTenabled();
static errorCode_u stMeasuringST_ON();
static errorCode_u stMeasuring();
static errorCode_u stError();

//manipulation functions
static errorCode_u writeRegister(adxl345Registers_e registerNumber, uint8_t value);
static errorCode_u readRegisters(adxl345Registers_e firstRegister, uint8_t* value, uint8_t size);
static errorCode_u integrateFIFO(int16_t* xValue, int16_t* yValue, int16_t* zValue);

//tool functions
static inline float arctanDegrees(int16_t direction, int16_t axisZ);
static inline int16_t twoComplement(uint8_t MSB, uint8_t LSB);

// Default DATA FORMAT (register 0x31) and FIFO CONTROL (register 0x38) register values
static const uint8_t DATA_FORMAT_DEFAULT = (ADXL_NO_SELF_TEST | ADXL_SPI_4WIRE | ADXL_INT_ACTIV_LOW | ADXL_RIGHT_JUSTIFY | ADXL_RANGE_16G);
static const uint8_t FIFO_CONTROL_DEFAULT = (ADXL_MODE_FIFO | ADXL_TRIGGER_INT1 | (ADXL_AVG_SAMPLES - 1));

//global variables
volatile uint8_t			adxlINT1occurred = 0;			///< Flag used to indicate the ADXL triggered an interrupt
volatile uint16_t			adxlTimer_ms = INT_TIMEOUT_MS;	///< Timer used in various states of the ADXL (in ms)
volatile uint16_t			adxlSPITimer_ms = 0;			///< Timer used to make sure SPI does not time out (in ms)

//state variables
static SPI_TypeDef*		_spiHandle = NULL;			///< SPI handle used with the ADXL345
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
 * @returns 			Success
 */
errorCode_u ADXL345initialise(const SPI_TypeDef* handle){
	_spiHandle = (SPI_TypeDef*)handle;
	LL_SPI_Disable(_spiHandle);

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
 * @return	 Success
 * @retval 1 Register number out of range
 * @retval 2 Timeout
 */
errorCode_u writeRegister(adxl345Registers_e registerNumber, uint8_t value){
	//assertions
	assert(_spiHandle);

	//if register number above known or within the reserved range, error
	if((registerNumber > ADXL_NB_REGISTERS) || ((uint8_t)(registerNumber - 1) < ADXL_HIGH_RESERVED_REG))
		return (createErrorCode(WRITE_REGISTER, 1, ERR_WARNING));

	//set timeout timer and enable SPI
	adxlSPITimer_ms = SPI_TIMEOUT_MS;
	LL_SPI_Enable(_spiHandle);

	//send the write instruction
	LL_SPI_TransmitData8(_spiHandle, ADXL_WRITE | ADXL_SINGLE | registerNumber);

	//wait for TX buffer to be ready and send value to write
	while(!LL_SPI_IsActiveFlag_TXE(_spiHandle) && adxlSPITimer_ms);
	if(adxlSPITimer_ms)
		LL_SPI_TransmitData8(_spiHandle, value);

	//wait for transaction to be finished and clear Overrun flag
	while(LL_SPI_IsActiveFlag_BSY(_spiHandle) && adxlSPITimer_ms);
	LL_SPI_ClearFlag_OVR(_spiHandle);

	//disable SPI
	LL_SPI_Disable(_spiHandle);

	//if timeout, error
	if(!adxlSPITimer_ms)
		return (createErrorCode(WRITE_REGISTER, 2, ERR_WARNING));

	return (ERR_SUCCESS);
}

/**
 * @brief Read several registers on the ADXL345
 *
 * @param firstRegister Number of the first register to read
 * @param[out] value Registers value array
 * @param size Number of registers to read
 * @return   Success
 * @retval 1 Register number out of range
 * @retval 2 Timeout
 */
errorCode_u readRegisters(adxl345Registers_e firstRegister, uint8_t* value, uint8_t size){
	static const uint8_t SPI_RX_FILLER = 0xFFU;	///< Value to send as a filler while receiving multiple bytes

	//if no bytes to read, success
	if(!size)
		return ERR_SUCCESS;

	//assertions
	assert(_spiHandle);
	assert(value);

	//if register numbers above known, error
	if(firstRegister > ADXL_NB_REGISTERS)
		return (createErrorCode(READ_REGISTERS, 1, ERR_WARNING));

	//set timeout timer and enable SPI
	adxlSPITimer_ms = SPI_TIMEOUT_MS;
	LL_SPI_Enable(_spiHandle);

	//send the read request and wait for it to be sent
	LL_SPI_TransmitData8(_spiHandle, ADXL_READ | ADXL_MULTIPLE | firstRegister);
	while((!LL_SPI_IsActiveFlag_TXE(_spiHandle)) && adxlSPITimer_ms);

	//receive the bytes to read
	uint8_t* iterator = value;
	do{
		//wait for data to be available, and read it
		while((!LL_SPI_IsActiveFlag_RXNE(_spiHandle)) && adxlSPITimer_ms);
		*iterator = LL_SPI_ReceiveData8(_spiHandle);

		//send a filler byte to keep the SPI clock running, to receive the next byte
		LL_SPI_TransmitData8(_spiHandle, SPI_RX_FILLER);
		
		iterator++;
		size--;
	}while(size && adxlSPITimer_ms);

	//wait for transaction to be finished and clear Overrun flag
	while(LL_SPI_IsActiveFlag_BSY(_spiHandle) && adxlSPITimer_ms);
	LL_SPI_ClearFlag_OVR(_spiHandle);

	//disable SPI
	LL_SPI_Disable(_spiHandle);

	//if timeout, error
	if(!adxlSPITimer_ms)
		return (createErrorCode(READ_REGISTERS, 2, ERR_WARNING));

	return (ERR_SUCCESS);
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
	return (arctanDegrees(axisValue, _finalValues[Z_AXIS].current));
}

/**
 * @brief Compute the angle (in degrees) between any axis and the Z axis
 *
 * @param direction Value (in G) of an axis
 * @param axisZ Value (in G) of the Z axis
 * @return Angle between direction and the Z axis
 */
static inline float arctanDegrees(int16_t direction, int16_t axisZ){
	static const float DEGREES_180 = 180.0f;	///< Value representing a flat angle

	if(!axisZ)
		return (0.0f);

	return ((atanf((float)direction / (float)axisZ) * DEGREES_180) * (float)M_1_PI);
}

/**
 * @brief Translate two bytes into an int16_t via a two's complement
 * 
 * @param MSB		Most Significant Byte
 * @param LSB		Least Significant Byte
 * @return int16_t	16 bit resulting number
 */
static inline int16_t twoComplement(uint8_t MSB, uint8_t LSB){
	return (int16_t)(((uint16_t)MSB << 8) | (uint16_t)LSB);
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
	static const uint8_t X_INDEX_MSB = 1U;	///< Index of the X MSB in the measurements
	static const uint8_t X_INDEX_LSB = 0U;	///< Index of the X LSB in the measurements
	static const uint8_t Y_INDEX_MSB = 3U;	///< Index of the Y MSB in the measurements
	static const uint8_t Y_INDEX_LSB = 2U;	///< Index of the Y LSB in the measurements
	static const uint8_t Z_INDEX_MSB = 5U;	///< Index of the Z MSB in the measurements
	static const uint8_t Z_INDEX_LSB = 4U;	///< Index of the Z LSB in the measurements	
	static uint8_t buffer[ADXL_NB_DATA_REGISTERS];

	*xValue = *yValue = *zValue = 0;

	//for each of the samples to read
	for(uint8_t i = 0 ; i < ADXL_AVG_SAMPLES ; i++){
		//read all data registers for 1 sample
		_result = readRegisters(DATA_X0, buffer, ADXL_NB_DATA_REGISTERS);
		if(IS_ERROR(_result)){
			_state = stError;
			return (pushErrorCode(_result, INTEGRATE, 1));
		}

		//add the measurements (formatted from a two's complement) to their final value buffer
		*xValue += twoComplement(buffer[X_INDEX_MSB], buffer[X_INDEX_LSB]);
		*yValue += twoComplement(buffer[Y_INDEX_MSB], buffer[Y_INDEX_LSB]);
		*zValue += twoComplement(buffer[Z_INDEX_MSB], buffer[Z_INDEX_LSB]);

		//wait for a while to make sure 5 us pass between two reads
		//	as stated in the datasheet, section "Retrieving data from the FIFO"
		volatile uint8_t tempo = 0x1FU;
		while(tempo--);
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
 * @retval 1 Device ID invalid
 * @retval 2 Unable to read device ID
 */
errorCode_u stStartup(){
	uint8_t deviceID = 0;

	//if 1s elapsed without reading the correct vendor ID, go error
	if(!adxlTimer_ms){
		_state = stError;
		return (createErrorCode(STARTUP, 1, ERR_CRITICAL));
	}

	//if unable to read device ID, error
	_result = readRegisters(DEVICE_ID, &deviceID, 1);
	if(IS_ERROR(_result))
		return (pushErrorCode(_result, STARTUP, 2));

	//if invalid device ID, exit
	if(deviceID != ADXL_DEVICE_ID)
		return (ERR_SUCCESS);

	//reset timeout timer and get to next state
	adxlTimer_ms = INT_TIMEOUT_MS;
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
	static const uint8_t initialisationArray[NB_REG_INIT][2] = {
		{DATA_FORMAT,			DATA_FORMAT_DEFAULT | ADXL_10BIT_RESOL},
		{BANDWIDTH_POWERMODE,	ADXL_POWER_NORMAL | ADXL_RATE_200HZ},
		{FIFO_CONTROL,			ADXL_MODE_BYPASS},		//clear the FIFOs first (blocks otherwise)
		{FIFO_CONTROL,			FIFO_CONTROL_DEFAULT},
		{POWER_CONTROL,			ADXL_MEASURE_MODE},		///
		{INTERRUPT_ENABLE,		ADXL_INT_WATERMARK},	///must come at the end
	};

	//write all registers values from the initialisation array
	for(uint8_t i = 0 ; i < NB_REG_INIT ; i++){
		_result = writeRegister(initialisationArray[i][0], initialisationArray[i][1]);
		if(IS_ERROR(_result)){
			_state = stError;
			return (pushErrorCode(_result, INIT, 1));
		}
	}

	//reset the timer and get to next state
	adxlTimer_ms = INT_TIMEOUT_MS;
	_state = stMeasuringST_OFF;
	return (_result);
}

/**
 * @brief State in which the ADXL does some measurements with self-test OFF
 * @note p. 22, 31 and 32 of the datasheet
 *
 * @retval 0 Success
 * @retval 1 Timeout while waiting for measurements
 * @retval 2 Error while integrating the FIFOs
 * @retval 3 Error while enabling the self-testing mode
 * @retval 4 Error while clearing the FIFOs
 */
errorCode_u stMeasuringST_OFF(){
	//if timeout, go error
	if(!adxlTimer_ms){
		_state = stError;
		return (createErrorCode(SELF_TESTING_OFF, 1, ERR_ERROR));
	}

	//if watermark interrupt not fired, exit
	if(!adxlINT1occurred)
		return (ERR_SUCCESS);

	//retrieve the integrated measurements (to be used with self-testing)
	_result = integrateFIFO(&_finalValues[X_AXIS].current, &_finalValues[Y_AXIS].current, &_finalValues[Z_AXIS].current);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TESTING_OFF, 2));
	}

	//Enable the self-test
	_result = writeRegister(DATA_FORMAT, DATA_FORMAT_DEFAULT | ADXL_SELF_TEST);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TESTING_OFF, 3));
	}

	//clear the FIFOs
	_result = writeRegister(FIFO_CONTROL, ADXL_MODE_BYPASS);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TESTING_OFF, 4));
	}

	//set timer to wait for 25ms and get to next state
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
	//if timer not elapsed yet, exit
	if(!adxlTimer_ms)
		return (ERR_SUCCESS);

	//enable FIFOs
	adxlINT1occurred = 0;
	_result = writeRegister(FIFO_CONTROL, FIFO_CONTROL_DEFAULT);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TEST_WAIT, 1)); 	// @suppress("Avoid magic numbers")
	}

	//reset timer and get to next state
	adxlTimer_ms = INT_TIMEOUT_MS;
	_state = stMeasuringST_ON;
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
errorCode_u stMeasuringST_ON(){
	int16_t STdeltaX = 0;
	int16_t STdeltaY = 0;
	int16_t STdeltaZ = 0;

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
	_result = integrateFIFO(&STdeltaX, &STdeltaY, &STdeltaZ);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TESTING_ON, 2));
	}

	//reset the data format
	_result = writeRegister(DATA_FORMAT, DATA_FORMAT_DEFAULT | ADXL_FULL_RESOL);
	if(IS_ERROR(_result)){
		_state = stError;
		return (pushErrorCode(_result, SELF_TESTING_ON, 3)); 	// @suppress("Avoid magic numbers")
	}

	//compute the self-test deltas
	STdeltaX -= _finalValues[X_AXIS].current;
	STdeltaY -= _finalValues[Y_AXIS].current;
	STdeltaZ -= _finalValues[Z_AXIS].current;

	//if self-test values out of range, error
	if((STdeltaX <= ADXL_ST_MINX_33_16G) || (STdeltaX >= ADXL_ST_MAXX_33_16G)
		|| (STdeltaY <= ADXL_ST_MINY_33_16G) || (STdeltaY >= ADXL_ST_MAXY_33_16G)
		|| (STdeltaZ <= ADXL_ST_MINZ_33_16G) || (STdeltaZ >= ADXL_ST_MAXZ_33_16G))
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
