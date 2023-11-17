/**
 * @file SSD1306.c
 * @brief Implement the functioning of the SSD1306 OLED screen via SPI and DMA
 * @author Gilles Henrard
 * @date 17/11/2023
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 */
#include "SSD1306.h"
#include "numbersVerdana16.h"
#include "SSD1306_registers.h"
#include "main.h"

//definitions
#define SPI_TIMEOUT_MS		10U		///< Maximum number of milliseconds SPI traffic should last before timeout
#define MAX_PARAMETERS		6U		///< Maximum number of parameters a command can have
#define MAX_DATA_SIZE		1024U	///< Maximum data size (128 * 64 bits / 8 bits per bytes)
#define MIN_ANGLE_DEG		-90.0f	///< Minimum angle allowed (in degrees)
#define MAX_ANGLE_DEG		90.0f	///< Maximum angle allowed (in degrees)
#define FLOAT_FACTOR_10		10.0f	///< Factor of 10 used in float calculations
#define INT_FACTOR_10		10U		///< Factor of 10 used in integer calculations
#define NEG_THRESHOLD		-0.05f	///< Threshold above which an angle is considered positive (circumvents float incaccuracies)
#define INDEX_SIGN			0		///< Index of the sign in the angle indexes array
#define INDEX_TENS			1U		///< Index of the tens in the angle indexes array
#define INDEX_UNITS			2U		///< Index of the units in the angle indexes array
#define INDEX_TENTHS		4U		///< Index of the tenths in the angle indexes array
#define ANGLE_NB_CHARS		6U		///< Number of characters in the angle array
#define NB_INIT_REGISERS	8U		///< Number of registers set at initialisation
#define SSD_LAST_COLUMN		127U	///< Index of the highest column
#define SSD_LAST_PAGE		31U		///< Index of the highest page

/**
 * @brief Enumeration of the function IDs of the SSD1306
 */
typedef enum _SSD1306functionCodes_e{
	INIT = 0,		///< SSD1306initialise()
	SEND_CMD,		///< SSD1306sendCommand()
	PRT_ANGLE,		///< SSD1306_printAngle()
	SENDING_DATA,	///< stSendingData()
	WAITING_DMA_RDY	///< stWaitingForTXdone()
}_SSD1306functionCodes_e;

/**
 * @brief SPI CS pin status enumeration
 */
typedef enum{
	DISABLED = 0,
	ENABLED,
}spiStatus_e;

/**
 * @brief SPI Data/command pin status enumeration
 */
typedef enum{
	COMMAND = 0,
	DATA,
}dataStatus_e;

/**
 * @brief Screen state machine state prototype
 *
 * @return Return code of the state
 */
typedef errorCode_u (*screenState)();

/**
 * @brief Structure used to initialise the registers
 */
typedef struct{
	SSD1306register_e	reg;			///< Register to initialise
	uint8_t				nbParameters;	///< Number of parameters provided to initialise
	uint8_t				value;			///< Value of the parameters
}SSD1306init_t;

//communication functions with the SSD1306
static inline void setSPIstatus(spiStatus_e value);
static inline void setDataStatus(dataStatus_e value);
static errorCode_u sendCommand(SSD1306register_e regNumber, const uint8_t parameters[], uint8_t nbParameters);

//state machine
static errorCode_u stIdle();
static errorCode_u stSendingData();
static errorCode_u stWaitingForTXdone();

static const SSD1306init_t initCommands[NB_INIT_REGISERS] = {			///< Array used to initialise the registers
		{SCAN_DIRECTION_N1_0,	0,	0x00},
		{HARDWARE_CONFIG,		1,	SSD_PIN_CONFIG_ALT | SSD_COM_REMAP_DISABLE},
		{SEGMENT_REMAP_127,		0,	0x00},
		{MEMORY_ADDR_MODE,		1,	SSD_HORIZONTAL_ADDR},
		{CONTRAST_CONTROL,		1,	SSD_CONTRAST_HIGHEST},
		{CLOCK_DIVIDE_RATIO,	1,	SSD_CLOCK_FREQ_MID | SSD_CLOCK_DIVIDER_1},
		{CHG_PUMP_REGULATOR,	1,	SSD_ENABLE_CHG_PUMP},
		{DISPLAY_ON,			0,	0x00},
};

//state variables
volatile uint16_t			screenTimer_ms = 0;				///< Timer used with screen SPI transmissions
static SPI_HandleTypeDef*	_SSD_SPIhandle = NULL;			///< SPI handle used with the SSD1306
static screenState			_state = stIdle;				///< State machine current state
static uint8_t				_screenBuffer[MAX_DATA_SIZE];	///< Buffer used to send data to the screen
static uint8_t 				_limitColumns[2];				///< Buffer used to set the first and last column to send
static uint8_t				_limitPages[2];					///< Buffer used to set the first and last page to send
static uint16_t				_size;							///< Number of bytes to send


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


/**
 * @brief Initialise the SSD1306
 *
 * @param handle SPI handle used
 * @retval 0 Success
 * @retval 1 Error while initialising the registers
 * @retval 2 Error while clearing the screen
 */
errorCode_u SSD1306initialise(SPI_HandleTypeDef* handle){
	errorCode_u result;
	_SSD_SPIhandle = handle;

	//make sure to disable SSD1306 SPI communication
	setSPIstatus(DISABLED);

	//reset the chip
	HAL_GPIO_WritePin(SSD1306_RST_GPIO_Port, SSD1306_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SSD1306_RST_GPIO_Port, SSD1306_RST_Pin, GPIO_PIN_SET);

	//initialisation taken from PDF p. 64 (Application Example)
	//	values which don't change from reset values aren't modified
	//TODO test for max oscillator frequency
	for(uint8_t i = 0 ; i < NB_INIT_REGISERS ; i++){
		result = sendCommand(initCommands[i].reg, &initCommands[i].value, initCommands[i].nbParameters);
		if(IS_ERROR(result))
			return (pushErrorCode(result, INIT, 1));
	}

	result = SSD1306clearScreen();
	if(IS_ERROR(result))
		return (pushErrorCode(result, INIT, 2));		// @suppress("Avoid magic numbers")

	return (ERR_SUCCESS);
}

/**
 * brief Set the SPI CS pin to enable/disable a SPI transmission
 *
 * @param value New CS pin status
 */
static inline void setSPIstatus(spiStatus_e value){
	HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, (value == ENABLED ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

/**
 * brief Set the Data/Command pin
 *
 * @param value Value of the data/command pin
 */
static inline void setDataStatus(dataStatus_e value){
	HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, (value == COMMAND ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

/**
 * @brief Send a command with parameters
 *
 * @param regNumber Register number
 * @param parameters Parameters to write
 * @param nbParameters Number of parameters to write
 * @retval 0 Success
 * @retval 1 Number of parameters above maximum
 * @retval 2 Error while sending the command
 * @retval 3 Error while sending the data
 */
errorCode_u sendCommand(SSD1306register_e regNumber, const uint8_t parameters[], uint8_t nbParameters){
	HAL_StatusTypeDef HALresult;
	errorCode_u result = ERR_SUCCESS;

	//if too many parameters, error
	if(nbParameters > MAX_PARAMETERS)
		return(createErrorCode(SEND_CMD, 1, ERR_WARNING));

	//set command pin and enable SPI
	setDataStatus(COMMAND);
	setSPIstatus(ENABLED);

	//send the command byte
	HALresult = HAL_SPI_Transmit(_SSD_SPIhandle, &regNumber, 1, SPI_TIMEOUT_MS);
	if(HALresult != HAL_OK){
		setSPIstatus(DISABLED);
		return (createErrorCodeLayer1(SEND_CMD, 2, HALresult, ERR_ERROR));
	}

	//if command send OK, send all parameters
	if(parameters && nbParameters){
		HALresult = HAL_SPI_Transmit(_SSD_SPIhandle, (uint8_t*)parameters, nbParameters, SPI_TIMEOUT_MS);
		if(HALresult != HAL_OK)
			result = createErrorCodeLayer1(SEND_CMD, 3, HALresult, ERR_ERROR); 		// @suppress("Avoid magic numbers")
	}

	//disable SPI and return status
	setSPIstatus(DISABLED);
	return (result);
}

/**
 * @brief Send the whole screen buffer to wipe it
 *
 * @return Success
 */
errorCode_u SSD1306clearScreen(){
	uint8_t* iterator = _screenBuffer;

	_limitColumns[0] = 0;
	_limitColumns[1] = SSD_LAST_COLUMN;
	_limitPages[0] = 0;
	_limitPages[1] = SSD_LAST_PAGE;
	_size = MAX_DATA_SIZE;

	for(uint16_t i = 0 ; i < MAX_DATA_SIZE ; i++)
		*(iterator++) = 0x00U;

	_state = stSendingData;
	return (ERR_SUCCESS);
}

/**
 * @brief Check if the screen is ready to accept new commands
 *
 * @return 1 if ready
 */
uint8_t isScreenReady(){
	return (_state == stIdle);
}

/**
 * @brief Print an angle (in degrees, with sign) on the screen
 *
 * @param angle	Angle to print
 * @param page	First page on which to print the angle (screen line)
 * @param column First column on which to print the angle
 *
 * @retval 0 Success
 * @retval 1 Angle above maximum amplitude
 */
errorCode_u SSD1306_printAngle(float angle, uint8_t page, uint8_t column){
	uint8_t charIndexes[ANGLE_NB_CHARS] = {INDEX_PLUS, 0, 0, INDEX_DOT, 0, INDEX_DEG};
	uint8_t* iterator = _screenBuffer;

	//if angle out of bounds, return error
	if((angle < MIN_ANGLE_DEG) || (angle > MAX_ANGLE_DEG))
		return (createErrorCode(PRT_ANGLE, 1, ERR_WARNING));

	//store the values
	_limitColumns[0] = column;
	_limitColumns[1] = column + (VERDANA_CHAR_WIDTH * ANGLE_NB_CHARS) - 1;
	_limitPages[0] = page;
	_limitPages[1] = page + 1;
	_size = ANGLE_NB_CHARS * VERDANA_NB_BYTES_CHAR;

	//if angle negative, replace plus sign with minus sign
	if(angle < NEG_THRESHOLD){
		charIndexes[INDEX_SIGN] = INDEX_MINUS;
		angle = -angle;
	}

	//fill the angle characters indexes array with the float values (tens, units, tenths)
	charIndexes[INDEX_TENS] = (uint8_t)(angle / FLOAT_FACTOR_10);
	charIndexes[INDEX_UNITS] = ((uint8_t)angle) % INT_FACTOR_10;
	charIndexes[INDEX_TENTHS] = (uint8_t)((uint16_t)(angle * FLOAT_FACTOR_10) % INT_FACTOR_10);

	//fill the buffer with all the required bitmaps bytes (column by column, then character by character, then page by page)
	for(page = 0 ; page < VERDANA_NB_PAGES ; page++){
		for(uint8_t character = 0 ; character < ANGLE_NB_CHARS ; character++){
			for(column = 0 ; column < VERDANA_CHAR_WIDTH ; column++){
				*iterator = verdana_16ptNumbers[charIndexes[character]][(VERDANA_CHAR_WIDTH * page) + column];
				iterator++;
			}
		}
	}

	//get to printing state
	_state = stSendingData;
	return (ERR_SUCCESS);
}

/**
 * @brief Run the state machine
 *
 * @return Return code of the current state
 */
errorCode_u SSD1306update(){
	return ((*_state)());
}


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


/**
 * @brief State in which the screen awaits for commands
 *
 * @return Success
 */
errorCode_u stIdle(){
	return (ERR_SUCCESS);
}

/**
 * @brief State in which data is sent to the screen
 *
 * @retval 0 Success
 * @retval 1 Error occurred while sending the column address command
 * @retval 1 Error occurred while sending the page address command
 * @retval 1 Error occurred while sending the data
 */
errorCode_u stSendingData(){
	errorCode_u result;
	HAL_StatusTypeDef HALresult;

	//send the set start and end column addresses
	result = sendCommand(COLUMN_ADDRESS, _limitColumns, 2);
	if(IS_ERROR(result)){
		_state = stIdle;
		return (pushErrorCode(result, SENDING_DATA, 1));
	}

	//send the set start and end page addresses
	/*result = */sendCommand(PAGE_ADDRESS, _limitPages, 2);
	if(IS_ERROR(result)){
		_state = stIdle;
		return (pushErrorCode(result, SENDING_DATA, 2));
	}

	//set GPIOs
	setDataStatus(DATA);
	setSPIstatus(ENABLED);

	//send the data
	screenTimer_ms = SPI_TIMEOUT_MS;
	HALresult = HAL_SPI_Transmit_DMA(_SSD_SPIhandle, _screenBuffer, _size);
	if(HALresult != HAL_OK){
		_state = stIdle;
		return (createErrorCodeLayer1(SENDING_DATA, 3, HALresult, ERR_ERROR)); 	// @suppress("Avoid magic numbers")
	}

	//get to next
	_state = stWaitingForTXdone;
	return (ERR_SUCCESS);
}

/**
 * @brief State in which the machine waits for a DMA transmission to end
 *
 * @retval 0 Success
 * @retval 1 Timeout while waiting for transmission to end
 */
errorCode_u stWaitingForTXdone(){
	//if timer elapsed, stop DMA and error
	if(!screenTimer_ms){
		setSPIstatus(DISABLED);
		HAL_SPI_DMAStop(_SSD_SPIhandle);
		_state = stIdle;
		return (createErrorCode(WAITING_DMA_RDY, 1, ERR_ERROR));
	}

	//if TX not done yet, exit
	if(HAL_SPI_GetState(_SSD_SPIhandle) != HAL_SPI_STATE_READY)
		return (ERR_SUCCESS);

	//disable SPI and get to idle state
	setSPIstatus(DISABLED);
	_state = stIdle;
	return (ERR_SUCCESS);
}
