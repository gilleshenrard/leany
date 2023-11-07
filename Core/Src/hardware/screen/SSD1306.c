/**
 * @file SSD1306.c
 * @brief Implement the functioning of the SSD1306 OLED screen via SPI and DMA
 * @author Gilles Henrard
 * @date 07/11/2023
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 */
#include "SSD1306.h"
#include "numbersVerdana16.h"
#include "SSD1306_registers.h"
#include "main.h"

//definitions
#define SSD1306_SPI_TIMEOUT_MS		10U		///< Maximum number of milliseconds SPI traffic should last before timeout
#define SSD1306_MAX_PARAMETERS		6U		///< Maximum number of parameters a command can have
#define SSD1306_MAX_DATA_SIZE		1024U	///< Maximum data size (128 * 64 bits / 8 bits per bytes)
#define SSD1306_MIN_ANGLE_DEG		-90.0f	///< Minimum angle allowed (in degrees)
#define SSD1306_MAX_ANGLE_DEG		90.0f	///< Maximum angle allowed (in degrees)
#define SSD1306_FLOAT_FACTOR_10		10.0f	///< Factor of 10 used in float calculations
#define SSD1306_INT_FACTOR_10		10U		///< Factor of 10 used in integer calculations
#define SSD1306_NEG_THRESHOLD		-0.05f	///< Threshold above which an angle is considered positive (circumvents float incaccuracies)
#define SSD1306_INDEX_SIGN			0		///< Index of the sign in the angle indexes array
#define SSD1306_INDEX_TENS			1U		///< Index of the tens in the angle indexes array
#define SSD1306_INDEX_UNITS			2U		///< Index of the units in the angle indexes array
#define SSD1306_INDEX_TENTHS		4U		///< Index of the tenths in the angle indexes array
#define SSD1306_ANGLE_NB_CHARS		6U		///< Number of characters in the angle array
#define SSD1306_NB_INIT_REGISERS	8U		///< Number of registers set at initialisation

//macros
#define SSD1306_ENABLE_SPI	HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_RESET);
#define SSD1306_DISABLE_SPI	HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_SET);
#define SSD1306_SET_COMMAND	HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_RESET);
#define SSD1306_SET_DATA	HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_SET);

/**
 * @brief Enumeration of the function IDs of the SSD1306
 */
typedef enum _SSD1306functionCodes_e{
	INIT = 0,		///< SSD1306initialise()
	SEND_CMD,		///< SSD1306sendCommand()
	SEND_DATA,		///< SSD1306sendData()
	CLR_SCREEN,		///< SSD1306clearScreen()
	PRT_ANGLE,		///< SSD1306_printAngle()
	PRINTING_ANGLE,	///< stPrintingAngle()
	WAITING_DMA_RDY	///< stWaitingForTXdone()
}_SSD1306functionCodes_e;

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
static errorCode_u SSD1306sendCommand(SSD1306register_e regNumber, const uint8_t parameters[], uint8_t nbParameters);
static errorCode_u SSD1306sendData(const uint8_t values[], uint16_t size);
static errorCode_u SSD1306clearScreen();

//state machine
static errorCode_u stIdle();
static errorCode_u stPrintingAngle();
static errorCode_u stWaitingForTXdone();

static const SSD1306init_t initCommands[SSD1306_NB_INIT_REGISERS] = {			///< Array used to initialise the registers
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
static SPI_HandleTypeDef*	SSD_SPIhandle = NULL;						///< SPI handle used with the SSD1306
static screenState			state = stIdle;								///< State machine current state
static uint8_t				screenBuffer[SSD1306_MAX_DATA_SIZE] = {0};	///< Buffer used to send data to the screen
volatile uint16_t			screenTimer_ms = 0;							///< Timer used with screen SPI transmissions
static float				nextAngle;									///< Buffer used to store an angle to print
static uint8_t				nextPage;									///< Buffer used to store a page at which to print
static uint8_t				nextColumn;									///< Buffer used to store a column at which to print


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
	SSD_SPIhandle = handle;

	//make sure to disable SSD1306 SPI communication
	SSD1306_DISABLE_SPI

	//reset the chip
	HAL_GPIO_WritePin(SSD1306_RST_GPIO_Port, SSD1306_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SSD1306_RST_GPIO_Port, SSD1306_RST_Pin, GPIO_PIN_SET);

	//initialisation taken from PDF p. 64 (Application Example)
	//	values which don't change from reset values aren't modified
	//TODO test for max oscillator frequency
	for(uint8_t i = 0 ; i < SSD1306_NB_INIT_REGISERS ; i++){
		result = SSD1306sendCommand(initCommands[i].reg, &initCommands[i].value, initCommands[i].nbParameters);
		if(IS_ERROR(result))
			return (pushErrorCode(result, INIT, 1));
	}

	result = SSD1306clearScreen();
	if(IS_ERROR(result))
		return (pushErrorCode(result, INIT, 2));		// @suppress("Avoid magic numbers")

	return (ERR_SUCCESS);
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
errorCode_u SSD1306sendCommand(SSD1306register_e regNumber, const uint8_t parameters[], uint8_t nbParameters){
	HAL_StatusTypeDef HALresult;
	errorCode_u result = ERR_SUCCESS;

	//if too many parameters, error
	if(nbParameters > SSD1306_MAX_PARAMETERS)
		return(createErrorCode(SEND_CMD, 1, ERR_WARNING));

	//set command pin and enable SPI
	SSD1306_SET_COMMAND
	SSD1306_ENABLE_SPI

	//send the command byte
	HALresult = HAL_SPI_Transmit(SSD_SPIhandle, &regNumber, 1, SSD1306_SPI_TIMEOUT_MS);
	if(HALresult != HAL_OK){
		SSD1306_DISABLE_SPI
		return (createErrorCodeLayer1(SEND_CMD, 2, HALresult, ERR_ERROR));
	}

	//if command send OK, send all parameters
	if(parameters && nbParameters){
		HALresult = HAL_SPI_Transmit(SSD_SPIhandle, (uint8_t*)parameters, nbParameters, SSD1306_SPI_TIMEOUT_MS);
		if(HALresult != HAL_OK)
			result = createErrorCodeLayer1(SEND_CMD, 3, HALresult, ERR_ERROR); 		// @suppress("Avoid magic numbers")
	}

	//disable SPI and return status
	SSD1306_DISABLE_SPI
	return (result);
}

/**
 * @brief Send data to the screen GDDRAM to be displayed
 *
 * @param values Data bytes to display
 * @param size Number of bytes to display
 * @retval 0 Success
 * @retval 1 Size above maximum
 * @retval 2 Error while sending data
 */
errorCode_u SSD1306sendData(const uint8_t values[], uint16_t size){
	errorCode_u result = ERR_SUCCESS;
	HAL_StatusTypeDef HALresult;

	//if nothing to send, exit
	if(!values || !size)
		return(ERR_SUCCESS);

	//if more bytes than sectors in the GDDRAM, error
	if(size > SSD1306_MAX_DATA_SIZE)
		return(createErrorCode(SEND_DATA, 1, ERR_WARNING));

	//set command pin and enable SPI
	SSD1306_SET_DATA
	SSD1306_ENABLE_SPI

	//transmit the buffer all at once
	HALresult = HAL_SPI_Transmit(SSD_SPIhandle, (uint8_t*)values, size, SSD1306_SPI_TIMEOUT_MS);
	if(HALresult != HAL_OK)
		result = createErrorCodeLayer1(SEND_DATA, 2, HALresult, ERR_ERROR);

	//disable SPI and return status
	SSD1306_DISABLE_SPI
	return (result);
}

/**
 * @brief Send the whole screen buffer to wipe it
 * @warning To use after initialisation so the buffer is clean
 *
 * @retval 0 Success
 * @retval 1 Error while sending the start/end columns
 * @retval 2 Error while sending the start/end pages
 * @retval 3 Error while sending the screen buffer
 */
errorCode_u SSD1306clearScreen(){
	const uint8_t limitColumns[2] = {0, 127};
	const uint8_t limitPages[2] = {0, 31};
	errorCode_u result;

	//set the start and end columns
	result = SSD1306sendCommand(COLUMN_ADDRESS, limitColumns, 2);
	if(IS_ERROR(result))
		return (pushErrorCode(result, CLR_SCREEN, 1));

	//set the start and end pages
	result = SSD1306sendCommand(PAGE_ADDRESS, limitPages, 2);
	if(IS_ERROR(result))
		return (pushErrorCode(result, CLR_SCREEN, 2));

	//send the buffer data
	result = SSD1306sendData(screenBuffer, SSD1306_MAX_DATA_SIZE);
	if(IS_ERROR(result))
		return (pushErrorCode(result, CLR_SCREEN, 3)); 		// @suppress("Avoid magic numbers")

	return (ERR_SUCCESS);
}

/**
 * @brief Check if the screen is ready to accept new commands
 *
 * @return 1 if ready
 */
uint8_t isScreenReady(){
	return (state == stIdle);
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
	//if angle out of bounds, return error
	if((angle < SSD1306_MIN_ANGLE_DEG) || (angle > SSD1306_MAX_ANGLE_DEG))
		return (createErrorCode(PRT_ANGLE, 1, ERR_WARNING));

	//store the values
	nextAngle = angle;
	nextPage = page;
	nextColumn = column;

	//get to printing state
	state = stPrintingAngle;
	return (ERR_SUCCESS);
}

/**
 * @brief Run the state machine
 *
 * @return Return code of the current state
 */
errorCode_u SSD1306update(){
	return ((*state)());
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
 * @brief State in which the screen is printing an angle label
 *
 * @retval 0 Success
 * @retval 1 Error occurred while sending the column address command
 * @retval 1 Error occurred while sending the page address command
 * @retval 1 Error occurred while sending the data
 */
errorCode_u stPrintingAngle(){
	uint8_t charIndexes[SSD1306_ANGLE_NB_CHARS] = {INDEX_PLUS, 0, 0, INDEX_DOT, 0, INDEX_DEG};
	const uint8_t limitColumns[2] = {nextColumn, nextColumn + (VERDANA_CHAR_WIDTH * 6) - 1};
	const uint8_t limitPages[2] = {nextPage, nextPage + 1};
	uint8_t* iterator = screenBuffer;
	errorCode_u result;
	HAL_StatusTypeDef HALresult;

	//if angle negative, replace plus sign with minus sign
	if(nextAngle < SSD1306_NEG_THRESHOLD){
		charIndexes[SSD1306_INDEX_SIGN] = INDEX_MINUS;
		nextAngle = -nextAngle;
	}

	//fill the angle characters indexes array with the float values (tens, units, tenths)
	charIndexes[SSD1306_INDEX_TENS] = (uint8_t)(nextAngle / SSD1306_FLOAT_FACTOR_10);
	charIndexes[SSD1306_INDEX_UNITS] = ((uint8_t)nextAngle) % SSD1306_INT_FACTOR_10;
	charIndexes[SSD1306_INDEX_TENTHS] = (uint8_t)((uint16_t)(nextAngle * SSD1306_FLOAT_FACTOR_10) % SSD1306_INT_FACTOR_10);

	//fill the buffer with all the required bitmaps bytes (column by column, then character by character, then page by page)
	for(nextPage = 0 ; nextPage < VERDANA_NB_PAGES ; nextPage++){
		for(uint8_t character = 0 ; character < SSD1306_ANGLE_NB_CHARS ; character++){
			for(nextColumn = 0 ; nextColumn < VERDANA_CHAR_WIDTH ; nextColumn++){
				*iterator = verdana_16ptNumbers[charIndexes[character]][(VERDANA_CHAR_WIDTH * nextPage) + nextColumn];
				iterator++;
			}
		}
	}

	//send the set start and end column addresses
	result = SSD1306sendCommand(COLUMN_ADDRESS, limitColumns, 2);
	if(IS_ERROR(result)){
		state = stIdle;
		return (pushErrorCode(result, PRINTING_ANGLE, 1));
	}

	//send the set start and end page addresses
	/*result = */SSD1306sendCommand(PAGE_ADDRESS, limitPages, 2);
	if(IS_ERROR(result)){
		state = stIdle;
		return (pushErrorCode(result, PRINTING_ANGLE, 2));
	}

	//set GPIOs
	SSD1306_SET_DATA
	SSD1306_ENABLE_SPI

	//send the data
	screenTimer_ms = SSD1306_SPI_TIMEOUT_MS;
	HALresult = HAL_SPI_Transmit_DMA(SSD_SPIhandle, screenBuffer, SSD1306_ANGLE_NB_CHARS * VERDANA_NB_BYTES_CHAR);
	if(HALresult != HAL_OK){
		state = stIdle;
		return (createErrorCodeLayer1(PRINTING_ANGLE, 3, HALresult, ERR_ERROR)); 	// @suppress("Avoid magic numbers")
	}

	//get to next
	state = stWaitingForTXdone;
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
		SSD1306_DISABLE_SPI
		HAL_SPI_DMAStop(SSD_SPIhandle);
		state = stIdle;
		return (createErrorCode(WAITING_DMA_RDY, 1, ERR_ERROR));
	}

	//if TX not done yet, exit
	if(HAL_SPI_GetState(SSD_SPIhandle) != HAL_SPI_STATE_READY)
		return (ERR_SUCCESS);

	//disable SPI and get to idle state
	SSD1306_DISABLE_SPI
	state = stIdle;
	return (ERR_SUCCESS);
}
