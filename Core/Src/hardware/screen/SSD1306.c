/**
 * @file SSD1306.c
 * @brief Implement the functioning of the SSD1306 OLED screen via SPI and DMA
 * @author Gilles Henrard
 * @date 29/10/2023
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
#define SSD1306_INDEX_TENS			1U		///< Index of the tens in the angle indexes array
#define SSD1306_INDEX_UNITS			2U		///< Index of the units in the angle indexes array
#define SSD1306_INDEX_TENTHS		4U		///< Index of the tenths in the angle indexes array
#define SSD1306_ANGLE_NB_CHARS		6U		///< Number of characters in the angle array

//macros
#define SSD1306_ENABLE_SPI HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_RESET);
#define SSD1306_DISABLE_SPI HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_SET);
#define SSD1306_SET_COMMAND HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_RESET);
#define SSD1306_SET_DATA HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_SET);

/**
 * @brief Enumeration of the function IDs of the SSD1306
 */
typedef enum _SSD1306functionCodes_e{
	INIT = 0,	///< SSD1306initialise()
	SEND_CMD,	///< SSD1306sendCommand()
	SEND_DATA,	///< SSD1306sendData()
	CLR_SCREEN,	///< SSD1306clearScreen()
	PRT_ANGLE	///< SSD1306_printAngle()
}_SSD1306functionCodes_e;

//SPI handle
SPI_HandleTypeDef* SSD_SPIhandle = NULL;	///< SPI handle used with the SSD1306

//pre-configured values
const uint8_t hardwareConfigInit = SSD_PIN_CONFIG_ALT | SSD_COM_REMAP_DISABLE;	///< Default hardware config value
const uint8_t addressingModeInit = SSD_HORIZONTAL_ADDR;							///< Default addressing mode value
const uint8_t contrastInit = SSD_CONTRAST_HIGHEST;								///< Default contrast value
const uint8_t clockInit = SSD_CLOCK_FREQ_MID | SSD_CLOCK_DIVIDER_1;				///< Default clock initialisation value
const uint8_t chargePumpInit = SSD_ENABLE_CHG_PUMP;								///< Default charge pump value

//state variables
uint8_t screenBuffer[SSD1306_MAX_DATA_SIZE] = {0};	///< Buffer used to send data to the screen

//communication functions with the SSD1306
static errorCode_u SSD1306sendCommand(SSD1306register_e regNumber, const uint8_t parameters[], uint8_t nbParameters);
static errorCode_u SSD1306sendData(const uint8_t values[], uint16_t size);
static errorCode_u SSD1306clearScreen();

/**
 * @brief Initialise the SSD1306
 *
 * @param handle SPI handle used
 * @retval 0 Success
 * @retval 1 Error while setting the scanning direction
 * @retval 2 Error while setting the hardware config
 * @retval 3 Error while setting the segment remapping
 * @retval 4 Error while setting the memory addressing mode
 * @retval 5 Error while setting the contrast control
 * @retval 6 Error while setting the clock divider ratio
 * @retval 7 Error while setting the charge pump regulation
 * @retval 8 Error while turning the display on
 * @retval 9 Error while clearing the screen
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
	//TODO check for pins hardware config (0xDA)
	//TODO test for max oscillator frequency

	result = SSD1306sendCommand(SCAN_DIRECTION_N1_0, NULL, 0);
	if(IS_ERROR(result))
		return (pushErrorCode(result, INIT, 1));

	result = SSD1306sendCommand(HARDWARE_CONFIG, &hardwareConfigInit, 1);
	if(IS_ERROR(result))
		return (pushErrorCode(result, INIT, 2));

	result = SSD1306sendCommand(SEGMENT_REMAP_127, NULL, 0);
	if(IS_ERROR(result))
		return (pushErrorCode(result, INIT, 3));		// @suppress("Avoid magic numbers")

	result = SSD1306sendCommand(MEMORY_ADDR_MODE, &addressingModeInit, 1);
	if(IS_ERROR(result))
		return (pushErrorCode(result, INIT, 4));		// @suppress("Avoid magic numbers")

	result = SSD1306sendCommand(CONTRAST_CONTROL, &contrastInit, 1);
	if(IS_ERROR(result))
		return (pushErrorCode(result, INIT, 5));		// @suppress("Avoid magic numbers")

	result = SSD1306sendCommand(CLOCK_DIVIDE_RATIO, &clockInit, 1);
	if(IS_ERROR(result))
		return (pushErrorCode(result, INIT, 6));		// @suppress("Avoid magic numbers")

	result = SSD1306sendCommand(CHG_PUMP_REGULATOR, &chargePumpInit, 1);
	if(IS_ERROR(result))
		return (pushErrorCode(result, INIT, 7));		// @suppress("Avoid magic numbers")

	result = SSD1306sendCommand(DISPLAY_ON, NULL, 0);
	if(IS_ERROR(result))
		return (pushErrorCode(result, INIT, 8));		// @suppress("Avoid magic numbers")

	result = SSD1306clearScreen();
	if(IS_ERROR(result))
		return (pushErrorCode(result, INIT, 9));		// @suppress("Avoid magic numbers")

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
 * @brief Print an angle (in degrees, with sign) on the screen
 *
 * @param angle	Angle to print
 * @param page	First page on which to print the angle (screen line)
 * @param column First column on which to print the angle
 *
 * @retval 0 Success
 * @retval 1 Angle above maximum amplitude
 * @retval 2 Error while sending the start/end columns
 * @retval 3 Error while sending the start/end pages
 * @retval 3 Error while sending the screen buffer
 */
errorCode_u SSD1306_printAngle(float angle, uint8_t page, uint8_t column){
	uint8_t charIndexes[SSD1306_ANGLE_NB_CHARS] = {INDEX_PLUS, 0, 0, INDEX_DOT, 0, INDEX_DEG};
	const uint8_t limitColumns[2] = {column, column + (VERDANA_CHAR_WIDTH * 6) - 1};
	const uint8_t limitPages[2] = {page, page + 1};
	uint8_t* iterator = screenBuffer;
	errorCode_u result;

	//if angle out of bounds, return error
	if((angle < SSD1306_MIN_ANGLE_DEG) || (angle > SSD1306_MAX_ANGLE_DEG))
		return (createErrorCode(PRT_ANGLE, 1, ERR_WARNING));

	//if angle negative, replace plus sign with minus sign
	if(angle < SSD1306_NEG_THRESHOLD){
		charIndexes[0] = INDEX_MINUS;
		angle = -angle;
	}

	//fill the angle characters indexes array with the float values (tens, units, tenths)
	charIndexes[SSD1306_INDEX_TENS] = (uint8_t)(angle / SSD1306_FLOAT_FACTOR_10);
	charIndexes[SSD1306_INDEX_UNITS] = ((uint8_t)angle) % SSD1306_INT_FACTOR_10;
	charIndexes[SSD1306_INDEX_TENTHS] = (uint8_t)((uint16_t)(angle * SSD1306_FLOAT_FACTOR_10) % SSD1306_INT_FACTOR_10);

	//send the set start and end column addresses
	result = SSD1306sendCommand(COLUMN_ADDRESS, limitColumns, 2);
	if(IS_ERROR(result))
		return (pushErrorCode(result, PRT_ANGLE, 2));

	//send the set start and end page addresses
	/*result = */SSD1306sendCommand(PAGE_ADDRESS, limitPages, 2);
	if(IS_ERROR(result))
		return (pushErrorCode(result, PRT_ANGLE, 3));		// @suppress("Avoid magic numbers")

	//fill the buffer with all the required bitmaps bytes (column by column, then character by character, then page by page)
	for(page = 0 ; page < 2 ; page++){
		for(uint8_t character = 0 ; character < SSD1306_ANGLE_NB_CHARS ; character++){
			for(column = 0 ; column < VERDANA_CHAR_WIDTH ; column++){
				*iterator = verdana_16ptNumbers[charIndexes[character]][(VERDANA_CHAR_WIDTH * page) + column];
				iterator++;
			}
		}
	}

	//send the buffer
	result = SSD1306sendData(screenBuffer, VERDANA_NB_BYTES_CHAR * SSD1306_ANGLE_NB_CHARS);
	if(IS_ERROR(result))
		return (pushErrorCode(result, PRT_ANGLE, 4)); 		// @suppress("Avoid magic numbers")

	return (ERR_SUCCESS);
}
