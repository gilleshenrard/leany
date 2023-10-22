/**
 * @file SSD1306.c
 * @brief Implement the functioning of the SSD1306 OLED screen via SPI and DMA
 * @author Gilles Henrard
 * @date 22/10/2023
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 */
#include "SSD1306.h"
#include "numbersVerdana16.h"
#include "SSD1306_registers.h"
#include "main.h"

//definitions
#define SSD1306_SUCCESS			0x00	///< Return code corresponding to a success
#define SSD1306_SPI_TIMEOUT_MS	10U		///< Maximum number of milliseconds SPI traffic should last before timeout
#define SSD1306_MAX_PARAMETERS	6U		///< Maximum number of parameters a command can have
#define SSD1306_MAX_DATA_SIZE	1024U	///< Maximum data size (128 * 64 bits / 8 bits per bytes)
#define SSD1306_MIN_ANGLE_DEG	-90.0f	///< Minimum angle allowed (in degrees)
#define SSD1306_MAX_ANGLE_DEG	90.0f	///< Maximum angle allowed (in degrees)
#define SSD1306_FLOAT_FACTOR_10	10.0f	///< Factor of 10 used in float calculations
#define SSD1306_INT_FACTOR_10	10U		///< Factor of 10 used in integer calculations
#define SSD1306_NEG_THRESHOLD	-0.05f	///< Threshold above which an angle is considered positive (circumvents float incaccuracies)
#define SSD1306_INDEX_TENS		1U		///< Index of the tens in the angle indexes array
#define SSD1306_INDEX_UNITS		2U		///< Index of the units in the angle indexes array
#define SSD1306_INDEX_TENTHS	4U		///< Index of the tenths in the angle indexes array
#define SSD1306_ANGLE_NB_CHARS	6U		///< Number of characters in the angle array

//macros
#define SSD1306_ENABLE_SPI HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_RESET);
#define SSD1306_DISABLE_SPI HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_SET);
#define SSD1306_SET_COMMAND HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_RESET);
#define SSD1306_SET_DATA HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_SET);

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
static uint16_t SSD1306sendCommand(SSD1306register_e regNumber, const uint8_t parameters[], uint8_t nbParameters);
static uint16_t SSD1306sendData(const uint8_t values[], uint16_t size);
static uint16_t SSD1306clearScreen();

/**
 * @brief Initialise the SSD1306
 *
 * @param handle SPI handle used
 */
void SSD1306initialise(SPI_HandleTypeDef* handle){
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
	//TODO check for charge pump

	SSD1306sendCommand(SCAN_DIRECTION_N1_0, NULL, 0);
	SSD1306sendCommand(HARDWARE_CONFIG, &hardwareConfigInit, 1);
	SSD1306sendCommand(SEGMENT_REMAP_127, NULL, 0);
	SSD1306sendCommand(MEMORY_ADDR_MODE, &addressingModeInit, 1);
	SSD1306sendCommand(CONTRAST_CONTROL, &contrastInit, 1);
	SSD1306sendCommand(CLOCK_DIVIDE_RATIO, &clockInit, 1);
	SSD1306sendCommand(CHG_PUMP_REGULATOR, &chargePumpInit, 1);
	SSD1306sendCommand(DISPLAY_ON, NULL, 0);

	SSD1306clearScreen();
}

/**
 * @brief Update the SSD1306
 *
 * @return Return code
 */
uint16_t SSD1306update(){
	return (SSD1306_SUCCESS);
}

/**
 * @brief Send a command with parameters
 *
 * @param regNumber Register number
 * @param parameters Parameters to write
 * @param nbParameters Number of parameters to write
 * @return Return code
 */
uint16_t SSD1306sendCommand(SSD1306register_e regNumber, const uint8_t parameters[], uint8_t nbParameters){
	HAL_StatusTypeDef result;

	//if too many parameters, error
	if(nbParameters > SSD1306_MAX_PARAMETERS)
		return(HAL_ERROR);

	//set command pin and enable SPI
	SSD1306_SET_COMMAND
	SSD1306_ENABLE_SPI

	//send the command byte
	result = HAL_SPI_Transmit(SSD_SPIhandle, &regNumber, 1, SSD1306_SPI_TIMEOUT_MS);

	//if command send OK, send all parameters
	if(parameters && nbParameters && (result == HAL_OK))
		result = HAL_SPI_Transmit(SSD_SPIhandle, (uint8_t*)parameters, nbParameters, SSD1306_SPI_TIMEOUT_MS);

	//disable SPI and return status
	SSD1306_DISABLE_SPI
	return (result != HAL_OK);
}

/**
 * @brief Send data to the screen GDDRAM to be displayed
 *
 * @param values Data bytes to display
 * @param size Number of bytes to display
 * @return
 */
uint16_t SSD1306sendData(const uint8_t values[], uint16_t size){
	HAL_StatusTypeDef result;

	//if nothing to send, exit
	if(!values || !size)
		return(0);

	//if more bytes than sectors in the GDDRAM, error
	if(size > SSD1306_MAX_DATA_SIZE)
		return(1);

	//set command pin and enable SPI
	SSD1306_SET_DATA
	SSD1306_ENABLE_SPI

	//transmit the buffer all at once
	result = HAL_SPI_Transmit(SSD_SPIhandle, (uint8_t*)values, size, SSD1306_SPI_TIMEOUT_MS);

	//disable SPI and return status
	SSD1306_DISABLE_SPI
	return (result != HAL_OK);
}

/**
 * @brief Send the whole screen buffer to wipe it
 * @warning To use after initialisation so the buffer is clean
 *
 * @return 0
 */
uint16_t SSD1306clearScreen(){
	const uint8_t limitColumns[2] = {0, 127};
	const uint8_t limitPages[2] = {0, 31};

	SSD1306sendCommand(COLUMN_ADDRESS, limitColumns, 2);
	SSD1306sendCommand(PAGE_ADDRESS, limitPages, 2);

	SSD1306sendData(screenBuffer, SSD1306_MAX_DATA_SIZE);

	return (0);
}

/**
 * @brief Print an angle (in degrees, with sign) on the screen
 *
 * @param angle	Angle to print
 * @param page	First page on which to print the angle (screen line)
 * @param column First column on which to print the angle
 * @return Error code
 */
uint16_t SSD1306_printAngle(float angle, uint8_t page, uint8_t column){
	uint8_t charIndexes[SSD1306_ANGLE_NB_CHARS] = {INDEX_PLUS, 0, 0, INDEX_DOT, 0, INDEX_DEG};
	const uint8_t limitColumns[2] = {column, column + (VERDANA_CHAR_WIDTH * 6) - 1};
	const uint8_t limitPages[2] = {page, page + 1};
	uint8_t* iterator = screenBuffer;

	//if angle out of bounds, return error
	if((angle < SSD1306_MIN_ANGLE_DEG) || (angle > SSD1306_MAX_ANGLE_DEG))
		return (1);

	//if angle negative, replace plus sign with minus sign
	if(angle < SSD1306_NEG_THRESHOLD){
		charIndexes[0] = INDEX_MINUS;
		angle = -angle;
	}

	//fill the angle characters indexes array with the float values (tens, units, tenths)
	charIndexes[SSD1306_INDEX_TENS] = (uint8_t)(angle / SSD1306_FLOAT_FACTOR_10);
	charIndexes[SSD1306_INDEX_UNITS] = ((uint8_t)angle) % SSD1306_INT_FACTOR_10;
	charIndexes[SSD1306_INDEX_TENTHS] = (uint8_t)((uint16_t)(angle * SSD1306_FLOAT_FACTOR_10) % SSD1306_INT_FACTOR_10);

	//send the set column and set page commands
	SSD1306sendCommand(COLUMN_ADDRESS, limitColumns, 2);
	SSD1306sendCommand(PAGE_ADDRESS, limitPages, 2);

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
	SSD1306sendData(screenBuffer, VERDANA_NB_BYTES_CHAR * SSD1306_ANGLE_NB_CHARS);
	return (0);
}
