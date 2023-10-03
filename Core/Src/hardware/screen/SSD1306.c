/**
 * @file SSD1306.c
 * @brief Implement the functioning of the SSD1306 OLED screen via SPI and DMA
 * @author Gilles Henrard
 * @date 03/10/2023
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 */
#include "SSD1306.h"
#include "main.h"

#define SSD1306_SUCCESS			0x00
#define SSD1306_SPI_TIMEOUT_MS	10U
#define SSD1306_MAX_PARAMETERS	6U

#define SSD1306_ENABLE_SPI HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_RESET);
#define SSD1306_DISABLE_SPI HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_SET);
#define SSD1306_SET_COMMAND HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_RESET);
#define SSD1306_SET_DATA HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_SET);

SPI_HandleTypeDef* SSD_SPIhandle = NULL;	///< SPI handle used with the SSD1306

const uint8_t contrastInit = SSD_CONTRAST_HIGHEST;
const uint8_t clockInit = SSD_CLOCK_FREQ_MID | SSD_CLOCK_DIVIDER_1;
const uint8_t chargePumpInit = SSD_ENABLE_CHG_PUMP;

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

	SSD1306sendCommand(CONTRAST_CONTROL, &contrastInit, 1);
	SSD1306sendCommand(CLOCK_DIVIDE_RATIO, &clockInit, 1);
	SSD1306sendCommand(CHG_PUMP_REGULATOR, &chargePumpInit, 1);
	SSD1306sendCommand(DISPLAY_ON, NULL, 0);
	SSD1306sendCommand(DISPLAY_ALL_ON, NULL, 0);
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
