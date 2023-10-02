/**
 * @file SSD1306.c
 * @brief Implement the functioning of the SSD1306 OLED screen via SPI and DMA
 * @author Gilles Henrard
 * @date 02/10/2023
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 */
#include "SSD1306.h"
#include "main.h"

#define SSD1306_SUCCESS			0x00
#define SSD1306_SPI_TIMEOUT_MS	10

#define SSD1306_ENABLE_SPI HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_RESET);
#define SSD1306_DISABLE_SPI HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_SET);

#define SSD1306_SET_COMMAND HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_RESET);
#define SSD1306_SET_DATA HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_SET);

SPI_HandleTypeDef* SSD_SPIhandle = NULL;	///< SPI handle used with the SSD1306

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

	SSD1306WriteValue(CONTRAST_CONTROL, SSD_CONTRAST_HIGHEST);
	SSD1306WriteValue(CLOCK_DIVIDE_RATIO, SSD_CLOCK_FREQ_MID | SSD_CLOCK_DIVIDER_1);
	SSD1306WriteValue(CHG_PUMP_REGULATOR, SSD_ENABLE_CHG_PUMP);
	SSD1306WriteRegister(DISPLAY_ON);
	SSD1306WriteRegister(DISPLAY_ALL_ON);
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
 * @brief Call a register command
 *
 * @param regNumber Register number
 * @return Return code
 */
uint16_t SSD1306WriteRegister(SSD1306register_e regNumber){
	HAL_StatusTypeDef result;

	SSD1306_ENABLE_SPI

	SSD1306_SET_COMMAND
	result = HAL_SPI_Transmit(SSD_SPIhandle, &regNumber, 1, SSD1306_SPI_TIMEOUT_MS);

	SSD1306_DISABLE_SPI

	return (result != HAL_OK);
}

/**
 * @brief Write a byte value to a register
 *
 * @param regNumber Register number
 * @param value Value to write
 * @return Return code
 */
uint16_t SSD1306WriteValue(SSD1306register_e regNumber, uint8_t value){
	HAL_StatusTypeDef result;

	SSD1306_ENABLE_SPI

	SSD1306_SET_COMMAND
	result = HAL_SPI_Transmit(SSD_SPIhandle, &regNumber, 1, SSD1306_SPI_TIMEOUT_MS);
	if(result == HAL_OK)
		result = HAL_SPI_Transmit(SSD_SPIhandle, &value, 1, SSD1306_SPI_TIMEOUT_MS);

	SSD1306_DISABLE_SPI

	return (result != HAL_OK);
}
