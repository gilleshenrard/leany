/**
 * @file SSD1306.c
 * @brief Implement the functioning of the SSD1306 OLED screen via SPI and DMA
 * @author Gilles Henrard
 * @date 02/10/2023
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 */
#include "SSD1306.h"

SPI_HandleTypeDef* SSD_SPIhandle = NULL;	///< SPI handle used with the SSD1306

/**
 * @brief Initialise the SSD1306
 *
 * @param handle SPI handle used
 */
void SSD1306initialise(SPI_HandleTypeDef* handle){
	SSD_SPIhandle = handle;
}

/**
 * @brief Update the SSD1306
 *
 * @return Return code
 */
uint16_t SSD1306update(){
	return (0);
}

/**
 * @brief Write a byte value to a register
 *
 * @param regNumber Register number
 * @param value Value to write
 * @return Return code
 */
uint16_t SSD1306WriteValue(SSD1306register_e regNumber, uint8_t value){
	UNUSED(regNumber);
	UNUSED(value);
	return (0);
}
