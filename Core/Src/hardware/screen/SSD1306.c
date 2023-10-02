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

	//initialisation taken from PDF p. 64 (Application Example)
	//	values which don't change from reset values aren't modified
	//TODO check for pins hardware config (0xDA)
	//TODO test for max oscillator frequency
	//TODO check for charge pump

	SSD1306WriteValue(CONTRAST_CONTROL, SSD_CONTRAST_HIGHEST);
	SSD1306WriteValue(CLOCK_DIVIDE_RATIO, SSD_CLOCK_FREQ_MID | SSD_CLOCK_DIVIDER_1);
	SSD1306WriteValue(CHG_PUMP_REGULATOR, SSD_ENABLE_CHG_PUMP);
	SSD1306WriteRegister(DISPLAY_ON);
}

/**
 * @brief Update the SSD1306
 *
 * @return Return code
 */
uint16_t SSD1306update(){
	return (0);
}

uint16_t SSD1306WriteRegister(SSD1306register_e regNumber){
	UNUSED(regNumber);
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
