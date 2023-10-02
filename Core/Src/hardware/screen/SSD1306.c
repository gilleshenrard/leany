/**
 * @file SSD1306.c
 * @brief Implement the functioning of the SSD1306 OLED screen via SPI and DMA
 * @author Gilles Henrard
 * @date 02/10/2023
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 */
#include "SSD1306.h"

SPI_HandleTypeDef* SSD_SPIhandle = NULL;

void SSD1306initialise(SPI_HandleTypeDef* handle){
	SSD_SPIhandle = handle;
}

uint16_t SSD1306update(){
	return (0);
}
