#include "SSD1306.h"

SPI_HandleTypeDef* SSD_SPIhandle = NULL;

void SSD1306initialise(SPI_HandleTypeDef* handle){
	SSD_SPIhandle = handle;
}

uint16_t SSD1306update(){
	return (0);
}
