#include "SSD1306.h"

I2C_HandleTypeDef* SSD_I2Chandle = NULL;

void SSD1306initialise(I2C_HandleTypeDef* handle){
	SSD_I2Chandle = handle;
}

uint16_t SSD1306update(){
	return (0);
}
