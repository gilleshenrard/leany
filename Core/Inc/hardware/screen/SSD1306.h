#ifndef INC_HARDWARE_SCREEN_SSD1306_H_
#define INC_HARDWARE_SCREEN_SSD1306_H_
#include <stdint.h>
#include <stm32f1xx.h>

void SSD1306initialise(SPI_HandleTypeDef* handle);
uint16_t SSD1306update();

uint16_t SSD1306_printAngle(float angle, uint8_t page, uint8_t column);

#endif /* INC_HARDWARE_SCREEN_SSD1306_H_ */
