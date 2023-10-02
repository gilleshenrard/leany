#ifndef INC_HARDWARE_SCREEN_SSD1306_H_
#define INC_HARDWARE_SCREEN_SSD1306_H_
#include <stdint.h>
#include <stm32f1xx.h>
#include "SSD1306_registers.h"

void SSD1306initialise(SPI_HandleTypeDef* handle);
uint16_t SSD1306update();
uint16_t SSD1306WriteRegister(SSD1306register_e regNumber);
uint16_t SSD1306WriteValue(SSD1306register_e regNumber, uint8_t value);

#endif /* INC_HARDWARE_SCREEN_SSD1306_H_ */
