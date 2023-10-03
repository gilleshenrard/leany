#ifndef INC_HARDWARE_SCREEN_SSD1306_H_
#define INC_HARDWARE_SCREEN_SSD1306_H_
#include <stdint.h>
#include <stm32f1xx.h>
#include "SSD1306_registers.h"

extern volatile uint16_t screenTimer_ms;

void SSD1306initialise(SPI_HandleTypeDef* handle);
uint16_t SSD1306update();
uint16_t SSD1306sendCommand(SSD1306register_e regNumber, const uint8_t parameters[], uint8_t nbParameters);
uint16_t SSD1306sendData(const uint8_t values[], uint16_t size);

#endif /* INC_HARDWARE_SCREEN_SSD1306_H_ */
