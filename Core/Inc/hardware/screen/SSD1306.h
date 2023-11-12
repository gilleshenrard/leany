#ifndef INC_HARDWARE_SCREEN_SSD1306_H_
#define INC_HARDWARE_SCREEN_SSD1306_H_
#include <stdint.h>
#include <stm32f1xx.h>
#include "errors.h"

//screen defaults
#define SSD1306_LINE1_PAGE		0U		///< Page number of the first screen line
#define SSD1306_LINE1_COLUMN	0U		///< Column number of the first screen line
#define SSD1306_LINE2_PAGE		3U		///< Page number of the second screen line
#define SSD1306_LINE2_COLUMN	0U		///< Column number of the second screen line

extern volatile uint16_t	screenTimer_ms;

errorCode_u SSD1306initialise(SPI_HandleTypeDef* handle);
errorCode_u SSD1306update();
uint8_t isScreenReady();
errorCode_u SSD1306clearScreen();
errorCode_u SSD1306_printAngle(float angle, uint8_t page, uint8_t column);

#endif /* INC_HARDWARE_SCREEN_SSD1306_H_ */
