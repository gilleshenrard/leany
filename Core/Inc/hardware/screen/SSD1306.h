#ifndef INC_HARDWARE_SCREEN_SSD1306_H_
#define INC_HARDWARE_SCREEN_SSD1306_H_
#include <main.h>
#include <stdint.h>
#include "errorstack.h"

//screen defaults
#define SSD1306_LINE1_PAGE		1U		///< Page number of the first screen line
#define SSD1306_LINE1_COLUMN	23U		///< Column number of the first screen line
#define SSD1306_LINE2_PAGE		5U		///< Page number of the second screen line
#define SSD1306_LINE2_COLUMN	23U		///< Column number of the second screen line

extern volatile uint16_t	screenTimer_ms;
extern volatile uint16_t	ssd1306SPITimer_ms;

uint8_t isScreenReady();
errorCode_u SSD1306initialise(SPI_TypeDef* handle, DMA_TypeDef* dma, uint32_t dmaChannel);
errorCode_u SSD1306update();
errorCode_u SSD1306drawBaseScreen();
errorCode_u SSD1306_printAngleTenths(int16_t angle, uint8_t page, uint8_t column);

#endif /* INC_HARDWARE_SCREEN_SSD1306_H_ */
