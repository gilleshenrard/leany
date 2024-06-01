#ifndef INC_HARDWARE_SCREEN_SSD1306_H_
#define INC_HARDWARE_SCREEN_SSD1306_H_
#include <main.h>
#include <stdint.h>
#include "errorstack.h"

/**
 * @brief Enumeration of the printable rotation axis
 */
typedef enum {
    ROLL = 0,
    PITCH
} rotationAxis_e;

/**
 * @brief Enumeration of the referential used
 */
typedef enum {
    ABSOLUTE = 0,
    RELATIVE
} referentialType_e;

extern volatile uint16_t screenTimer_ms;
extern volatile uint16_t ssd1306SPITimer_ms;

uint8_t     isScreenReady();
errorCode_u SSD1306initialise(SPI_TypeDef* handle, DMA_TypeDef* dma, uint32_t dmaChannel);
errorCode_u SSD1306update();
errorCode_u SSD1306drawBaseScreen();
errorCode_u SSD1306_printAngleTenths(int16_t angle, rotationAxis_e rotationAxis);
errorCode_u SSD1306_printReferentialIcon(referentialType_e type);
errorCode_u SSD1306_printHoldIcon(uint8_t status);
errorCode_u SSD1306_turnDisplayOFF();

#endif /* INC_HARDWARE_SCREEN_SSD1306_H_ */
