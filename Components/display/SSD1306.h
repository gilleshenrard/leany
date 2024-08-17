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

errorCode_u ssd1306Initialise(SPI_TypeDef* handle, DMA_TypeDef* dma, uint32_t dmaChannel);
errorCode_u ssd1306Update();
uint8_t     isScreenReady();
errorCode_u ssd1306PrintAngleTenths(int16_t angleTenths, rotationAxis_e rotationAxis);
errorCode_u ssd1306PrintReferentialIcon(referentialType_e type);
errorCode_u ssd1306PrintHoldIcon(uint8_t status);
errorCode_u ssd1306TurnDisplayOFF();

#endif /* INC_HARDWARE_SCREEN_SSD1306_H_ */
