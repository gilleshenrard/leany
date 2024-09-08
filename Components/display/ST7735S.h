#ifndef INC_HARDWARE_SCREEN_ST7735S_H_
#define INC_HARDWARE_SCREEN_ST7735S_H_

#include "ST7735_initialisation.h"
#include "errorstack.h"
#include "stm32f103xb.h"

errorCode_u st7735sInitialise(SPI_TypeDef* handle, DMA_TypeDef* dma, uint32_t dmaChannel);
errorCode_u st7735sUpdate(void);
errorCode_u st7735sSetOrientation(orientation_e orientation);
#endif
