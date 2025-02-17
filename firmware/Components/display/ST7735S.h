#ifndef INC_HARDWARE_SCREEN_ST7735S_H_
#define INC_HARDWARE_SCREEN_ST7735S_H_

#include <stdint.h>
#include "ST7735_initialisation.h"
#include "errorstack.h"
#include "stm32f103xb.h"

enum {
    MESSAGE_ALIGNMENT = 8U,
};

typedef enum {
    MSG_HOLD = 0,
    MSG_ZERO,
    MSG_PWROFF,
    NB_MESSAGES
} messageID_e;

typedef struct {
    messageID_e ID;
    int16_t     value;
} __attribute__((aligned(MESSAGE_ALIGNMENT))) displayMessage_t;

errorCode_u createST7735Stask(SPI_TypeDef* handle, DMA_TypeDef* dma, uint32_t dmaChannel);
errorCode_u st7735sSetOrientation(orientation_e orientation);
uint8_t     sendDisplayMessage(const displayMessage_t* message);
void        st7735sDMAinterruptHandler(void);
#endif
