#ifndef INC_HARDWARE_SCREEN_ST7735S_H_
#define INC_HARDWARE_SCREEN_ST7735S_H_

#include <stdint.h>
#include "ST7735_initialisation.h"
#include "errorstack.h"
#include "stm32f103xb.h"

enum {
    MESSAGE_ALIGNMENT = 8U,
};

/**
 * Enumeration of the message types
 */
typedef enum {
    MSG_HOLD = 0,  ///< A hold operation has been performed
    MSG_ZERO,      ///< A Zeroing operation has been performed
    MSG_PWROFF,    ///< A power off operation has been performed
    NB_MESSAGES    ///< Number of available messages
} messageID_e;

/**
 * Structure of a message in the queue used to communicate with the ST7735S
 */
typedef struct {
    messageID_e ID;     ///< Message type
    int16_t     value;  ///< Message value
} __attribute__((aligned(MESSAGE_ALIGNMENT))) displayMessage_t;

errorCode_u createST7735Stask(SPI_TypeDef* handle, DMA_TypeDef* dma, uint32_t dmaChannel);
errorCode_u st7735sSetOrientation(orientation_e orientation);
uint8_t     sendDisplayMessage(const displayMessage_t* message);
void        st7735sDMAinterruptHandler(void);
#endif
