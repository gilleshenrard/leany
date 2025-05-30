#ifndef INC_HARDWARE_SCREEN_ST7735S_H_
#define INC_HARDWARE_SCREEN_ST7735S_H_
#include <stdint.h>
#include "FreeRTOS.h"
#include "ST7735_initialisation.h"
#include "errorstack.h"
#include "task.h"

enum {
    DISPLAY_WIDTH  = 160U,  ///< Number of pixels in width
    DISPLAY_HEIGHT = 128U,  ///< Number of pixels in height
};

typedef struct {
    uint8_t x0;
    uint8_t y0;
    uint8_t x1;
    uint8_t y1;
} __attribute((aligned(4))) area_t;

errorCode_u st7735sSetOrientation(orientation_e orientation);
void        attachUItask(TaskHandle_t handle);
errorCode_u configureST7735S(void);
errorCode_u setWindow(uint8_t Xstart, uint8_t Ystart, uint8_t width, uint8_t height);
void        turnBacklightON(void);
errorCode_u sendScreenData(const uint16_t data[], size_t nbBytes, size_t maxBytes, const area_t* screenArea);

static inline uint8_t getAreaWidth(const area_t* area) {
    return (uint8_t)(area->x1 - area->x0 + 1U);
}

static inline uint8_t getAreaHeight(const area_t* area) {
    return (uint8_t)(area->y1 - area->y0 + 1U);
}
#endif
