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

/**
 * Structure defining the coordinates of a window on the display
 */
typedef struct {
    uint8_t x0;  ///< X value of the top-left corner of the window in [pixels]
    uint8_t y0;  ///< Y value of the top-left corner of the window in [pixels]
    uint8_t x1;  ///< X value of the bottom-right corner of the window in [pixels]
    uint8_t y1;  ///< Y value of the bottom-right corner of the window in [pixels]
} __attribute((aligned(4))) area_t;

errorCode_u st7735sSetOrientation(orientation_e orientation);
void        attachUItask(TaskHandle_t handle);
errorCode_u configureST7735S(void);
errorCode_u setWindow(uint8_t Xstart, uint8_t Ystart, uint8_t width, uint8_t height);
void        turnBacklightON(void);
errorCode_u sendScreenData(const uint16_t data[], size_t nbBytes, size_t maxBytes, const area_t* screenArea);

/**
 * Get the width of a display window in [pixels]
 *
 * @param area Window of which get the width
 * @return Width in [pixels]
 */
static inline uint8_t getAreaWidth(const area_t* area) {
    return (uint8_t)(area->x1 - area->x0 + 1U);
}

/**
 * Get the height of a display window in [pixels]
 *
 * @param area Window of which get the height
 * @return Height in [pixels]
 */
static inline uint8_t getAreaHeight(const area_t* area) {
    return (uint8_t)(area->y1 - area->y0 + 1U);
}
#endif
