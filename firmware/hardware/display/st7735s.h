/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef INC_HARDWARE_SCREEN_ST7735S_H
#define INC_HARDWARE_SCREEN_ST7735S_H
#include <FreeRTOS.h>
#include <stdint.h>
#include <task.h>

#include "errorstack.h"
#include "st7735_initialisation.h"

enum {
    kDisplayWidth = 160U,   ///< Number of pixels in width
    kDisplayHeight = 128U,  ///< Number of pixels in height
};

/**
 * Structure defining the coordinates of a window on the display
 */
typedef struct {
    uint8_t x0;  ///< X value of the top-left corner of the window in [pixels]
    uint8_t y0;  ///< Y value of the top-left corner of the window in [pixels]
    uint8_t x1;  ///< X value of the bottom-right corner of the window in [pixels]
    uint8_t y1;  ///< Y value of the bottom-right corner of the window in [pixels]
} __attribute((aligned(4))) Area;

ErrorCode st7735sSetOrientation(Orientation orientation);
void attachUItask(TaskHandle_t handle);
ErrorCode configureST7735S(void);
ErrorCode setWindow(uint8_t x_start, uint8_t y_start, uint8_t width, uint8_t height);
void turnBacklightON(void);
ErrorCode sendScreenData(const uint16_t data[], size_t nb_bytes, size_t max_bytes, const Area* screen_area);

/**
 * Get the width of a display window in [pixels]
 *
 * @param area Window of which get the width
 * @return Width in [pixels]
 */
static inline uint8_t getAreaWidth(const Area* area) { return (uint8_t)(area->x1 - area->x0 + 1U); }

/**
 * Get the height of a display window in [pixels]
 *
 * @param area Window of which get the height
 * @return Height in [pixels]
 */
static inline uint8_t getAreaHeight(const Area* area) { return (uint8_t)(area->y1 - area->y0 + 1U); }
#endif
