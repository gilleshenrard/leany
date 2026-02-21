/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file icons.h
 * @author Gilles Henrard
 */
#ifndef INC_HARDWARE_SCREEN_ICONS_H
#define INC_HARDWARE_SCREEN_ICONS_H
#include <stdbool.h>

#include "bitmap.h"
#include "errorstack.h"

/**
 * Enumeration of the icons available
 */
typedef enum {
    kIconHold = 0,  ///< Hold icon
    kIconAbsolute,  ///< Absolute measurements icon
    kIconRelative,  ///< Relative measurements icon
    kIconPitch,     ///< Pitch axis icon
    kIconRoll,      ///< Roll axis icon
    kIconBattery,   ///< Battery icon (without level)
    kIconWarning,   ///< Large warning icon
    kNbIcons,       ///< Number of icons declared
} Icon;

/**
 * Icon metadata
 */
typedef struct {
    uint8_t width_px;     ///< Icon width in [px]
    uint8_t height_px;    ///< Icon height in [px]
    uint8_t pool_offset;  ///< Offset of the first icon byte in the bitmaps pool
} IconMetadata;

ErrorCode uncompressIcon(Icon icon, Bitmap* bitmap_metadata, ColourBigEndian foreground_colour);
void drawBatteryLevelBars(Pixel buffer[], uint8_t battery_percent, bool charging);
uint8_t getIconWidth(Icon icon);
uint8_t getIconHeight(Icon icon);

#endif
