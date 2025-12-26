/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef UI_DISPLAY_H
#define UI_DISPLAY_H
#include "battery.h"
#include "bitmap.h"
#include "icons.h"
#include "label.h"
#include "orientation.inc"

enum {
    kStatusIconsY = 6U,      ///< Y coordinate of the status icons on screen
    kHoldIconX = 6U,         ///< X coordinate of the Hold icon
    kModeIconX = 21U,        ///< X coordinate of the Mode icon
    kStatusBarHeight = 22U,  ///< Height of the status bar
};

/**
 * Forward-declaration of Area to avoid circular reference
 */
typedef struct TypeArea Area;

/**
 * Battery level and status indicator
 */
typedef struct {
    uint8_t x;    ///< Indicator X coordinate
    uint8_t y;    ///< Indicator Y coordinate
    Label label;  ///< Percentage label
} BatteryIndicator;

ErrorCode printLabel(const Label* label, const char* string, uint8_t string_size, ColourBigEndian foreground_colour);
ErrorCode fillBackground(Pixel* buffer, size_t buffer_size, Orientation orientation, ColourBigEndian colour);
ErrorCode printRectangle(Pixel* buffer, size_t buffer_size, const Area* area, ColourBigEndian colour);
ErrorCode printIcon(Icon icon, uint8_t x_left_px, uint8_t y_top_px, ColourBigEndian foreground_colour);
ErrorCode printBatteryIndicator(const BatteryStatus* status, BatteryIndicator* indicator);
ErrorCode printStatusBar(uint8_t holding_status, uint8_t zeroing_status, uint8_t screen_width_px);
BatteryIndicator* getBatteryIndicator(void);

#endif  //UI_DISPLAY_H
