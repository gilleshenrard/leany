/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file label.h
 * @author Gilles Henrard
 */
#ifndef UI_LABEL_H
#define UI_LABEL_H
#include <stdint.h>

#include "bitmap.h"
#include "errorstack.h"
#include "fonts.h"

/**
 * Label string horizontal alignment
 */
typedef enum {
    kAlignmentCentered = 0,  ///< Centered (default)
    kAlignmentLeft,          ///< Left
    kAlignmentRight,         ///< Right
} Alignment;

/**
 * String label to display on screen
 */
typedef struct {
    uint8_t x_left;            ///< Left X coordinate of the label, relative to the screen
    uint8_t y_top;             ///< Top Y coordinate of the label, relative to the screen
    uint8_t width_px;          ///< Width of the label (used because a string can have variable width)
    const FontMetadata* font;  ///< Font with which display the label
    Alignment alignment;       ///< String horizontal alignment
} Label;

ErrorCode uncompressLabel(const Label* label, Pixel* buffer, const char string[], uint8_t string_length,
                          ColourBigEndian foreground_colour);

#endif
