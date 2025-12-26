/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef UI_BITMAP_H
#define UI_BITMAP_H
#include <stdint.h>

#include "errorstack.h"

enum {
    kBitmapAlign = 16U,  ///< Memory alignment for the Bitmap structure
};

/**
 * Colour compensation offsets for RGB565 conversion.
 * These offsets adjust colours to better match designer intent
 * within RGB565 constraints, compensating for bit depth loss.
 */
typedef enum {
    kBluePerceptionOffset = 0x0002U,  ///< Bright blue compensation
} ColourShift;

/**
 * @brief Pixel type definition
 */
typedef uint16_t Pixel;

/**
 * @brief Bitmap row type definition
 */
typedef uint16_t BitmapRow;

/**
 * Enumeration of all the colours used
 */
typedef enum {
    kColourForeground = 0xFFFFU,                     ///< 0xFFFFFF in RGB565 big endian
    kColourBackground = 0xA210U,                     ///< 0x131313 in RGB565 big endian
    kColourEnabled = 0x518CU,                        ///< 0x898989 in RGB565 big endian
    kColourDisabled = 0x4529U,                       ///< 0x272727 in RGB565 big endian
    kColourDecoration = 0x0842U,                     ///< 0x424242 in RGB565 big endian
    kColourAccent = 0x7B1C + kBluePerceptionOffset,  ///< 0x188EE2 in RGB565 big endian
    kColourOK = 0xA854U,                             ///< 0x55963E in RGB565 big endian
    kColourWarning = 0xE4C4U,                        ///< 0xC59C20 in RGB565 big endian
    kColourCritical = 0xA2A8U,                       ///< 0xAC1414 in RGB565 big endian
    kColourCriticalOpacity50 = 0xA260U,              ///< 0x601414 in RGB565 big endian
    kColourCriticalOpacity28 = 0x8238U,              ///< 0x3E1313 in RGB565 big endian
} ColourBigEndian;

/**
 * Structure defining a Bitmap metadata
 */
typedef struct {
    Pixel* output_buffer;        ///< Buffer to which uncompress a bitmap
    uint8_t width_px;            ///< Width in [px] of the bitmap asset
    uint8_t height_px;           ///< Height in [px] of the bitmap asset
    uint8_t container_width_px;  ///< Width in [px] of the area containing the bitmap
    uint8_t x_offset_px;         ///< X offset in [px] of the bitmap in its container
    uint8_t y_offset_px;         ///< Y offset in [px] of the bitmap in its container
} __attribute__((aligned(kBitmapAlign))) Bitmap;

ErrorCode uncompressBitmap(Bitmap* metadata, const void* bitmap, ColourBigEndian foreground_colour);

#endif
