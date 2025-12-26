/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#include "bitmap.h"

#include <assert.h>
#include <stdint.h>

#include "errorstack.h"

enum {
    kSmallRowWidthPx = 16U,  ///< Maximum number of pixels in a small icon row
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
static_assert(kSmallRowWidthPx == (sizeof(BitmapRow) * 8U), "Wrong size for the small bitmap row");

static void uncompressRow16(const Bitmap* metadata, Pixel* output_buffer, const BitmapRow* bitmap, uint8_t row,
                            ColourBigEndian foreground_colour);
static void uncompressRow32(const Bitmap* metadata, Pixel* output_buffer, const uint32_t* bitmap, uint8_t row,
                            ColourBigEndian foreground_colour);

/**
 * Decompress a metadata bitmap into a pixel buffer
 *
 * @param metadata Bitmap metadata (output buffer, offsets, area width)
 * @param bitmap Pointer to the metadata bitmap data (row-major, MSB-first)
 * @param foreground_colour Colour to use for foreground pixels (1 bits)
 * @retval 0 Success
 * @retval 1 NULL pointer provided
 * @retval 2 X offset and metadata width out of container width range
 */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
ErrorCode uncompressBitmap(Bitmap* metadata, const void* bitmap, ColourBigEndian foreground_colour) {
    if (!bitmap || !metadata) {
        return createErrorCode(1, 1, kErrorError);
    }

    if (!metadata->container_width_px || !metadata->width_px || !metadata->height_px) {
        return kSuccessCode;
    }

    if (((uint16_t)metadata->x_offset_px + (uint16_t)metadata->width_px) > metadata->container_width_px) {
        return createErrorCode(1, 2, kErrorError);
    }

    for (uint8_t row = 0; row < metadata->height_px; row++) {
        const uint16_t container_offset =
            (uint16_t)((row + metadata->y_offset_px) * metadata->container_width_px) + metadata->x_offset_px;

        Pixel* pixel = &metadata->output_buffer[container_offset];
        if (metadata->width_px <= kSmallRowWidthPx) {
            uncompressRow16(metadata, pixel, bitmap, row, foreground_colour);
        } else {
            uncompressRow32(metadata, pixel, bitmap, row, foreground_colour);
        }
    }

    return kSuccessCode;
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Uncompress a bitmap with maximum 16 pixels of width
 *
 * @param metadata Bitmap metadata (output buffer, offsets, area width)
 * @param[out] output_buffer Buffer to which uncompress the bitmap
 * @param bitmap Pointer to the metadata bitmap data (row-major, MSB-first)
 * @param row Bitmap row to uncompress
 * @param foreground_colour Colour to use for foreground pixels (1 bits)
 */
static void uncompressRow16(const Bitmap* metadata, Pixel* output_buffer, const BitmapRow* bitmap, uint8_t row,
                            ColourBigEndian foreground_colour) {
    // NOLINTNEXTLINE (cppcoreguidelines-avoid-magic-numbers)
    BitmapRow row_mask = (1U << ((sizeof(BitmapRow) * 8U) - 1U));

    for (uint8_t column = 0; column < metadata->width_px; column++) {
        const Pixel colour = ((bitmap[row] & row_mask) ? (Pixel)foreground_colour : kColourBackground);
        *(output_buffer++) = colour;
        row_mask >>= 1U;
    }
}

/**
 * Uncompress a bitmap with maximum 32 pixels of width
 *
 * @param metadata Bitmap metadata (output buffer, offsets, area width)
 * @param[out] output_buffer Buffer to which uncompress the bitmap
 * @param bitmap Pointer to the metadata bitmap data (row-major, MSB-first)
 * @param row Bitmap row to uncompress
 * @param foreground_colour Colour to use for foreground pixels (1 bits)
 */
static void uncompressRow32(const Bitmap* metadata, Pixel* output_buffer, const uint32_t* bitmap, uint8_t row,
                            ColourBigEndian foreground_colour) {
    // NOLINTNEXTLINE (cppcoreguidelines-avoid-magic-numbers)
    uint32_t row_mask = (1U << ((sizeof(uint32_t) * 8U) - 1U));

    for (uint8_t column = 0; column < metadata->width_px; column++) {
        const Pixel colour = ((bitmap[row] & row_mask) ? (Pixel)foreground_colour : kColourBackground);
        *(output_buffer++) = colour;
        row_mask >>= 1U;
    }
}
