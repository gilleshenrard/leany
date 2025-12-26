/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "label.h"

#include <stddef.h>
#include <stdint.h>

#include "bitmap.h"
#include "errorstack.h"
#include "fonts.h"

enum {
    kStringMaxLength = 20U,  ///< Maximum number of characters
};

static ErrorCode uncompressStringBitmaps(const uint8_t indexes[kStringMaxLength], size_t string_length,
                                         const FontMetadata* font, ColourBigEndian foreground_colour,
                                         const Bitmap* label_metadata);
static uint8_t getStringXoffset(const Label* label, uint8_t string_width_px);
static void wipeLabelLeftRight(const Label* label, Pixel* buffer, uint8_t x_offset_in_label_px,
                               uint8_t string_width_px);

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Uncompress a label in the buffer
 *
 * @param label Label to uncompress
 * @param[out] buffer Buffer into which decompress the label
 * @param string String to display in the buffer
 * @param string_length Number of characters in the string
 * @param foreground_colour Foreground colour of the label
 * @retval 0 Success
 * @retval 1 NULL pointer provided
 * @return Bitmap uncompressing return value
 */
ErrorCode uncompressLabel(const Label* label, Pixel* buffer, const char string[], uint8_t string_length,
                          ColourBigEndian foreground_colour) {
    if (!label || !buffer || !string) {
        return createErrorCode(1, 1, kErrorError);
    }

    if (!string_length) {
        return kSuccessCode;
    }

    if (string_length > kStringMaxLength) {
        string_length = kStringMaxLength;
    }

    //compute the whole string width in [px]
    uint8_t char_indexes[kStringMaxLength];
    uint8_t string_width_px = 0;
    for (uint8_t character = 0; character < string_length; character++) {
        const uint8_t index = getCharIndex(label->font, string[character]);
        char_indexes[character] = index;
        string_width_px += label->font->descriptors[index].width_px;
    }
    if (string_width_px > label->width_px) {
        return createErrorCode(1, 2, kErrorError);
    }

    //compute string horizontal alignment offset within the label
    const uint8_t x_offset_in_label_px = getStringXoffset(label, string_width_px);

    //fill out the label buffer metadata
    Bitmap label_metadata = {
        .output_buffer = buffer,
        .x_offset_px = x_offset_in_label_px,
        .container_width_px = label->width_px,
    };

    //wipe the left and right unused label parts
    wipeLabelLeftRight(label, buffer, x_offset_in_label_px, string_width_px);

    //uncompress the final string
    ErrorCode result =
        uncompressStringBitmaps(char_indexes, string_length, label->font, foreground_colour, &label_metadata);
    EXIT_ON_ERROR(result, 1, kErrorError);

    return kSuccessCode;
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Uncompress the bitmaps of all the characters in a pixel buffer
 *
 * @param indexes Array of characters bitmap pool indexes
 * @param string_length Number of the characters in the string
 * @param font Font to use
 * @param foreground_colour Foreground to apply to the string
 * @param bitmap_metadata 
 * @retval 0 Success
 * @retval 1 NULL pointer provided
 * @retval 2 Character decompression returned 0 px width
 */
static ErrorCode uncompressStringBitmaps(const uint8_t indexes[kStringMaxLength], size_t string_length,
                                         const FontMetadata* font, ColourBigEndian foreground_colour,
                                         const Bitmap* label_metadata) {
    if (!indexes || !font || !label_metadata) {
        return createErrorCode(2, 1, kErrorError);
    }

    if (!label_metadata->container_width_px) {
        return kSuccessCode;
    }

    Bitmap offset_metadata = *label_metadata;
    for (size_t character = 0; character < string_length; character++) {
        const uint8_t char_index = indexes[character];
        const uint8_t char_width_px = uncompressCharacter(font, char_index, foreground_colour, &offset_metadata);
        if (char_width_px == 0U) {
            return createErrorCode(2, 2, kErrorError);
        }
        offset_metadata.x_offset_px += char_width_px;
    }

    return kSuccessCode;
}

/**
 * Get the X offset of a string within a label, depending on its alignment
 *
 * @param label Label containing the string
 * @param string_width_px Total width of the string's combined charcter bitmaps in [px]
 * @return String horizontal offset in [px]
 */
static uint8_t getStringXoffset(const Label* label, uint8_t string_width_px) {
    if (!label) {
        return 0;
    }

    if (label->width_px < string_width_px) {
        return 0;
    }

    switch (label->alignment) {
        case kAlignmentRight:
            return (uint8_t)((label->width_px - string_width_px) - 1U);
            break;

        case kAlignmentLeft:
            return 0;
            break;

        case kAlignmentCentered:
        default:
            return (uint8_t)((label->width_px - string_width_px) / 2U);
            break;
    }
}

/**
 * Wipe the parts of a label not unoccupied by a string
 *
 * @param label Label to clean up
 * @param buffer Buffer into which the label is uncompressed
 * @param x_offset_in_label_px Offset of the string in the label in [px]
 * @param string_width_px Total width of the string in [px]
 */
static void wipeLabelLeftRight(const Label* label, Pixel* buffer, uint8_t x_offset_in_label_px,
                               uint8_t string_width_px) {
    const uint8_t right_empty_width_px = (uint8_t)(label->width_px - (x_offset_in_label_px + string_width_px));
    for (uint8_t row = 0; row < label->font->height_px; row++) {
        Pixel* label_row = &buffer[row * label->width_px];

        for (uint8_t column = 0; column < (x_offset_in_label_px + 1U); column++) {
            label_row[column] = kColourBackground;
        }

        for (uint8_t column = 0; column < right_empty_width_px; column++) {
            label_row[label->width_px - column - 1U] = kColourBackground;
        }
    }
}
