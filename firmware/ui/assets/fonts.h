/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef INC_HARDWARE_SCREEN_FONTS_H
#define INC_HARDWARE_SCREEN_FONTS_H
#include <stddef.h>
#include <stdint.h>

#include "bitmap.h"

/**
 * Character metadata
 */
typedef struct {
    char ascii_character;  ///< ASCII value the bitmap represents
    uint8_t width_px;      ///< Width of the bitmap in [px]
} CharacterMetadata;

/**
 * Font metadata
 */
typedef struct {
    uint8_t height_px;                     ///< Height of each character bitmap in the font
    uint8_t nb_characters;                 ///< Number of bitmaps represented in the font
    const CharacterMetadata* descriptors;  ///< Metadata of all the bitmaps in the font
    const BitmapRow* bitmaps;              ///< Bitmaps of all the characters in the font
} FontMetadata;

uint8_t uncompressCharacter(const FontMetadata* font, uint8_t character_index, ColourBigEndian foreground_colour,
                            Bitmap* bitmap_metadata);
uint8_t getCharIndex(const FontMetadata* font, char ascii_character);

extern const FontMetadata kInterV_7pt_alpha_descriptor;
extern const FontMetadata kInterVSemiBold_14_num_descriptor;

#endif
