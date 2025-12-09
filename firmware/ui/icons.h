/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef INC_HARDWARE_SCREEN_ICONS_H
#define INC_HARDWARE_SCREEN_ICONS_H
#include <stdint.h>

enum {
    kVerdanaNbRows = 37U,     ///< Number of rows of pixel each Verdana character has
    kVerdanaNbColumns = 30U,  ///< Number of columns of pixel each Verdana character has
};

typedef enum {
    kBrightGrayBigEndian = 0x7DEFU,    ///< 0xEEEEEE in RGB565 with MSB and LSB switched
    kDarkCharcoalBigEndian = 0xA631U,  ///< 0x333333 in RGB565 with MSB and LSB switched
} ColourBigEndian;

typedef enum {
    kVerdana0 = 0,         ///< Character '0' in Verdana font
    kVerdana1,             ///< Character '1' in Verdana font
    kVerdana2,             ///< Character '2' in Verdana font
    kVerdana3,             ///< Character '3' in Verdana font
    kVerdana4,             ///< Character '4' in Verdana font
    kVerdana5,             ///< Character '5' in Verdana font
    kVerdana6,             ///< Character '6' in Verdana font
    kVerdana7,             ///< Character '7' in Verdana font
    kVerdana8,             ///< Character '8' in Verdana font
    kVerdana9,             ///< Character '9' in Verdana font
    kVerdanaPlus,          ///< Character '+' in Verdana font
    kVerdanaMinus,         ///< Character '-' in Verdana font
    kVerdanaDot,           ///< Character '.' in Verdana font
    kNbVerdataCharacters,  ///< Number of characters implemented in Verdana
} VerdanaCharacter;

/**
 * @brief Pixel type definition
 */
typedef uint16_t Pixel;

void uncompressCharacter(Pixel buffer[], VerdanaCharacter character);

#endif
