/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef INC_HARDWARE_SCREEN_ICONS_H
#define INC_HARDWARE_SCREEN_ICONS_H
#include <stdint.h>

enum {
    kVerdanaNbRows = 37U,
    kVerdanaNbColumns = 30U,
};

typedef enum {
    kBrightGrayBigEndian = 0x7DEFU,    ///< #EEEEEE in RGB565 with MSB and LSB switched
    kDarkCharcoalBigEndian = 0xA631U,  ///< #333333 in RGB565 with MSB and LSB switched
} ColourBigEndian;

typedef enum {
    kVerdana0 = 0,
    kVerdana1,
    kVerdana2,
    kVerdana3,
    kVerdana4,
    kVerdana5,
    kVerdana6,
    kVerdana7,
    kVerdana8,
    kVerdana9,
    kVerdanaPlus,
    kVerdanaMinus,
    kVerdanaDot,
    kNbVerdataCharacters,
} VerdanaCharacter;

/**
 * @brief Pixel type definition
 */
typedef uint16_t Pixel;

void uncompressCharacter(Pixel buffer[], VerdanaCharacter character);

#endif
