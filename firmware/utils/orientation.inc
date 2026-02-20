/*
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef INC_HARDWARE_DISPLAY_ORIENTATION_H
#define INC_HARDWARE_DISPLAY_ORIENTATION_H

/**
 * @brief Enumeration of the available display kOrientations
 */
typedef enum {
    kPortrait = 0,   ///< Portrait
    kPortrait180,    ///< Portrait, rotated 180°
    kLandscape,      ///< Landscape
    kLandscape180,   ///< Landscape, rotated 180°
    kNBorientations  ///< Number of available kOrientations
} Orientation;

#endif
