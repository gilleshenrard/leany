/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef INC_HARDWARE_SCREEN_ST7735S_INITIALISATION_H
#define INC_HARDWARE_SCREEN_ST7735S_INITIALISATION_H
#include <stdint.h>

#include "st7735_registers.h"

enum {
    kST7735nbCommands = 16U,       ///< Number of commands to send during initialisation
    kST7735structAlignment = 16U,  ///< Configuration command structure memory alignment size
};

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

/**
 * @brief Type associated with a register value
 */
typedef uint8_t Register;

/**
 * @brief Structure describing a command to send during configuration
 */
typedef struct {
    ST7735register register_number;  ///< Number of the register to send
    uint8_t nb_parameters;           ///< Number of parameters sent after the register number
    const Register* parameters;      ///< Array of parameters
} __attribute__((aligned(kST7735structAlignment))) ST7735command;

extern const ST7735command kST7735configurationScript[kST7735nbCommands];
extern const Register kOrientations[kNBorientations];

#endif
