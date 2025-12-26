/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef INC_HARDWARE_SCREEN_ST7735S_INITIALISATION_H
#define INC_HARDWARE_SCREEN_ST7735S_INITIALISATION_H
#include <stdint.h>

#include "orientation.inc"
#include "st7735_registers.inc"

enum {
    kST7735nbCommands = 16U,  ///< Number of commands to send during initialisation
};

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
} ST7735command;

extern const ST7735command kST7735configurationScript[kST7735nbCommands];
extern const Register kOrientations[kNBorientations];

#endif
