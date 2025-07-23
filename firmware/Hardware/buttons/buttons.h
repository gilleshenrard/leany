/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef INC_BUTTONS_H
#define INC_BUTTONS_H
#include <stdint.h>

/**
 * @brief Enumeration of all the managed buttons
 */
typedef enum { kButtonZero = 0, kButtonHold, kNBbuttons } ButtonType;

void createButtonsTask(void);
uint8_t isButtonReleased(ButtonType button);
uint8_t isButtonPressed(ButtonType button);
uint8_t isButtonHeldDown(ButtonType button);
uint8_t buttonHasRisingEdge(ButtonType button);
uint8_t buttonHasFallingEdge(ButtonType button);

#endif
