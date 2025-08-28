/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef INC_BUTTONS_H
#define INC_BUTTONS_H
#include "errorstack.h"

/**
 * @brief Enumeration of all the managed buttons
 */
typedef enum { kButtonZero = 0, kButtonHold, kNBbuttons } ButtonType;

ErrorCode runButtonsStateMachine(void);

#endif
