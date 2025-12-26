/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef UI_UI_H
#define UI_UI_H
#include <stddef.h>
#include <stdint.h>

#include "errorstack.h"
#include "hardware_events.h"

ErrorCode createUItask(void);
uint8_t dispatchEventToUI(Event event);

#endif  // UI_UI_H
