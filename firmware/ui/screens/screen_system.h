/*
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef UI_SCREENS_SCREEN_SYSTEM_H
#define UI_SCREENS_SCREEN_SYSTEM_H
#include <stddef.h>

#include "errorstack.h"
#include "hardware_events.h"

ErrorCode setupSystemScreen(void);
ErrorCode treatSystemScreenMessages(const uint8_t message_flags[kNbEvents]);

#endif
