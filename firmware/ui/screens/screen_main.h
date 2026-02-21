/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file screen_main.h
 * @author Gilles Henrard
 */
#ifndef UI_SCREEN_MAIN_H
#define UI_SCREEN_MAIN_H
#include <stdint.h>

#include "errorstack.h"
#include "hardware_events.h"
#include "orientation.inc"

ErrorCode setupMainScreen(void);
ErrorCode treatMainScreenMessages(uint8_t message_flags[kNbEvents]);
ErrorCode changeLayoutOrientation(Orientation new_orientation);

#endif
