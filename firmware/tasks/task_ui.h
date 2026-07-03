/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file task_ui.h
 * @author Gilles Henrard
 */
#ifndef UI_UI_H
#define UI_UI_H
#include <stddef.h>

#include "errorstack.h"
#include "hardware_events.h"

ErrorCode createUItask(void);
bool dispatchEventToUI(Event event);

#endif  // UI_UI_H
