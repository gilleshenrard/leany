/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file debugpin.h
 * @author Gilles Henrard
 */
#ifndef HARDWARE_SYSUTILS_DEBUGPIN
#define HARDWARE_SYSUTILS_DEBUGPIN
#include <main.h>

/**
 * Set the debug pin to high
 */
static inline void setDebugPin(void) { LL_GPIO_SetOutputPin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin); }

/**
 * Set the debug pin to low
 */
static inline void resetDebugPin(void) { LL_GPIO_ResetOutputPin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin); }

#endif
