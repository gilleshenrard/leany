/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file systick.h
 * @author Gilles Henrard
 */
#ifndef UTILS_SYSTICK_H
#define UTILS_SYSTICK_H

#include <stdint.h>

uint32_t getCurrentTick(void);
uint8_t systickTimeout(uint32_t start_tick, uint32_t timeout_ms);

#endif
