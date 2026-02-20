/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file systick.c
 * @author Gilles Henrard
 */
#include "systick.h"

#include <FreeRTOS.h>  // NOLINT(misc-include-cleaner,-warnings-as-errors)
#include <portmacro.h>
#include <projdefs.h>
#include <stdint.h>
#include <task.h>

_Static_assert((sizeof(TickType_t) == sizeof(uint32_t)), "Ticktype_t mismaches uint32_t size");

/**
 * Get the current system tick
 *
 * @return System tick
 */
uint32_t getCurrentTick(void) { return (uint32_t)xTaskGetTickCount(); }

/**
 * Check if a timeout in [ms] has occurred
 *
 * @param start_tick The tick to compare to now to check for a timeout
 * @param timeout_ms The timeout span in milliseconds
 * @retval 1 Timeout has occurred
 * @retval 0 Timeout has not occurred
 */
uint8_t systickTimeout(uint32_t start_tick, uint32_t timeout_ms) {
    return ((getCurrentTick() - start_tick) >= pdMS_TO_TICKS(timeout_ms));
}
