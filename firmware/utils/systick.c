/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file systick.c
 * @author Gilles Henrard
 */
// cppcheck-suppress-file unusedFunction
#include "systick.h"

#include <FreeRTOS.h>  // NOLINT(misc-include-cleaner,-warnings-as-errors)
#include <portmacro.h>
#include <projdefs.h>
#include <stdint.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_ll_rcc.h>
#include <task.h>

_Static_assert((sizeof(TickType_t) == sizeof(uint32_t)), "Ticktype_t mismaches uint32_t size");

/**
 * Get the current system tick
 *
 * @return System tick
 */
uint32_t getCurrentTick(void) { return (uint32_t)xTaskGetTickCount(); }

/**
 * Get the current system tick, in an ISR-safe way
 *
 * @return System tick
 */
uint32_t getCurrentTickISR(void) { return (uint32_t)xTaskGetTickCountFromISR(); }

/**
 * Check if a timeout in [ms] has occurred
 *
 * @param start_tick The tick to compare to now to check for a timeout
 * @param timeout_ms The timeout span in milliseconds
 * @retval true Timeout has occurred
 * @retval false Timeout has not occurred
 */
bool systickTimeout(uint32_t start_tick, uint32_t timeout_ms) {
    return ((getCurrentTick() - start_tick) >= pdMS_TO_TICKS(timeout_ms));
}

/**
 * @brief  Prime the HSE oscillator ahead of SystemClock_Config()
 *
 * @details On cold power-on, a quartz crystal requires time to build up
 *          sufficient oscillation amplitude before the STM32 clock controller
 *          asserts the HSE ready flag. SystemClock_Config() waits on that flag
 *          with no timeout, so a crystal that is still settling will cause the
 *          MCU to spin indefinitely — leaving all peripherals uninitialised.
 *
 *          This function activates the oscillator amplifier early, while the
 *          MCU is still running on HSI, and waits long enough for the crystal
 *          to stabilise. By the time SystemClock_Config() checks
 *          LL_RCC_HSE_IsReady(), the flag is already set.
 *
 * @note   This function is to be called between HAL_Init() and SystemClock_Config()
 *
 * @note   LL_RCC_HSE_Enable() is idempotent: SET_BIT on an already-set bit is
 *         a hardware no-op, so the redundant call inside SystemClock_Config()
 *         is harmless.
 * @note   HAL_Delay() is safe here because HAL_Init() has already configured
 *         the SysTick on the internal HSI clock.
 */
void primeHSEOscillator(void) {
    const uint32_t startup_delay_ms = 100U;

    LL_RCC_HSE_Enable();
    HAL_Delay(startup_delay_ms);
}
