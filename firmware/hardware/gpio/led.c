/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "led.h"

#include <main.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_ll_tim.h>

static inline uint32_t hexToCompareValue(uint8_t hexvalue);
static void applyLEDpwm(Colour colour);

static Colour current_colour = kBlack;    ///< Colour to apply to the blink effect
static uint16_t blink_halfperiod_ms = 0;  ///< Half the milliseconds to operate an on/off blink cycle
static uint32_t current_tick = 0;         ///< Last system tick value saved
static uint8_t led_is_on = 0;             ///< Flag indicating whether the LED is on or off
static EffectState effect = kOFF;         ///< Current LED effect state

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Initialise the RGB LED
 */
void initialiseLED(void) {
    LL_TIM_EnableCounter(TIM2);

    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
}

/**
 * Run the LED state machine
 */
void runLEDstateMachine(void) {
    //only handle blinking for now
    if (effect != kBLINKING) {
        return;
    }

    //if half period not elapsed, exit
    if (!timeout(current_tick, blink_halfperiod_ms)) {
        return;
    }
    current_tick = HAL_GetTick();

    //invert the LED status and power
    applyLEDpwm(led_is_on ? kBlack : current_colour);
    led_is_on = !led_is_on;
}

/**
 * Set the new LED effect
 *
 * @param new_effect New effect to apply
 * @param period_ms Effect period in [ms] (if applicable)
 */
void LEDsetEffect(EffectState new_effect, uint16_t period_ms) {
    //solid black led means it's off
    if ((new_effect == kSOLID) && !led_is_on) {
        new_effect = kOFF;
    }

    //apply the new colour
    Colour colour = kBlack;
    if (new_effect != kOFF) {
        colour = current_colour;
    }
    applyLEDpwm(colour);

    //save the effect and period
    effect = new_effect;
    current_tick = HAL_GetTick();
    blink_halfperiod_ms = (period_ms / 2U);
}

/**
 * Set the new LED colour with a HEX value
 *
 * @param hexa_code New HEX colour value (right justified)
 */
void LEDsetColourHex(const uint32_t hexa_code) {
    const Colour colour = {.hex_value = hexa_code};
    LEDsetColour(&colour);
}

/**
 * Set the new LED colour with a Colour value
 *
 * @param colour New colour
 */
void LEDsetColour(const Colour* colour) {
    if (!colour) {
        return;
    }

    applyLEDpwm(*colour);

    if (colour->hex_value == kBlack.hex_value) {
        led_is_on = 0;
    } else {
        led_is_on = 1;
        current_colour = *colour;
    }
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Get the comparate register value corresponding for a HEX value
 *
 * @param hexvalue HEX value between 0 and 255
 * @return Corresponding compare register value to set to the timer
 */
static inline uint32_t hexToCompareValue(uint8_t hexvalue) {
    if (!hexvalue) {
        return 0;
    }

    //use the Auto Reload Register to compute the proportional Compare Register value to return
    const uint32_t auto_reload_value = (LL_TIM_GetAutoReload(TIM2) + 1U);
    return ((auto_reload_value * hexvalue) / UINT8_MAX) - 1U;
}

/**
 * Apply a colour's set of comparate values to the LED pwm output pins
 */
static void applyLEDpwm(Colour colour) {
    LL_TIM_OC_SetCompareCH1(TIM2, hexToCompareValue(colour.channels.red));
    LL_TIM_OC_SetCompareCH2(TIM2, hexToCompareValue(colour.channels.green));
    LL_TIM_OC_SetCompareCH3(TIM2, hexToCompareValue(colour.channels.blue));
}
