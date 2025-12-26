/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @brief Implement the GPIO buttons state and debouncing
 * @author Gilles Henrard
 */
#include "buttons.h"

#include <hardware_events.h>
#include <main.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_ll_gpio.h>

#include "errorstack.h"

_Static_assert((uint8_t)(kNBbuttons <= UINT8_MAX), "The application supports maximum 255 buttons");

enum {
    kDebounceTimeMS = 50U,       ///< Number of milliseconds to wait for debouncing
    kHoldingTimeMS = 1000U,      ///< Number of milliseconds to wait before considering a button is held down
    kEdgeDetectionTimeMS = 40U,  ///< Number of milliseconds during which a falling/rising edge can be detected
};

/**
 * Enumeration of the different button states
 */
typedef enum {
    kButtonReleased = 0,  ///< stateReleased() : Button is released
    kButtonPressed,       ///< statePressed() : Button is pressed, but not held
    kButtonHeld,          ///< stateHeldDown() : Button is held down
} ButtonState;

/**
 * @brief Structure defining a button GPIO
 */
typedef struct {
    GPIO_TypeDef* port;  ///< GPIO port used
    uint32_t pin;        ///< GPIO pin used
    ButtonState state;   ///< Current state of the GPIO button
} Button;

/**
 * @brief Structure holding all the timers used by the buttons
 */
typedef struct {
    uint32_t debouncing_ms;    ///< Timer used for debouncing (in ms)
    uint32_t holding_ms;       ///< Timer used to detect if a button is held down (in ms)
    uint32_t rising_edge_ms;   ///< Timer used to detect a rising edge (in ms)
    uint32_t falling_edge_ms;  ///< Timer used to detect a falling edge (in ms)
} GPIOtimer;

//machine state
static void stateReleased(ButtonType button);
static void statePressed(ButtonType button);
static void stateHeld(ButtonType button);

//state variables
static GPIOtimer buttons_timers[kNBbuttons];  ///< Array of timers used by the buttons

/**
 * @brief Buttons initialisation array
 */
static Button buttons[kNBbuttons] = {
    [kButtonZero] = {.port = ZERO_BUTTON_GPIO_Port, .pin = ZERO_BUTTON_Pin, .state = kButtonReleased},
    [kButtonHold] = {.port = HOLD_BUTTON_GPIO_Port, .pin = HOLD_BUTTON_Pin, .state = kButtonReleased},
};

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Run each button state machine
 *
 * @retval 0 Success
 * @retval 1 Invalid button state reached
 */
ErrorCode runButtonsStateMachine(void) {
    for (uint8_t i = 0; i < (uint8_t)kNBbuttons; i++) {
        switch (buttons[i].state) {
            case kButtonReleased:
                stateReleased(i);
                break;

            case kButtonPressed:
                statePressed(i);
                break;

            case kButtonHeld:
                stateHeld(i);
                break;

            default:
                return createErrorCode(2, 1, kErrorWarning);
                break;
        }
    }

    return kSuccessCode;
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief State in which the button is released
 * 
 * @param button Button for which run the state
 */
static void stateReleased(ButtonType button) {
    const uint32_t current_tick = HAL_GetTick();

    //if button released, restart debouncing timer
    if (LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin)) {
        buttons_timers[button].debouncing_ms = current_tick;
    }

    //if button not pressed for long enough, exit
    if (!timeout(buttons_timers[button].debouncing_ms, kDebounceTimeMS)) {
        return;
    }

    //set the timer during which rising edge can be read, and get to pressed state
    buttons_timers[button].rising_edge_ms = current_tick;
    buttons_timers[button].holding_ms = current_tick;
    buttons[button].state = kButtonPressed;
}

/**
 * @brief State in which the button is pressed, but not yet held
 * 
 * @param button Button for which run the state
 */
static void statePressed(ButtonType button) {
    //if button still pressed, restart debouncing timer
    if (!LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin)) {
        buttons_timers[button].debouncing_ms = HAL_GetTick();

        //if button maintained for long enough, get to held down state
        if (timeout(buttons_timers[button].holding_ms, kHoldingTimeMS)) {
            buttons[button].state = kButtonHeld;

            if (button == kButtonZero) {
                triggerHardwareEvent(kEventCancelZero);
            }
            if (button == kButtonHold) {
                triggerHardwareEvent(kEventToggleScreen);
            }
            return;
        }
    }

    //if button not released for long enough, exit
    if (!timeout(buttons_timers[button].debouncing_ms, kDebounceTimeMS)) {
        return;
    }

    //set the timer during which falling edge can be read, and get to pressed state
    buttons_timers[button].falling_edge_ms = HAL_GetTick();
    buttons[button].state = kButtonReleased;

    //react to specific buttons
    if (button == kButtonZero) {
        triggerHardwareEvent(kEventZero);
    } else if (button == kButtonHold) {
        triggerHardwareEvent(kEventHold);
    }
}

/**
 * @brief State in which the button is held down
 * 
 * @param button Button for which run the state
 */
static void stateHeld(ButtonType button) {
    //if button still pressed, restart debouncing timer
    if (!LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin)) {
        buttons_timers[button].debouncing_ms = HAL_GetTick();
    }

    //if button not released for long enough, exit
    if (!timeout(buttons_timers[button].debouncing_ms, kDebounceTimeMS)) {
        return;
    }

    //set the timer during which falling edge can be read, and get to pressed state
    buttons_timers[button].falling_edge_ms = HAL_GetTick();
    buttons[button].state = kButtonReleased;
}
