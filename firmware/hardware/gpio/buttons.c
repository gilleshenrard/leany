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

enum {
    kDebounceTimeMS = 50U,       ///< Number of milliseconds to wait for debouncing
    kHoldingTimeMS = 1000U,      ///< Number of milliseconds to wait before considering a button is held down
    kEdgeDetectionTimeMS = 40U,  ///< Number of milliseconds during which a button state change can be detected
};

/**
 * @brief Enumeration of all the managed buttons
 */
typedef enum { kButtonZero = 0, kButtonHold, kNBbuttons } ButtonType;
_Static_assert((kNBbuttons <= UINT8_MAX), "The application supports maximum 255 buttons");

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
    GPIO_TypeDef* port;            ///< GPIO port used
    uint32_t pin;                  ///< GPIO pin used
    ButtonState state;             ///< Current state of the GPIO button
    uint32_t start_debounce_tick;  ///< Tick at which last debouncing started (in ticks)
    uint32_t start_hold_tick;      ///< Tick at which last holdind down started (in ticks)
    uint32_t detect_click_tick;    ///< Tick at which last click detection started (in ticks)
    uint32_t detect_hold_tick;     ///< Tick at which last holding down detection started (in ticks)
} Button;

//machine state
static void reactToButtons(void);
static void stateReleased(ButtonType button);
static void statePressed(ButtonType button);
static void stateHeld(ButtonType button);
static uint8_t consumeClickEvent(ButtonType button);
static uint8_t consumeHoldEvent(ButtonType button);

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
                stateReleased((ButtonType)i);
                break;

            case kButtonPressed:
                statePressed((ButtonType)i);
                break;

            case kButtonHeld:
                stateHeld((ButtonType)i);
                break;

            default:
                return createErrorCode(2, 1, kErrorWarning);
        }
    }

    reactToButtons();
    return kSuccessCode;
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * React to specific buttons' change of state
 */
static void reactToButtons(void) {
    if (consumeClickEvent(kButtonZero)) {
        triggerHardwareEvent(kEventZero);
    }

    if (consumeClickEvent(kButtonHold)) {
        triggerHardwareEvent(kEventHold);
    }

    if (consumeHoldEvent(kButtonZero)) {
        triggerHardwareEvent(kEventCancelZero);
    }

    if (consumeHoldEvent(kButtonHold)) {
        triggerHardwareEvent(kEventToggleScreen);
    }
}

/**
 * @brief State in which the button is released
 * 
 * @param button Button for which run the state
 */
static void stateReleased(ButtonType button) {
    const uint32_t current_tick = HAL_GetTick();

    //if button released, restart debouncing timer
    if (LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin)) {
        buttons[button].start_debounce_tick = current_tick;
    }

    //if button not pressed for long enough, exit
    if (!timeout(buttons[button].start_debounce_tick, kDebounceTimeMS)) {
        return;
    }

    //set the timer during which hold can be detected, and get to pressed state
    buttons[button].start_hold_tick = current_tick;
    buttons[button].state = kButtonPressed;
}

/**
 * @brief State in which the button is pressed, but not yet held
 * 
 * @param button Button for which run the state
 */
static void statePressed(ButtonType button) {
    const uint32_t current_tick = HAL_GetTick();

    //if button still pressed, restart debouncing timer
    if (!LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin)) {
        buttons[button].start_debounce_tick = current_tick;

        //if button maintained for long enough, get to held down state
        if (timeout(buttons[button].start_hold_tick, kHoldingTimeMS)) {
            buttons[button].state = kButtonHeld;
            buttons[button].detect_hold_tick = current_tick;
            return;
        }
    }

    //if button not released for long enough, exit
    if (!timeout(buttons[button].start_debounce_tick, kDebounceTimeMS)) {
        return;
    }

    //set the timer during which falling edge can be read, and get to released state
    buttons[button].detect_click_tick = current_tick;
    buttons[button].state = kButtonReleased;
}

/**
 * @brief State in which the button is held down
 * 
 * @param button Button for which run the state
 */
static void stateHeld(ButtonType button) {
    const uint32_t current_tick = HAL_GetTick();

    //if button still pressed, restart debouncing timer
    if (!LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin)) {
        buttons[button].start_debounce_tick = current_tick;
    }

    //if button not released for long enough, exit
    if (!timeout(buttons[button].start_debounce_tick, kDebounceTimeMS)) {
        return;
    }

    //set the timer during which falling edge can be read, and get to released state
    buttons[button].state = kButtonReleased;
}

/**
 * Check if a button had a click event recently
 * @note This will reset the click detection timer
 *
 * @param button Button to check
 * @retval 1 Click detected
 * @retval 0 No click detected
 */
static uint8_t consumeClickEvent(ButtonType button) {
    //avoid a false detection at boot
    if (HAL_GetTick() <= kEdgeDetectionTimeMS) {
        return 0;
    }

    const uint8_t clicked = ((buttons[button].state == kButtonReleased) &&
                             !timeout(buttons[button].detect_click_tick, kEdgeDetectionTimeMS));

    //reset the timer so the same event cannot trigger a positive more than once
    buttons[button].detect_click_tick = 0;
    return clicked;
}

/**
 * Check if a button had a hold event recently
 * @note This will reset the hold detection timer
 *
 * @param button Button to check
 * @retval 1 Hold detected
 * @retval 0 No hold detected
 */
static uint8_t consumeHoldEvent(ButtonType button) {
    //avoid a false detection at boot
    if (HAL_GetTick() <= kEdgeDetectionTimeMS) {
        return 0;
    }

    const uint8_t held =
        ((buttons[button].state == kButtonHeld) && !timeout(buttons[button].detect_hold_tick, kEdgeDetectionTimeMS));

    //reset the timer so the same event cannot trigger a positive more than once
    buttons[button].detect_hold_tick = 0;
    return held;
}
