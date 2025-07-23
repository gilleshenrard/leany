/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @brief Implement the GPIO buttons state and debouncing
 * @author Gilles Henrard
 * @date 13/06/2025
 */
#include "buttons.h"

#include <FreeRTOS.h>
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_def.h>
#include <stm32f1xx_ll_gpio.h>
#include <task.h>

_Static_assert((bool)(kNBbuttons <= UINT8_MAX), "The application supports maximum 255 buttons");

enum {
    kStackSize = 32U,               ///< Amount of words in the task stack
    kTaskLowPriority = 8U,          ///< FreeRTOS number for a low priority task
    kDebounceTimeMS = 50U,          ///< Number of milliseconds to wait for debouncing
    kHoldingTimeMS = 1000U,         ///< Number of milliseconds to wait before considering a button is held down
    kEdgeDetectionTimeMS = 40U,     ///< Number of milliseconds during which a falling/rising edge can be detected
    kButtonStructAlignment = 128U,  ///< Alignment size used for buttons structure to make its accesses more efficient
    kTimerStructAlignment = 16U,    ///< Alignment size used for timers structure to make its accesses more efficient
    kMutexTimeoutMS = 2U,           ///< Max. number of milliseconds during which a task can attempt to take a mutex
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
    GPIO_TypeDef* port;             ///< GPIO port used
    uint32_t pin;                   ///< GPIO pin used
    ButtonState state;              ///< Current state of the GPIO button
    SemaphoreHandle_t mutex;        ///< handle of the mutex used to protect button updates
    StaticSemaphore_t mutex_state;  ///< mutex state variables
} __attribute__((aligned(kButtonStructAlignment))) Button;

/**
 * @brief Structure holding all the timers used by the buttons
 */
typedef struct {
    uint32_t debouncing_ms;    ///< Timer used for debouncing (in ms)
    uint32_t holding_ms;       ///< Timer used to detect if a button is held down (in ms)
    uint32_t rising_edge_ms;   ///< Timer used to detect a rising edge (in ms)
    uint32_t falling_edge_ms;  ///< Timer used to detect a falling edge (in ms)
} __attribute__((aligned(kTimerStructAlignment))) GPIOtimer;

//utility functions
static void taskButtons(void* argument);

//machine state
static void stateReleased(ButtonType button);
static void statePressed(ButtonType button);

//state variables
static volatile TaskHandle_t task_handle = NULL;  ///< handle of the FreeRTOS task
static GPIOtimer buttons_timers[kNBbuttons];      ///< Array of timers used by the buttons

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
 * @brief Create FreeRTOS static task for the buttons
 */
void createButtonsTask(void) {
    static StackType_t task_stack[kStackSize] = {0};  ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};             ///< Task state variables

    //create the static task
    task_handle = xTaskCreateStatic(taskButtons, "GPIO buttons task", kStackSize, NULL, kTaskLowPriority, task_stack,
                                    &task_state);
    if (!task_handle) {
        Error_Handler();
    }

    //create the buttons' mutex
    for (uint8_t button = 0; button < (uint8_t)kNBbuttons; button++) {
        buttons[button].mutex = xSemaphoreCreateMutexStatic(&buttons[button].mutex_state);
        if (!buttons[button].mutex) {
            Error_Handler();
        }
    }
}

/**
 * @brief Run each button state machine
 */
static void taskButtons(void* argument) {
    UNUSED(argument);

    while (1) {
        for (uint8_t i = 0; i < (uint8_t)kNBbuttons; i++) {
            switch (buttons[i].state) {
                case kButtonReleased:
                    stateReleased(i);
                    break;

                case kButtonPressed:
                case kButtonHeld:
                    statePressed(i);
                    break;

                default:
                    Error_Handler();
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5U));
    }
}

/**
 * @brief Check if a button is released
 * 
 * @param button    Button to check
 * @retval 0        Button is pressed
 * @retval 1        Button is released
 */
uint8_t isButtonReleased(ButtonType button) {
    if (button >= kNBbuttons) {
        return 0;
    }

    uint8_t released = 0;
    if (xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(kMutexTimeoutMS)) == pdTRUE) {
        released = (buttons[button].state == kButtonReleased);
        xSemaphoreGive(buttons[button].mutex);
    }

    return released;
}

/**
 * @brief Check if a button is pressed or held down
 * 
 * @param button    Button to check
 * @retval 0        Button is released
 * @retval 1        Button is pressed
 */
uint8_t isButtonPressed(ButtonType button) {
    if (button >= kNBbuttons) {
        return 0;
    }

    uint8_t pressed = 0;
    if (xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(kMutexTimeoutMS)) == pdTRUE) {
        pressed = ((buttons[button].state == kButtonPressed) || (buttons[button].state == kButtonHeld));
        xSemaphoreGive(buttons[button].mutex);
    }

    return pressed;
}

/**
 * @brief Check if a button is held down
 * 
 * @param button    Button to check
 * @retval 0        Button is released or not yet held down
 * @retval 1        Button is held down
 */
uint8_t isButtonHeldDown(ButtonType button) {
    if (button >= kNBbuttons) {
        return 0;
    }

    uint8_t held = 0;
    if (xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(kMutexTimeoutMS)) == pdTRUE) {
        held = (buttons[button].state == kButtonHeld);
        xSemaphoreGive(buttons[button].mutex);
    }

    return held;
}

/**
 * @brief Check if a button has recently had a rising edge
 * @details Rising edge occurrs when a button goes from released to pressed
 * 
 * @param button    Button to check
 * @retval 0        Button has not had a rising edge
 * @retval 1        Button has had a rising edge
 */
uint8_t buttonHasRisingEdge(ButtonType button) {
    if (button >= kNBbuttons) {
        return 0;
    }

    //check if there has been a rising edge within the last [detection time] ms
    uint8_t tmp = 0;
    if (xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(kMutexTimeoutMS)) == pdTRUE) {
        tmp = !timeout(buttons_timers[button].rising_edge_ms, kEdgeDetectionTimeMS);

        buttons_timers[button].rising_edge_ms = 0;
        xSemaphoreGive(buttons[button].mutex);
    }

    //make sure the software has been up for long enough
    tmp &= timeout(0, kEdgeDetectionTimeMS);
    return (tmp > 0);
}

/**
 * @brief Check if a button has recently had a falling edge
 * @details Rising edge occurrs when a button goes from pressed to released
 * 
 * @param button    Button to check
 * @retval 0        Button has not had a falling edge
 * @retval 1        Button has had a falling edge
 */
uint8_t buttonHasFallingEdge(ButtonType button) {
    if (button >= kNBbuttons) {
        return 0;
    }

    //check if there has been a falling edge within the last [detection time] ms
    uint8_t tmp = 0;
    if (xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(kMutexTimeoutMS)) == pdTRUE) {
        tmp = !timeout(buttons_timers[button].falling_edge_ms, kEdgeDetectionTimeMS);

        buttons_timers[button].falling_edge_ms = 0;
        xSemaphoreGive(buttons[button].mutex);
    }

    //make sure the software has been up for long enough
    tmp &= timeout(0, kEdgeDetectionTimeMS);
    return (tmp > 0);
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief State in which the button is released
 * 
 * @param button Button for which run the state
 */
static void stateReleased(ButtonType button) {
    if (xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(kMutexTimeoutMS)) == pdFALSE) {
        return;
    }

    const uint32_t current_tick = HAL_GetTick();

    //if button released, restart debouncing timer
    if (LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin)) {
        buttons_timers[button].debouncing_ms = current_tick;
    }

    //if button not pressed for long enough, exit
    if (!timeout(buttons_timers[button].debouncing_ms, kDebounceTimeMS)) {
        xSemaphoreGive(buttons[button].mutex);
        return;
    }

    //set the timer during which rising edge can be read, and get to pressed state
    buttons_timers[button].rising_edge_ms = current_tick;
    buttons_timers[button].holding_ms = current_tick;
    buttons[button].state = kButtonPressed;

    xSemaphoreGive(buttons[button].mutex);
}

/**
 * @brief State in which the button is pressed, but not yet held
 * 
 * @param button Button for which run the state
 */
static void statePressed(ButtonType button) {
    if (xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(kMutexTimeoutMS)) == pdFALSE) {
        return;
    }

    //if button still pressed, restart debouncing timer
    if (!LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin)) {
        buttons_timers[button].debouncing_ms = HAL_GetTick();

        //if button maintained for long enough, get to held down state
        if (timeout(buttons_timers[button].holding_ms, kHoldingTimeMS)) {
            buttons[button].state = kButtonHeld;
        }
    }

    //if button not released for long enough, exit
    if (!timeout(buttons_timers[button].debouncing_ms, kDebounceTimeMS)) {
        xSemaphoreGive(buttons[button].mutex);
        return;
    }

    //set the timer during which falling edge can be read, and get to pressed state
    buttons_timers[button].falling_edge_ms = HAL_GetTick();
    buttons[button].state = kButtonReleased;

    xSemaphoreGive(buttons[button].mutex);
}
