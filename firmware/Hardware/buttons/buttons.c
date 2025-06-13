/**
 * @brief Implement the GPIO buttons state and debouncing
 * @author Gilles Henrard
 * @date 13/06/2025
 */
#include "buttons.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "main.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_ll_gpio.h"
#include "task.h"

_Static_assert((bool)(NB_BUTTONS <= UINT8_MAX), "The application supports maximum 255 buttons");

enum {
    STACK_SIZE              = 32U,    ///< Amount of words in the task stack
    TASK_LOW_PRIORITY       = 8U,     ///< FreeRTOS number for a low priority task
    DEBOUNCE_TIME_MS        = 50U,    ///< Number of milliseconds to wait for debouncing
    HOLDING_TIME_MS         = 1000U,  ///< Number of milliseconds to wait before considering a button is held down
    EDGEDETECTION_TIME_MS   = 40U,    ///< Number of milliseconds during which a falling/rising edge can be detected
    BUTTON_STRUCT_ALIGNMENT = 128U,   ///< Alignment size used for buttons structure to make its accesses more efficient
    TIMERS_STRUCT_ALIGNMENT = 16U,    ///< Alignment size used for timers structure to make its accesses more efficient
    MUTEX_TIMEOUT_MS        = 2U,     ///< Max. number of milliseconds during which a task can attempt to take a mutex
};

/**
 * Enumeration of the different button states
 */
typedef enum {
    BUT_RELEASED = 0,  ///< stateReleased() : Button is released
    BUT_PRESSED,       ///< statePressed() : Button is pressed, but not held
    BUT_HELD,          ///< stateHeldDown() : Button is held down
} buttonstate_e;

/**
 * @brief Structure defining a button GPIO
 */
typedef struct {
    GPIO_TypeDef*     port;        ///< GPIO port used
    uint32_t          pin;         ///< GPIO pin used
    buttonstate_e     state;       ///< Current state of the GPIO button
    SemaphoreHandle_t mutex;       ///< handle of the mutex used to protect button updates
    StaticSemaphore_t mutexState;  ///< mutex state variables
} __attribute__((aligned(BUTTON_STRUCT_ALIGNMENT))) button_t;

/**
 * @brief Structure holding all the timers used by the buttons
 */
typedef struct {
    uint32_t debouncing_ms;   ///< Timer used for debouncing (in ms)
    uint32_t holding_ms;      ///< Timer used to detect if a button is held down (in ms)
    uint32_t risingEdge_ms;   ///< Timer used to detect a rising edge (in ms)
    uint32_t fallingEdge_ms;  ///< Timer used to detect a falling edge (in ms)
} __attribute__((aligned(TIMERS_STRUCT_ALIGNMENT))) gpioTimer_t;

//utility functions
static void taskButtons(void* argument);

//machine state
static void stateReleased(button_e button);
static void statePressed(button_e button);

//state variables
static volatile TaskHandle_t taskHandle = NULL;          ///< handle of the FreeRTOS task
static gpioTimer_t           buttonsTimers[NB_BUTTONS];  ///< Array of timers used by the buttons

/**
 * @brief Buttons initialisation array
 */
static button_t buttons[NB_BUTTONS] = {
    [BTN_ZERO] = {.port = ZERO_BUTTON_GPIO_Port, .pin = ZERO_BUTTON_Pin, .state = BUT_RELEASED},
    [BTN_HOLD] = {.port = HOLD_BUTTON_GPIO_Port, .pin = HOLD_BUTTON_Pin, .state = BUT_RELEASED},
};

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create FreeRTOS static task for the buttons
 */
void createButtonsTask(void) {
    static StackType_t  taskStack[STACK_SIZE] = {0};  ///< Buffer used as the task stack
    static StaticTask_t taskState             = {0};  ///< Task state variables

    //create the static task
    taskHandle =
        xTaskCreateStatic(taskButtons, "GPIO buttons task", STACK_SIZE, NULL, TASK_LOW_PRIORITY, taskStack, &taskState);
    if(!taskHandle) {
        Error_Handler();
    }

    //create the buttons' mutex
    for(uint8_t button = 0; button < (uint8_t)NB_BUTTONS; button++) {
        buttons[button].mutex = xSemaphoreCreateMutexStatic(&buttons[button].mutexState);
        if(!buttons[button].mutex) {
            Error_Handler();
        }
    }
}

/**
 * @brief Run each button state machine
 */
static void taskButtons(void* argument) {
    UNUSED(argument);

    while(1) {
        for(uint8_t i = 0; i < (uint8_t)NB_BUTTONS; i++) {
            switch(buttons[i].state) {
                case BUT_RELEASED:
                    stateReleased(i);
                    break;

                case BUT_PRESSED:
                case BUT_HELD:
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
uint8_t isButtonReleased(button_e button) {
    if(button >= NB_BUTTONS) {
        return 0;
    }

    uint8_t released = 0;
    if(xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        released = (buttons[button].state == BUT_RELEASED);
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
uint8_t isButtonPressed(button_e button) {
    if(button >= NB_BUTTONS) {
        return 0;
    }

    uint8_t pressed = 0;
    if(xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        pressed = ((buttons[button].state == BUT_PRESSED) || (buttons[button].state == BUT_HELD));
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
uint8_t isButtonHeldDown(button_e button) {
    if(button >= NB_BUTTONS) {
        return 0;
    }

    uint8_t held = 0;
    if(xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        held = (buttons[button].state == BUT_HELD);
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
uint8_t buttonHasRisingEdge(button_e button) {
    if(button >= NB_BUTTONS) {
        return 0;
    }

    //check if there has been a rising edge within the last [detection time] ms
    uint8_t tmp = 0;
    if(xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        tmp = !timeout(buttonsTimers[button].risingEdge_ms, EDGEDETECTION_TIME_MS);

        buttonsTimers[button].risingEdge_ms = 0;
        xSemaphoreGive(buttons[button].mutex);
    }

    //make sure the software has been up for long enough
    tmp &= timeout(0, EDGEDETECTION_TIME_MS);
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
uint8_t buttonHasFallingEdge(button_e button) {
    if(button >= NB_BUTTONS) {
        return 0;
    }

    //check if there has been a falling edge within the last [detection time] ms
    uint8_t tmp = 0;
    if(xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        tmp = !timeout(buttonsTimers[button].fallingEdge_ms, EDGEDETECTION_TIME_MS);

        buttonsTimers[button].fallingEdge_ms = 0;
        xSemaphoreGive(buttons[button].mutex);
    }

    //make sure the software has been up for long enough
    tmp &= timeout(0, EDGEDETECTION_TIME_MS);
    return (tmp > 0);
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief State in which the button is released
 * 
 * @param button Button for which run the state
 */
static void stateReleased(button_e button) {
    if(xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdFALSE) {
        return;
    }

    const uint32_t currentTick = HAL_GetTick();

    //if button released, restart debouncing timer
    if(LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin)) {
        buttonsTimers[button].debouncing_ms = currentTick;
    }

    //if button not pressed for long enough, exit
    if(!timeout(buttonsTimers[button].debouncing_ms, DEBOUNCE_TIME_MS)) {
        xSemaphoreGive(buttons[button].mutex);
        return;
    }

    //set the timer during which rising edge can be read, and get to pressed state
    buttonsTimers[button].risingEdge_ms = currentTick;
    buttonsTimers[button].holding_ms    = currentTick;
    buttons[button].state               = BUT_PRESSED;

    xSemaphoreGive(buttons[button].mutex);
}

/**
 * @brief State in which the button is pressed, but not yet held
 * 
 * @param button Button for which run the state
 */
static void statePressed(button_e button) {
    if(xSemaphoreTake(buttons[button].mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdFALSE) {
        return;
    }

    //if button still pressed, restart debouncing timer
    if(!LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin)) {
        buttonsTimers[button].debouncing_ms = HAL_GetTick();

        //if button maintained for long enough, get to held down state
        if(timeout(buttonsTimers[button].holding_ms, HOLDING_TIME_MS)) {
            buttons[button].state = BUT_HELD;
        }
    }

    //if button not released for long enough, exit
    if(!timeout(buttonsTimers[button].debouncing_ms, DEBOUNCE_TIME_MS)) {
        xSemaphoreGive(buttons[button].mutex);
        return;
    }

    //set the timer during which falling edge can be read, and get to pressed state
    buttonsTimers[button].fallingEdge_ms = HAL_GetTick();
    buttons[button].state                = BUT_RELEASED;

    xSemaphoreGive(buttons[button].mutex);
}
