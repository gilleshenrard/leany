/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @brief Implement the FreeRTOS task taking care of simple GPIO modules
 * @author Gilles Henrard
 */
#include "task_gpio.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <stddef.h>
#include <stm32f1xx_hal_def.h>
#include <task.h>

#include "buttons.h"
#include "errorstack.h"
#include "hal_adc.h"
#include "led.h"

enum {
    kStackSize = 150U,      ///< Amount of words in the task stack
    kTaskLowPriority = 8U,  ///< FreeRTOS number for a low priority task
};

//task functions
static void taskGPIO(void* argument);

//state variables
static volatile TaskHandle_t task_handle = NULL;  ///< handle of the FreeRTOS task
static ErrorCode result;

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create FreeRTOS static task for the buttons
 */
void createGPIOtask(void) {
    static StackType_t task_stack[kStackSize] = {0};  ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};             ///< Task state variables

    //create the static task
    task_handle = xTaskCreateStatic(taskGPIO, "GPIO task", kStackSize, NULL, kTaskLowPriority, task_stack, &task_state);
    configASSERT(task_handle)
}

/**
 * @brief Run the GPIO state machine
 */
static void taskGPIO(void* argument) {
    UNUSED(argument);

    initialiseLED();
    initialiseUserADC();

    while (1) {
        result = runButtonsStateMachine();
        if (isError(result)) {
            Error_Handler();
        }

        runLEDstateMachine();
        runUserADCstateMachine();

        vTaskDelay(pdMS_TO_TICKS(5U));
    }
}
