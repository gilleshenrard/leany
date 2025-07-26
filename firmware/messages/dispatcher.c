/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file ui.c
 * @brief Implement the messages queues dispatcher
 * @author Gilles Henrard
 * @date 26/07/2025
 */
#include "dispatcher.h"

#include <FreeRTOS.h>
#include <main.h>
#include <stm32f1xx_hal_def.h>
#include <task.h>

#include "errorstack.h"

enum {
    kStackSize = 300U,      ///< Amount of words in the task stack
    kTaskLowPriority = 8U,  ///< FreeRTOS number for a low priority task
};

//utility tasks
static void runDispatchertask(void *argument);

//state variables
static volatile TaskHandle_t task_handle = NULL;  ///< handle of the FreeRTOS task

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create the messages queue dispatcher FreeRTOS task
 *
 * @return Success
 */
ErrorCode createMessageDispatchertask(void) {
    static StackType_t task_stack[kStackSize] = {0};  ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};             ///< Task state variables}

    // create the static task
    task_handle = xTaskCreateStatic(runDispatchertask, "Dispatch task", kStackSize, NULL, kTaskLowPriority, task_stack,
                                    &task_state);
    if (!task_handle) {
        Error_Handler();
    }

    return (kSuccessCode);
}

/**
 * Run the dispatcher task
 *
 * @param argument Unused
 */
static void runDispatchertask(void *argument) {
    UNUSED(argument);

    while (1) {
    }
}
