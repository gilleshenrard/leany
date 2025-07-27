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
#include <FreeRTOSConfig.h>
#include <portmacro.h>
#include <projdefs.h>
#include <queue.h>
#include <stdint.h>
#include <task.h>

#include "buttons.h"
#include "errorstack.h"
#include "mems_bmi270.h"
#include "sensorfusion.h"

enum {
    kStackSize = 300U,      ///< Amount of words in the task stack
    kTaskLowPriority = 8U,  ///< FreeRTOS number for a low priority task
    kUImessageTimeoutMS = 2U,
    kUIqueueLength = 10U,
};

//utility tasks
static void runDispatchertask(void *argument);

//state variables
static TaskHandle_t task_handle = (void *)0;  ///< handle of the FreeRTOS task
static QueueHandle_t ui_queue = (void *)0;
static uint8_t ready = 0;

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
    static uint8_t ui_queue_buffer[kUIqueueLength * sizeof(Message)];
    static StaticQueue_t ui_queue_state;

    //create the UI messages queue
    ui_queue = xQueueCreateStatic(kUIqueueLength, sizeof(Message), ui_queue_buffer, &ui_queue_state);
    configASSERT(ui_queue);

    // create the static task
    task_handle = xTaskCreateStatic(runDispatchertask, "Dispatch task", kStackSize, (void *)0, kTaskLowPriority,
                                    task_stack, &task_state);
    configASSERT(task_handle);

    ready = 1;
    return (kSuccessCode);
}

/**
 * Run the dispatcher task
 *
 * @param argument Unused
 */
static void runDispatchertask(void *argument) {
    (void)argument;

    Message message;
    while (1) {
        if (anglesChanged()) {
            message = (Message){.type = kMessageXValue, .value = getAngleDegreesTenths(kXaxis)};
            xQueueSend(ui_queue, &message, pdMS_TO_TICKS(kUImessageTimeoutMS));

            message = (Message){.type = kMessageYValue, .value = getAngleDegreesTenths(kYaxis)};
            xQueueSend(ui_queue, &message, pdMS_TO_TICKS(kUImessageTimeoutMS));
        }

        if (buttonHasRisingEdge(kButtonZero)) {
            bmi270ZeroDown();

            message = (Message){.type = kMessageZero, .value = 0};
            xQueueSend(ui_queue, &message, pdMS_TO_TICKS(kUImessageTimeoutMS));
        }

        if (isButtonHeldDown(kButtonZero)) {
            bmi270CancelZeroing();

            message = (Message){.type = kMessageCancelZero, .value = 0};
            xQueueSend(ui_queue, &message, pdMS_TO_TICKS(kUImessageTimeoutMS));
        }
    }
}

/**
 * Get a message from the UI queue
 * @param[out] message_to_get Message buffer to fill with a message
 * @retval 1 A message could be retrieved
 * @retval 0 No message could be retrieved in a timely manner
 */
uint8_t getUImessage(Message *message_to_get) {
    if (!ready) {
        return 0;
    }
    return (xQueueReceive(ui_queue, message_to_get, pdMS_TO_TICKS(kUImessageTimeoutMS)) == pdPASS);
}
