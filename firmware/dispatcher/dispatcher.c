/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file dispatcher.c
 * @brief Dispatches hardware events to UI via a FreeRTOS message queue.
 * 
 * This module listens for sensor or button-triggered hardware events,
 * performs any required processing (like zeroing), and sends formatted
 * messages to the UI queue. It uses a statically allocated task and queue.
 * 
 * @note This task must be created after hardware_events group is initialized.
 *
 * @author Gilles Henrard
 * @date 27/07/2025
 */
#include "dispatcher.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <portmacro.h>
#include <projdefs.h>
#include <queue.h>
#include <stdint.h>
#include <task.h>

#include "errorstack.h"
#include "hardware_events.h"
#include "mems_bmi270.h"
#include "sensorfusion.h"

enum {
    kStackSize = 65U,        ///< Amount of words in the task stack
    kTaskLowPriority = 8U,   ///< FreeRTOS number for a low priority task
    kUImessageDelayMS = 2U,  ///< Number of milliseconds for ui messages delays
    kEventDelayMS = 20U,     ///< Number of milliseconds for hardware events delay
    kUIqueueLength = 50U,    ///< Number of slots available in the UI queue
};

//utility tasks
static void runDispatchertask(void *argument);
static void dispatchEventToUI(MessageID type, int16_t value);

//state variables
static TaskHandle_t task_handle = (void *)0;  ///< handle of the FreeRTOS task
static QueueHandle_t ui_queue = (void *)0;    ///< UI queue
static uint8_t ready = 0;                     ///< Flag indicating whether the queue is ready to be used

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create the messages queue and events dispatcher FreeRTOS task
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
 * @brief Dispatcher FreeRTOS task.
 * @details Waits on hardware events and pushes corresponding messages to the UI queue.
 *
 * @param argument Unused
 */
static void runDispatchertask(void *argument) {
    (void)argument;

    while (1) {
        const EventBits_t events = (EventBits_t)kEventXValue | (EventBits_t)kEventYValue | (EventBits_t)kEventZero |
                                   (EventBits_t)kEventCancelZero;
        EventBits_t events_triggered = waitForHardwareEvents(events, kEventDelayMS);

        //if any angle changed happened, push them in the UI queue
        if ((events_triggered & ((EventBits_t)kEventXValue | (EventBits_t)kEventYValue))) {
            dispatchEventToUI(kMessageXValue, getAngleDegreesTenths(kXaxis));
            dispatchEventToUI(kMessageYValue, getAngleDegreesTenths(kYaxis));
        }

        //react to a zero button press
        if ((events_triggered & (EventBits_t)kEventZero)) {
            bmi270ZeroDown();
            dispatchEventToUI(kMessageZero, 0);
        }

        //react to a zero button holding down
        if ((events_triggered & (EventBits_t)kEventCancelZero)) {
            bmi270CancelZeroing();

            dispatchEventToUI(kMessageCancelZero, 0);
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
    return (xQueueReceive(ui_queue, message_to_get, pdMS_TO_TICKS(kUImessageDelayMS)) == pdPASS);
}

/**
 * Dispatch an event to the UI
 * @details This will push a message to the back of the UI messages queue
 *
 * @param type Message type
 * @param value Value of the message 
 */
static void dispatchEventToUI(const MessageID type, const int16_t value) {
    Message message = (Message){.type = type, .value = value};
    xQueueSend(ui_queue, &message, 0);
}
