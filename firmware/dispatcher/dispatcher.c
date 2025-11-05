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
 * @date 30/09/2025
 */
#include "dispatcher.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <message_buffer.h>
#include <portmacro.h>
#include <projdefs.h>
#include <stdint.h>
#include <task.h>

#include "errorstack.h"
#include "generic_command.inc"
#include "hardware_events.h"
#include "imu.h"
#include "led.h"
#include "sensorfusion.h"
#include "serial.h"

enum {
    kStackSize = 150U,       ///< Amount of words in the task stack
    kTaskLowPriority = 8U,   ///< FreeRTOS number for a low priority task
    kUImessageDelayMS = 2U,  ///< Number of milliseconds for ui messages delays
    kEventDelayMS = 20U,     ///< Number of milliseconds for hardware events delay
    kUIqueueLength = 50U,    ///< Number of slots available in the UI queue
};

//utility tasks
static void runDispatchertask(void* argument);
static uint8_t dispatchEventToUI(MessageID type, int16_t value);
static void handleAngleChangeEvent(void);
static void handleZeroingEvent(void);
static void handleZeroingCancelEvent(void);
static void handleHoldingEvent(uint8_t* holding);
static void handleSerialCommandEvent(void);

//state variables
static TaskHandle_t task_handle = (void*)0;         ///< handle of the FreeRTOS task
static MessageBufferHandle_t ui_buffer = (void*)0;  ///< UI message buffer
static uint8_t ready = 0;                           ///< Flag indicating whether the queue is ready to be used

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create the messages queue and events dispatcher FreeRTOS task
 *
 * @return Success
 */
ErrorCode createMessageDispatchertask(void) {
    static StackType_t task_stack[kStackSize] = {0};                     // Buffer used as the task stack
    static StaticTask_t task_state = {0};                                // Task state variables
    static uint8_t ui_message_buffer[kUIqueueLength * sizeof(Message)];  // Message buffer to use with UI messages
    static StaticStreamBuffer_t ui_buffer_state;                         // UI message buffer state variables

    //create the UI message buffer
    ui_buffer = xMessageBufferCreateStatic(kUIqueueLength * sizeof(Message), ui_message_buffer, &ui_buffer_state);
    configASSERT(ui_buffer);

    // create the static task
    task_handle = xTaskCreateStatic(runDispatchertask, "Dispatch task", kStackSize, (void*)0, kTaskLowPriority,
                                    task_stack, &task_state);
    configASSERT(task_handle);

    ready = 1;
    return (kSuccessCode);
}

/**
 * Get a message from the UI queue
 * @param[out] message_to_get Message buffer to fill with a message
 * @retval 1 A message could be retrieved
 * @retval 0 No message could be retrieved in a timely manner
 */
uint8_t getUImessage(Message* message_to_get) {
    if (!ready) {
        return 0;
    }
    return (xMessageBufferReceive(ui_buffer, message_to_get, sizeof(Message), pdMS_TO_TICKS(kUImessageDelayMS)) ==
            sizeof(Message));
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Dispatcher FreeRTOS task.
 * @details Waits on hardware events and pushes corresponding messages to the UI queue.
 *
 * @param argument Unused
 */
static void runDispatchertask(void* argument) {
    (void)argument;

    uint8_t holding = 0;

    while (1) {
        //if no new event received, loopback
        if (!waitForHardwareEvents(kEventDelayMS)) {
            continue;
        }

        handleAngleChangeEvent();
        handleZeroingEvent();
        handleZeroingCancelEvent();
        handleHoldingEvent(&holding);
        handleSerialCommandEvent();
    }
}

/**
 * Dispatch an event to the UI
 * @details This will push a message to the back of the UI messages queue
 *
 * @param type Message type
 * @param value Value of the message 
 * @retval 1 Message dispatched
 * @retval 0 Timeout or error while dispatching the message
 */
static uint8_t dispatchEventToUI(const MessageID type, const int16_t value) {
    Message message = (Message){.type = type, .value = value};
    return (xMessageBufferSend(ui_buffer, &message, sizeof(Message), 0) == sizeof(Message));
}

/**
 * Handle a Angle Changed hardware event
 */
static void handleAngleChangeEvent(void) {
    if (!isHardwareEventTriggered(kEventXValue) && !isHardwareEventTriggered(kEventYValue)) {
        return;
    }

    dispatchEventToUI(kMessageXValue, getAngleDegreesTenths(kXaxis));
    dispatchEventToUI(kMessageYValue, getAngleDegreesTenths(kYaxis));
}

/**
 * Handle a Zeroing hardware event
 */
static void handleZeroingEvent(void) {
    if (!isHardwareEventTriggered(kEventZero)) {
        return;
    }

    LEDblink(&kWhiteDimmed, kSlowblinkPeriod_ms);
    IMUzeroDown();
    dispatchEventToUI(kMessageZero, 0);
}

/**
 * Handle a Cancel Zeroing hardware event
 */
static void handleZeroingCancelEvent(void) {
    if (!isHardwareEventTriggered(kEventCancelZero)) {
        return;
    }

    LEDsolid(&kBlack);
    IMUcancelZeroing();

    dispatchEventToUI(kMessageCancelZero, 0);
}

/**
 * Handle a Toggle Hold hardware event
 *
 * @param[out] holding The current holding state (updated upon exiting the function)
 */
static void handleHoldingEvent(uint8_t* holding) {
    if (!isHardwareEventTriggered(kEventHold)) {
        return;
    }

    const Colour led_colour = (*holding ? kBlue : kBlack);

    LEDsolid(&led_colour);
    *holding = !(*holding);
}

/**
 * Handle a Serial Command hardware event
 */
static void handleSerialCommandEvent(void) {
    if (!isHardwareEventTriggered(kEventSerialCommand)) {
        return;
    }

    char buffer[kFloatBufferSize];

    GenericCommand command;
    popSerialCommand(&command);

    switch (command.code) {
        case kCmdKI:
            if (!command.is_read) {
                setIMU_KI(command.parameter.float_value);
                break;
            }

            floatToString(getIMU_KI(), buffer, kFloatBufferSize, 3);
            logSerial(kMaxErrorLevel, "%s", buffer);
            break;

        case kCmdKP:
            if (!command.is_read) {
                setIMU_KP(command.parameter.float_value);
                break;
            }

            floatToString(getIMU_KP(), buffer, kFloatBufferSize, 3);
            logSerial(kMaxErrorLevel, "%s", buffer);
            break;

        case kCmdAlignmentEnable:
            if (!command.is_read) {
                setIMUalignmentCheckEnabled((uint8_t)command.parameter.int_value);
                break;
            }

            intToString(isIMUalignmentCheckEnabled(), buffer, kFloatBufferSize);
            logSerial(kMaxErrorLevel, "%s", buffer);
            break;

        case kCmdLogLevel:
            if (!command.is_read) {
                setLogLevel(command.parameter.int_value);
                break;
            }

            intToString(getLogLevel(), buffer, kFloatBufferSize);
            logSerial(kMaxErrorLevel, "%s", buffer);
            break;

        case kCmdNoBehaviour:
        default:
            return;
    }
}
