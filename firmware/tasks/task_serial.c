/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file task_serial.c
 * @brief Implement the FreeRTOS task taking care of the serial communication
 * @author Gilles Henrard
 */
#include "task_serial.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <portmacro.h>
#include <projdefs.h>
#include <queue.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_usart.h>
#include <stream_buffer.h>
#include <string.h>
#include <task.h>

#include "errorstack.h"
#include "hardware_events.h"
#include "leany_std.h"
#include "scpi_parser.h"
#include "serial_command_types.h"
#include "systick.h"

enum : uint8_t {
    kOutboundSize = 64U,       ///< Maximum length of a message sent via serial
    kStackSize = 100U,         ///< Amount of words in the task stack
    kTaskLowPriority = 3U,     ///< FreeRTOS number for a low priority task
    kSerialTimeoutMS = 10U,    ///< Maximum number before considering a serial timeout
    kOutboundQueueSize = 15U,  ///< Number of messages the outbound queue can fit
    kCommandsQueueSize = 10U,  ///< Number of codes the commands queue can fit
    kInboundQueueSize = 100U,  ///< Number of characters the inbound queue can fit
    kInboundTimeoutMS = 5U,    ///< Maximum number of milliseconds to wait for rx data
};

/**
 * Enumeration of the IDs of the functions used by the BMI270 implementation
 */
typedef enum : uint8_t {
    kFunctionTask = 1,  ///< taskSerial() : Function running the serial state machine
    kSerialSend = 2,    ///< serialSend() : Function used to send data via serial
} FunctionCode;

/**
 * Definition of an outbound serial message
 */
//NOLINTNEXTLINE(altera-struct-pack-align)
typedef struct {
    char message[kOutboundSize];  ///< Buffer holding the message data
} __attribute((packed())) OutboundMessage;

//private functions
static void runInboundTask(void* argument);
static void runOutboundTask(void* argument);
static ErrorCode serialSend(const char msg[], size_t length);

//state variables
static TaskHandle_t inbound_task_handle = nullptr;              ///< handle of the inbound FreeRTOS task
static TaskHandle_t outbound_task_handle = nullptr;             ///< handle of the outbound FreeRTOS task
static QueueHandle_t queue_outbound = nullptr;                  ///< handle of the outbound message queue
static QueueHandle_t queue_commands = nullptr;                  ///< handle of the parsed commands queue
static volatile StreamBufferHandle_t stream_inbound = nullptr;  ///< First stream of the dual-buffer reception
static ErrorCode result;                                        ///< Variable used to store error codes
static bool is_timed_out = false;                               ///< Variable used to test for timeouts
static ErrorLevel log_level = kErrorError;                      ///< Current minimum error level logging will process

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Handle the reception of a character received via UART
 */
void uartInterruptTriggered(void) {
    //if inbound stream not created yet, exit
    if (!stream_inbound) {
        return;
    }

    //push the character received to the back of the queue
    BaseType_t has_woken = 0;
    const char received = LL_USART_ReceiveData8(USART1);
    LL_USART_ClearFlag_RXNE(USART1);
    xStreamBufferSendFromISR(stream_inbound, &received, 1U, &has_woken);
    portYIELD_FROM_ISR(has_woken);
}

/**
 * Create the FreeRTOS serial communication task
 */
void createSerialtask(void) {
    // NOLINTBEGIN (hicpp-use-nullptr)
    static StackType_t inbound_task_stack[kStackSize];
    static StaticTask_t inbound_task_state;
    static StackType_t outbound_task_stack[kStackSize];
    static StaticTask_t outbound_task_state;
    static OutboundMessage outbound_buffer[kOutboundQueueSize];
    static char inbound_buffer[kInboundQueueSize];
    static SerialCommand commands_buffer[kCommandsQueueSize];
    static StaticQueue_t outbound_state;
    static StaticQueue_t commands_state;
    static StaticStreamBuffer_t inbound_state;
    // NOLINTEND

    //create the static task
    inbound_task_handle = xTaskCreateStatic(runInboundTask, "Inbound serial task", kStackSize, nullptr,
                                            kTaskLowPriority, inbound_task_stack, &inbound_task_state);
    configASSERT(inbound_task_handle);

    outbound_task_handle = xTaskCreateStatic(runOutboundTask, "Outbound serial task", kStackSize, nullptr,
                                             kTaskLowPriority, outbound_task_stack, &outbound_task_state);
    configASSERT(outbound_task_handle);

    queue_outbound =
        xQueueCreateStatic(kOutboundQueueSize, sizeof(OutboundMessage), (uint8_t*)outbound_buffer, &outbound_state);
    configASSERT(queue_outbound);

    stream_inbound = xStreamBufferCreateStatic((kInboundQueueSize * 1U), 1U, (uint8_t*)inbound_buffer, &inbound_state);
    configASSERT(stream_inbound);

    queue_commands =
        xQueueCreateStatic(kCommandsQueueSize, sizeof(SerialCommand), (uint8_t*)commands_buffer, &commands_state);
    configASSERT(queue_commands);

    LL_USART_Enable(USART1);
    LL_USART_EnableIT_RXNE(USART1);
}

/**
 * Send a log message via Serial connection
 *
 * @param level Level of the log to send @note Replies to commands have Max level
 * @param format Format to apply to the log
 * @param ... Variable list of arguments to format
 */
void logSerial(ErrorLevel level, const char format[], ...) {
    //outbound queue not created yet, exit
    if (!queue_outbound) {
        return;
    }

    //log level too low or invalid
    if ((level > kMaxErrorLevel) || (level < log_level)) {
        return;
    }

    // NOLINTBEGIN (clang-analyzer-security.VAList)
    OutboundMessage packed_message;
    va_list args;
    va_start(args, format);

    const bool is_a_log = (level != kMaxErrorLevel);
    if (is_a_log) {
        packed_message.message[0] = '!';
    }

    (void)leany_vsnprintf(&packed_message.message[is_a_log], (kOutboundSize - 1U - (uint8_t)is_a_log), format, args);
    const size_t length = getStringLength(packed_message.message, kOutboundSize - 1);
    packed_message.message[length] = '\n';
    packed_message.message[length + 1U] = '\0';

    va_end(args);
    // NOLINTEND

    //try adding the item to the queue
    (void)xQueueSend(queue_outbound, (void*)&packed_message, pdMS_TO_TICKS(kSerialTimeoutMS));
}

/**
 * Get a serial command to send from the queue
 *
 * @param[out] command_received The command to send
 * @return Whether a command could be retrieved in a timely manner
 */
bool popSerialCommand(SerialCommand* command_received) {
    return (xQueueReceive(queue_commands, command_received, 0) == pdTRUE);
}

/**
 * Set the minimum level a log must have to be sent
 *
 * @param level New level
 */
void setLogLevel(ErrorLevel level) {
    if (level > kMaxErrorLevel) {
        return;
    }

    log_level = level;
}

/**
 * Get the minimum level a log must have to be sent
 *
 * @return Log level
 */
ErrorLevel getLogLevel(void) { return log_level; }

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Run the serial communication task state machine
 *
 * @param argument Unused 
 */
static void runInboundTask(void* argument) {
    (void)argument;

    SerialCommand command_received = {0};
    resetSCPIparser();

    char received = 0;
    while (1) {
        (void)xStreamBufferReceive(stream_inbound, &received, 1U, portMAX_DELAY);

        const bool done = pushSCPIcharacter(received, &command_received);
        if (!done) {
            continue;
        }

        (void)xQueueSend(queue_commands, &command_received, kSerialTimeoutMS);
        triggerHardwareEvent(kEventSerialCommand);
        resetCommand(&command_received);
    }
}

/**
 * Run the serial communication task state machine
 *
 * @param argument Unused 
 */
static void runOutboundTask(void* argument) {
    (void)argument;

    char message[kOutboundSize];
    while (1) {
        if (xQueueReceive(queue_outbound, message, pdMS_TO_TICKS(5U)) != pdTRUE) {
            continue;
        }

        result = serialSend(message, getStringLength(message, kOutboundSize));
    }
}

/**
 * Send data via UART
 *
 * @param msg Data to send
 * @param length Number of bytes to send
 * @retval 0 Success
 * @retval 1 Timeout while waiting for byte to be sent
 */
static ErrorCode serialSend(const char msg[], size_t length) {
    TickType_t start_tick = getCurrentTick();

    if (!length) {
        return kSuccessCode;
    }

    if (!msg) {
        return createErrorCode(kSerialSend, 1, kErrorError);
    }

    is_timed_out = false;
    const char* iterator = msg;
    do {
        LL_USART_TransmitData8(USART1, *(iterator++));

        while (!LL_USART_IsActiveFlag_TC(USART1) && !is_timed_out) {
            is_timed_out = systickTimeout(start_tick, kSerialTimeoutMS);
        };
    } while (--length && !is_timed_out);

    if (is_timed_out) {
        return createErrorCode(kSerialSend, 2, kErrorError);
    }

    return kSuccessCode;
}
