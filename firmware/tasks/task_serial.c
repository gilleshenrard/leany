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
#include "scpi_commands.h"
#include "scpi_parser.h"
#include "serial_command_types.h"
#include "systick.h"

enum {
    kOutboundSize = 64U,       ///< Maximum length of a message sent via serial
    kStackSize = 200U,         ///< Amount of words in the task stack
    kTaskLowPriority = 3U,     ///< FreeRTOS number for a low priority task
    kSerialTimeoutMS = 10U,    ///< Maximum number before considering a serial timeout
    kOutboundQueueSize = 15U,  ///< Number of messages the outbound queue can fit
    kCommandsQueueSize = 10U,  ///< Number of codes the commands queue can fit
    kInboundQueueSize = 100U,  ///< Number of characters the inbound queue can fit
    kInboundTimeoutMS = 5U,    ///< Maximum number of milliseconds to wait for rx data
    kSCPImaxTreeDepth = 16U,   ///< Maximum depth of the command tree traversal
    kExampleStringSize = 32U,  ///< Maximum characters in the example string
};

/**
 * Structure defining an SCPI node tree traversal frame
 */
typedef struct {
    const Node* node;          ///< Node traversed
    uint8_t next_child_index;  ///< Depth of the next child
} TraversalFrame;

/**
 * Enumeration of the IDs of the functions used by the BMI270 implementation
 */
typedef enum {
    kFunctionTask = 1,     ///< taskSerial() : Function running the serial state machine
    kSerialSend = 2,       ///< serialSend() : Function used to send data via serial
    kDumpTree = 3,         ///< dumpScpiCommandTree(): Function used to send the command tree to serial line
    kSendLine = 4,         ///< sendScpiTreeLine(): Function used to send a command node
    kSendIndentation = 5,  ///< sendIndentation(): Function used to send indentation over serial
} FunctionCode;

/**
 * Definition of an outbound serial message
 */
//NOLINTNEXTLINE(altera-struct-pack-align)
typedef struct {
    char message[kOutboundSize];  ///< Buffer holding the message data
} __attribute((packed())) OutboundMessage;

//private functions
static void taskSerial(void* argument);
static ErrorCode serialSend(const char msg[], size_t length);

//state variables
static TaskHandle_t task_handle = nullptr;                      ///< handle of the FreeRTOS task
static QueueHandle_t queue_outbound = nullptr;                  ///< handle of the outbound message queue
static QueueHandle_t queue_commands = nullptr;                  ///< handle of the parsed commands queue
static volatile StreamBufferHandle_t stream_inbound = nullptr;  ///< First stream of the dual-buffer reception
static ErrorCode result;                                        ///< Variable used to store error codes
static uint8_t is_timed_out = 0;                                ///< Variable used to test for timeouts
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
    static StackType_t task_stack[kStackSize] = {0};  ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};             ///< Task state variables
    static OutboundMessage outbound_buffer[kOutboundQueueSize];
    static char inbound_buffer[kInboundQueueSize];
    static SerialCommand commands_buffer[kCommandsQueueSize];
    static StaticQueue_t outbound_state;
    static StaticQueue_t commands_state;
    static StaticStreamBuffer_t inbound_state;
    // NOLINTEND

    //create the static task
    task_handle =
        xTaskCreateStatic(taskSerial, "Serial task", kStackSize, nullptr, kTaskLowPriority, task_stack, &task_state);
    configASSERT(task_handle);

    queue_outbound =
        xQueueCreateStatic(kOutboundQueueSize, sizeof(OutboundMessage), (uint8_t*)outbound_buffer, &outbound_state);
    configASSERT(queue_outbound);

    stream_inbound = xStreamBufferCreateStatic((kInboundQueueSize * 1U), 1U, (uint8_t*)inbound_buffer, &inbound_state);
    configASSERT(stream_inbound);

    queue_commands =
        xQueueCreateStatic(kCommandsQueueSize, sizeof(SerialCommand), (uint8_t*)commands_buffer, &commands_state);
    configASSERT(queue_commands);
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

    const uint8_t is_a_log = (level != kMaxErrorLevel);
    if (is_a_log) {
        packed_message.message[0] = '!';
    }

    //NOLINTNEXTLINE(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, clang-diagnostic-format-nonliteral)
    (void)leany_vsnprintf(&packed_message.message[is_a_log], (kOutboundSize - 1U - is_a_log), format, args);
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
uint8_t popSerialCommand(SerialCommand* command_received) {
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
static void taskSerial(void* argument) {
    char message[kOutboundSize];

    (void)argument;

    LL_USART_Enable(USART1);
    LL_USART_EnableIT_RXNE(USART1);

    SerialCommand command_received = {0};
    resetSCPIparser();

    char received = 0;
    while (1) {
        //wait for a while until either data ready or timeout
        if (xStreamBufferReceive(stream_inbound, &received, 1U, pdMS_TO_TICKS(kInboundTimeoutMS)) > 0) {
            const uint8_t done = pushSCPIcharacter(received, &command_received);
            if (!done) {
                continue;
            }

            (void)xQueueSend(queue_commands, &command_received, kSerialTimeoutMS);
            triggerHardwareEvent(kEventSerialCommand);
            resetCommand(&command_received);
            continue;
        }

        //if there are messages to send, pull a one and send it
        if (xQueueReceive(queue_outbound, message, 0U) == pdTRUE) {
            result = serialSend(message, getStringLength(message, kOutboundSize));
        }
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

    is_timed_out = 0;
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
