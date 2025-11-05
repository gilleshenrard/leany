/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
/**
 * @brief Implement the FreeRTOS task taking care of UART serial communication
 * @author Gilles Henrard
 * @date 05/11/2025
 */
#include "serial.h"

#include <FreeRTOS.h>
#include <main.h>
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
#include "generic_command.inc"
#include "hardware_events.h"
#include "scpi_parser.h"

enum {
    kOutboundSize = 32U,       ///< Maximum length of a message sent via serial
    kStackSize = 150U,         ///< Amount of words in the task stack
    kTaskLowPriority = 8U,     ///< FreeRTOS number for a low priority task
    kSerialTimeoutMS = 10U,    ///< Maximum number before considering a serial timeout
    kOutboundQueueSize = 15U,  ///< Number of messages the outbound queue can fit
    kCommandsQueueSize = 10U,  ///< Number of codes the commands queue can fit
    kInboundQueueSize = 100U,  ///< Number of characters the inbound queue can fit
    kInboundTimeoutMS = 5U,    ///< Maximum number of milliseconds to wait for rx data
};

/**
 * Enumeration of the IDs of the functions used by the BMI270 implementation
 */
typedef enum {
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
static void taskSerial(void* argument);
static ErrorCode serialSend(const char msg[], size_t length);

//state variables
static TaskHandle_t task_handle = NULL;             ///< handle of the FreeRTOS task
static QueueHandle_t queue_outbound = NULL;         ///< handle of the outbound message queue
static QueueHandle_t queue_commands = NULL;         ///< handle of the parsed commands queue
static StreamBufferHandle_t stream_inbound = NULL;  ///< First stream of the dual-buffer reception
static ErrorCode result;                            ///< Variable used to store error codes
static uint8_t timeout_value = 0;                   ///< Variable used to test for timeouts
static ErrorLevel log_level = kErrorError;          ///< Current minimum error level logging will process

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
    static StackType_t task_stack[kStackSize] = {0};  ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};             ///< Task state variables
    static OutboundMessage outbound_buffer[kOutboundQueueSize];
    static char inbound_buffer[kInboundQueueSize];
    static GenericCommand commands_buffer[kCommandsQueueSize];
    static StaticQueue_t outbound_state;
    static StaticQueue_t commands_state;
    static StaticStreamBuffer_t inbound_state;

    //create the static task
    task_handle =
        xTaskCreateStatic(taskSerial, "Serial task", kStackSize, NULL, kTaskLowPriority, task_stack, &task_state);
    if (!task_handle) {
        Error_Handler();
    }

    queue_outbound =
        xQueueCreateStatic(kOutboundQueueSize, sizeof(OutboundMessage), (uint8_t*)outbound_buffer, &outbound_state);
    if (!queue_outbound) {
        Error_Handler();
    }

    stream_inbound = xStreamBufferCreateStatic((kInboundQueueSize * 1U), 1U, (uint8_t*)inbound_buffer, &inbound_state);
    if (!stream_inbound) {
        Error_Handler();
    }

    queue_commands =
        xQueueCreateStatic(kCommandsQueueSize, sizeof(GenericCommand), (uint8_t*)commands_buffer, &commands_state);
    if (!queue_commands) {
        Error_Handler();
    }
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

    OutboundMessage packed_message;
    va_list args;
    va_start(args, format);

    const uint8_t is_a_log = (level != kMaxErrorLevel);
    if (is_a_log) {
        packed_message.message[0] = '!';
    }

    (void)vsnprintf(&packed_message.message[is_a_log], (kOutboundSize - 1U - is_a_log), format, args);
    const size_t length = strnlen(packed_message.message, kOutboundSize - 1);
    packed_message.message[length] = '\n';
    packed_message.message[length + 1U] = '\0';

    va_end(args);

    //try adding the item to the queue
    (void)xQueueSend(queue_outbound, (void*)&packed_message, pdMS_TO_TICKS(kSerialTimeoutMS));
}

/**
 * Get a serial command to send from the queue
 *
 * @param[out] command_received The command to send
 * @return Whether a command could be retrieved in a timely manner
 */
uint8_t popSerialCommand(GenericCommand* command_received) {
    return (xQueueReceive(queue_commands, command_received, 0) == pdTRUE);
}

/**
 * Parse a float to a string
 *
 * @param value Value to parse 
 * @param[out] out_buffer Buffer in which store the parsed string
 * @param buffer_size Maximum number of characters in the buffer
 * @param precision Number of decimals to parse after the separator
 */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void floatToString(float value, char out_buffer[], uint8_t buffer_size, uint8_t precision) {
    // Extract integer part
    uint16_t ipart = (uint16_t)value;

    // Extract floating part
    float fpart = value - (float)ipart;

    // convert integer part to string
    const uint8_t written = (uint8_t)snprintf(out_buffer, (buffer_size - 2U), "%u", ipart);
    out_buffer[written] = '.';

    if (!precision) {
        out_buffer[written + 1] = '0';
        out_buffer[written + 2] = '\0';
        return;
    }

    while (precision--) {
        fpart *= 10.0F;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
    }
    (void)snprintf(&out_buffer[written + 1], (buffer_size - written - 1U), "%u", (uint16_t)fpart);
}

void intToString(uint32_t value, char out_buffer[], uint8_t buffer_size) {
    if (!value) {
        out_buffer[0] = '0';
        out_buffer[1] = '\n';
        return;
    }

    uint32_t magnitude = 1U;
    while (value / magnitude) {
        magnitude *= 10U;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
    }
    magnitude /= 10U;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)

    uint8_t index = 0;
    while ((index < buffer_size) && magnitude) {
        out_buffer[index] = '0' + (char)(value / magnitude);
        index++;
        magnitude /= 10U;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
    }
    out_buffer[index] = '\0';
}

void setLogLevel(ErrorLevel level) {
    if (level > kMaxErrorLevel) {
        return;
    }

    log_level = level;
}

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

    GenericCommand command_received = {0};
    resetSCPIparser();

    char received = 0;
    while (1) {
        //wait for a while until either data ready or timeout
        if (xStreamBufferReceive(stream_inbound, &received, 1U, pdMS_TO_TICKS(kInboundTimeoutMS)) > 0) {
            const uint8_t done = pushSCPIcharacter(received, &command_received);
            if (done) {
                (void)xQueueSend(queue_commands, &command_received, kSerialTimeoutMS);
                triggerHardwareEvent(kEventSerialCommand);
                resetCommand(&command_received);
            }
            continue;
        }

        //if there are messages to send, pull a one and send it
        if (xQueueReceive(queue_outbound, message, 0U) == pdTRUE) {
            result = serialSend(message, strnlen(message, kOutboundSize));
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
    TickType_t start_tick = xTaskGetTickCount();

    const char* iterator = msg;
    do {
        LL_USART_TransmitData8(USART1, *(iterator++));
        EXIT_ON_TIMEOUT(LL_USART_IsActiveFlag_TC(USART1), kSerialTimeoutMS, kSerialSend, 1)
    } while (--length && !timeout(start_tick, kSerialTimeoutMS));

    return kSuccessCode;
}
