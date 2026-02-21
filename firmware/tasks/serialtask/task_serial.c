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
#include "leany_std.h"
#include "scpi_commands.h"
#include "scpi_parser.h"

enum {
    kOutboundSize = 64U,       ///< Maximum length of a message sent via serial
    kStackSize = 200U,         ///< Amount of words in the task stack
    kTaskLowPriority = 8U,     ///< FreeRTOS number for a low priority task
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
static ErrorCode dumpScpiCommandTree(void);
static ErrorCode sendIndentation(uint8_t depth);
static ErrorCode sendScpiTreeLine(const Node* node, uint8_t depth);

//state variables
static TaskHandle_t task_handle = NULL;                      ///< handle of the FreeRTOS task
static QueueHandle_t queue_outbound = NULL;                  ///< handle of the outbound message queue
static QueueHandle_t queue_commands = NULL;                  ///< handle of the parsed commands queue
static volatile StreamBufferHandle_t stream_inbound = NULL;  ///< First stream of the dual-buffer reception
static ErrorCode result;                                     ///< Variable used to store error codes
static uint8_t timeout_value = 0;                            ///< Variable used to test for timeouts
static ErrorLevel log_level = kErrorError;                   ///< Current minimum error level logging will process

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
    configASSERT(task_handle);

    queue_outbound =
        xQueueCreateStatic(kOutboundQueueSize, sizeof(OutboundMessage), (uint8_t*)outbound_buffer, &outbound_state);
    configASSERT(queue_outbound);

    stream_inbound = xStreamBufferCreateStatic((kInboundQueueSize * 1U), 1U, (uint8_t*)inbound_buffer, &inbound_state);
    configASSERT(stream_inbound);

    queue_commands =
        xQueueCreateStatic(kCommandsQueueSize, sizeof(GenericCommand), (uint8_t*)commands_buffer, &commands_state);
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
                if (command_received.code == kCmdHelp) {
                    (void)dumpScpiCommandTree();
                }
                (void)xQueueSend(queue_commands, &command_received, kSerialTimeoutMS);
                triggerHardwareEvent(kEventSerialCommand);
                resetCommand(&command_received);
            }
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
    TickType_t start_tick = xTaskGetTickCount();

    if (!length) {
        return kSuccessCode;
    }

    if (!msg) {
        return createErrorCode(kSerialSend, 1, kErrorError);
    }

    const char* iterator = msg;
    do {
        LL_USART_TransmitData8(USART1, *(iterator++));
        EXIT_ON_TIMEOUT(LL_USART_IsActiveFlag_TC(USART1), kSerialTimeoutMS, kSerialSend, 2)
    } while (--length && !timeout(start_tick, kSerialTimeoutMS));

    return kSuccessCode;
}

/**
 * Send the whole command tree to serial
 *
 * @retval 0 Success
 * @retval 1 Root node is null
 * @retval 2 Error while sending a tree node
 */
static ErrorCode dumpScpiCommandTree(void) {
    TraversalFrame traversal_stack[kSCPImaxTreeDepth];
    uint8_t stack_depth = 0U;
    ErrorCode error_code;

    const char example[kExampleStringSize] = "Example -> :imu:kp 0.5\n\n";
    serialSend(example, kExampleStringSize);

    const Node* root_node = getRootNode();
    if (!root_node) {
        return createErrorCode(kDumpTree, 1, kErrorError);
    }

    traversal_stack[0].node = root_node;
    traversal_stack[0].next_child_index = 0U;
    stack_depth = 1U;

    while (stack_depth > 0U) {
        TraversalFrame* current_frame = NULL;
        const Node* current_node = NULL;

        current_frame = &traversal_stack[stack_depth - 1U];
        current_node = current_frame->node;

        // if current node has no child, print it
        if (current_frame->next_child_index == 0U) {
            error_code = sendScpiTreeLine(current_node, stack_depth - (uint8_t)1U);
            EXIT_ON_ERROR(error_code, kDumpTree, 2)
        }

        if (current_frame->next_child_index >= current_node->nb_children) {
            stack_depth--;
            continue;
        }

        if (stack_depth >= kSCPImaxTreeDepth) {
            return createErrorCode(kDumpTree, 3, kErrorError);
        }

        const Node* child_node = &current_node->children[current_frame->next_child_index];
        current_frame->next_child_index++;
        traversal_stack[stack_depth].node = child_node;
        traversal_stack[stack_depth].next_child_index = 0U;
        stack_depth++;
    }

    return kSuccessCode;
}

/**
 * Send a command node over serial
 *
 * @param node Node to send
 * @param depth Node depth in the tree
 * @retval 0 Success
 * @retval 1 Error while sending the indentation
 * @retval 2 Error while sending the ':' character
 * @retval 3 Error while sending the node
 */
static ErrorCode sendScpiTreeLine(const Node* node, uint8_t depth) {
    // Ignore the node if root
    if (depth == 0U) {
        return kSuccessCode;
    }

    ErrorCode error_code = sendIndentation(depth);
    EXIT_ON_ERROR(error_code, kSendLine, 1)

    const size_t name_length = getStringLength(node->scpi.long_name, kSCPImaxCommandSize);
    error_code = serialSend(":", 1U);
    EXIT_ON_ERROR(error_code, kSendLine, 2)
    error_code = serialSend(node->scpi.long_name, name_length);
    EXIT_ON_ERROR(error_code, kSendLine, 3)

    // error_code = serialSend(scpiCommandSuffix(&node->scpi), getStringLength(scpiCommandSuffix(&node->scpi), 64U));
    // EXIT_ON_ERROR(error_code, 3, 4)

    return serialSend("\n", 1U);
}

/**
 * Send indentation over serial depending on depth
 *
 * @param depth Node depth
 * @retval 0 Success
 * @retval 1 Error while sending the indentation
 */
static ErrorCode sendIndentation(uint8_t depth) {
    // Starting index at 1 to ignore root
    for (uint8_t index = 1U; index < depth; index++) {
        ErrorCode error_code = serialSend("    ", 4U);
        EXIT_ON_ERROR(error_code, kSendIndentation, 1)
    }

    return kSuccessCode;
}
