/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file task_dispatcher.c
 * @brief Dispatches hardware events to UI via a FreeRTOS message queue.
 * 
 * This module listens for sensor or button-triggered hardware events,
 * performs any required processing (like zeroing), and sends formatted
 * messages to the UI queue. It uses a statically allocated task and queue.
 * 
 * @note This task must be created after hardware_events group is initialized.
 *
 * @author Gilles Henrard
 */
#include "task_dispatcher.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stddef.h>
#include <stdint.h>
#include <task.h>

#include "errorstack.h"
#include "hardware_events.h"
#include "led.h"
#include "orientation.h"
#include "scpi_commands.h"
#include "serial_command_types.h"
#include "task_battery.h"
#include "task_imu.h"
#include "task_serial.h"
#include "task_ui.h"

enum : uint8_t {
    kStackSize = 150U,           ///< Amount of words in the task stack
    kTaskLowPriority = 3U,       ///< FreeRTOS number for a low priority task
    kEventDelayMS = 20U,         ///< Number of milliseconds for hardware events delay
    kMutexTimeoutMs = 10U,       ///< Maximum number of milliseconds before considering a mutex timeout
    kSCPImaxTreeDepth = 16U,     ///< Maximum depth of the command tree traversal
    kSCPIindentationWidth = 4U,  ///< Number of spaces in the SCPI indentation
    kSCPImaxIndextation = (kSCPImaxTreeDepth * kSCPIindentationWidth),  ///< Maximum size of the indent. buffer
    kExampleStringSize = 32U,                                           ///< Maximum characters in the example string
};

/**
 * Function IDs
 */
typedef enum : uint8_t {
    kDumpTree = 1,         ///< dumpScpiCommandTree(): Function used to send the command tree to serial line
    kSendLine = 2,         ///< sendScpiTreeLine(): Function used to send a command node
    kSendIndentation = 3,  ///< sendIndentation(): Function used to send indentation over serial
} FunctionCode;

/**
 * Structure defining an SCPI node tree traversal frame
 */
typedef struct {
    const Node* node;          ///< Node traversed
    uint8_t next_child_index;  ///< Depth of the next child
} TraversalFrame;

// NOLINTNEXTLINE(misc-use-internal-linkage,readability-identifier-naming)
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName);

//utility tasks
static void runDispatchertask(void* argument);
static void transmitEventsToUI(void);
static void handleZeroingEvent(const SerialCommand* command);
static void handleZeroingCancelEvent(const SerialCommand* command);
static void handleHoldingEvent(uint8_t* holding, const SerialCommand* command);
static uint8_t handleSerialCommandEvent(SerialCommand* command);
static void handleSerialReadCommandEvent(const SerialCommand* command);
static void handleSerialWriteCommandEvent(const SerialCommand* command);
static void handleBatteryStatusEvent(const SerialCommand* command);
static ErrorCode dumpScpiCommandTree(void);
static ErrorCode sendScpiTreeLine(const Node* node, uint8_t depth);
static ErrorCode formatIndentation(uint8_t depth, char out_buffer[kSCPImaxIndextation]);

static SemaphoreHandle_t events_mutex = nullptr;  ///< Mutex used to protect events coming from the dispatcher
static ErrorCode last_error = {.dword = 0};       ///< Last error detected

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create the messages queue and events dispatcher FreeRTOS task
 *
 * @return Success
 */
ErrorCode createMessageDispatchertask(void) {
    // NOLINTBEGIN (hicpp-use-nullptr)
    static StackType_t task_stack[kStackSize] = {0};    // Buffer used as the task stack
    static StaticTask_t task_state = {0};               // Task state variables
    static StaticSemaphore_t events_mutex_state = {0};  ///< UI mutex state variables
    // NOLINTEND

    //create a semaphore to protect events
    events_mutex = xSemaphoreCreateMutexStatic(&events_mutex_state);
    configASSERT(events_mutex);

    // create the static task
    TaskHandle_t task_handle = xTaskCreateStatic(runDispatchertask, "Dispatch task", kStackSize, nullptr,
                                                 kTaskLowPriority, task_stack, &task_state);
    configASSERT(task_handle);

    return kSuccessCode;
}

/**
 * Set the last error code detected
 *
 * @param error Error code
 * @retval 1 Successfully set
 * @retval 0 Could not be set
 */
uint8_t setLastErrorCode(ErrorCode error) {
    if (!events_mutex) {
        return 0;
    }

    if (xSemaphoreTake(events_mutex, pdMS_TO_TICKS(kMutexTimeoutMs)) == pdFALSE) {
        return 0;
    }

    last_error = error;
    (void)xSemaphoreGive(events_mutex);
    return 1;
}

/**
 * Get the last error code detected
 *
 * @param[out] error Error code
 * @retval 1 Successfully retrieved
 * @retval 0 Could not be retrieved
 */
uint8_t getLastErrorCode(ErrorCode* error) {
    if (!events_mutex) {
        return 0;
    }

    if (xSemaphoreTake(events_mutex, pdMS_TO_TICKS(kMutexTimeoutMs)) == pdFALSE) {
        return 0;
    }

    *error = last_error;
    (void)xSemaphoreGive(events_mutex);

    return 1;
}

/**
 * Catch any task stack overflow
 * @details This overrides FreeRTOS's default behaviour
 *
 * @param xTask Handle of the task overflowing
 * @param pcTaskName Name of the task overflowing
 */
// cppcheck-suppress unusedFunction
// NOLINTNEXTLINE(readability-identifier-naming,readability-non-const-parameter)
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName) {
    (void)xTask;
    (void)pcTaskName;

    Error_Handler();
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

    SerialCommand command;
    uint8_t holding = 0;

    while (1) {
        //if no new event received, loopback
        if (!waitForHardwareEvents(kEventDelayMS)) {
            continue;
        }

        transmitEventsToUI();

        //check if a serial command has been received
        command = (SerialCommand){0};
        (void)handleSerialCommandEvent(&command);

        //handle hardware events which need additional behaviour
        handleZeroingEvent(&command);
        handleZeroingCancelEvent(&command);
        handleHoldingEvent(&holding, &command);
        handleBatteryStatusEvent(&command);

        clearHardwareEvents();
    }
}

/**
 * Transfer all the triggered hardware events to the UI
 * @note If any issue occurs with the UI message queue, the event is lost
 */
static void transmitEventsToUI(void) {
    //transmit the triggered events to the UI
    for (uint8_t event = 0; event < (uint8_t)kNbEvents; event++) {
        if (event == kEventSerialCommand) {
            continue;
        }

        if (!isHardwareEventTriggered(event)) {
            continue;
        }

        (void)dispatchEventToUI(event);
    }
}

/**
 * Handle a Zeroing hardware event
 *
 * @param command Latest command received
 */
static void handleZeroingEvent(const SerialCommand* command) {
    if ((command->code == kCmdToggleZero) && !command->is_read && !isIMUzeroed()) {
        triggerHardwareEvent(kEventZero);
    }

    if (!isHardwareEventTriggered(kEventZero)) {
        return;
    }

    LEDsetColour(&kWhiteDimmed);
    LEDsetEffect(kBLINKING, kSlowblinkPeriod_ms);
    IMUzeroDown();
}

/**
 * Handle a Cancel Zeroing hardware event
 *
 * @param command Latest command received
 */
static void handleZeroingCancelEvent(const SerialCommand* command) {
    if ((command->code == kCmdToggleZero) && !command->is_read && isIMUzeroed()) {
        triggerHardwareEvent(kEventCancelZero);
    }

    if (!isHardwareEventTriggered(kEventCancelZero)) {
        return;
    }

    LEDsetEffect(kOFF, 0);
    IMUcancelZeroing();
}

/**
 * Handle a Toggle Hold hardware event
 *
 * @param[out] holding The current holding state (updated upon exiting the function)
 * @param command Latest command received
 */
static void handleHoldingEvent(uint8_t* holding, const SerialCommand* command) {
    if (command->code == kCmdToggleHold) {
        if (command->is_read) {
            logSerial(kMaxErrorLevel, "%u", isIMUmeasurementsHolding());
            return;
        }

        triggerHardwareEvent(kEventHold);
    }

    if (!isHardwareEventTriggered(kEventHold)) {
        return;
    }

    *holding = toggleIMU_hold();
    const Colour led_colour = (*holding ? kBlue : kBlack);
    LEDsetColour(&led_colour);
    LEDsetEffect(kSOLID, 0);
}

/**
 * Handle a Serial Command hardware event
 *
 * @param[out] command Command to populate
 * @retval 1 Command received
 * @retval 0 Command not received
 */
static uint8_t handleSerialCommandEvent(SerialCommand* command) {
    if (!isHardwareEventTriggered(kEventSerialCommand)) {
        return 0;
    }

    popSerialCommand(command);

    if (command->is_read) {
        handleSerialReadCommandEvent(command);
    } else {
        handleSerialWriteCommandEvent(command);
    }

    return 1;
}

/**
 * Handle a battery charge status change event
 *
 * @param command Latest command received
 */
static void handleBatteryStatusEvent(const SerialCommand* command) {
    if (!isHardwareEventTriggered(kEventBatteryStatus) && (command->code != kCmdBatteryPercent) &&
        (command->code != kCmdBatteryCharge)) {
        return;
    }

    BatteryStatus status;
    if (isError(getBatteryStatus(&status))) {
        return;
    }

    if (command->code == kCmdBatteryPercent) {
        if (command->is_read) {
            logSerial(kMaxErrorLevel, "%u", status.level_percents);
            return;
        }

        triggerHardwareEvent(kEventBatteryStatus);
    }

    if (command->code == kCmdBatteryCharge) {
        if (command->is_read) {
            logSerial(kMaxErrorLevel, "%u", status.charging);
            return;
        }

        triggerHardwareEvent(kEventBatteryStatus);
    }

    if (!isHardwareEventTriggered(kEventBatteryStatus)) {
        return;
    }
}

/**
 * Handle specifically a Serial Read Command hardware event
 *
 * @param command Serial Read command to run
 */
static void handleSerialReadCommandEvent(const SerialCommand* command) {
    // A large switch is the most straightforward way to handle serial read commands.
    // Therefore, Lizard linter can ignore this function's length
    // #lizard forgives(cyclomatic_complexity)
    uint8_t orientation = 0;

    switch (command->code) {
        case kCmdKI:
            logSerial(kMaxErrorLevel, "%f", (double)getIMU_KI());
            break;

        case kCmdKP:
            logSerial(kMaxErrorLevel, "%f", (double)getIMU_KP());
            break;

        case kCmdAlignmentEnable:
            logSerial(kMaxErrorLevel, "%u", isIMUalignmentCheckEnabled());
            break;

        case kCmdLogLevel:
            logSerial(kMaxErrorLevel, "%u", getLogLevel());
            break;

        case kCmdToggleZero:
            logSerial(kMaxErrorLevel, "%u", isIMUzeroed());
            break;

        case kCmdOrientation:
            getDisplayOrientation((Orientation*)&orientation);
            logSerial(kMaxErrorLevel, "%u", orientation);
            break;

        case kCmdHelp:
            dumpScpiCommandTree();
            break;

        case kCmdBatteryOff:
        case kCmdErrorCode:
        case kCmdBatteryPercent:
        case kCmdBatteryCharge:
        case kCmdToggleHold:
        case kCmdNoBehaviour:
        case kCmdToggleScreen:
        case kCmdLedColour:
        case kCmdLedEffect:
        default:
            return;
    }
}

/**
 * Handle specifically a Serial Write Command hardware event
 *
 * @param command Serial Write command to run
 */
static void handleSerialWriteCommandEvent(const SerialCommand* command) {
    // A large switch is the most straightforward way to handle serial write commands.
    // Therefore, Lizard linter can ignore this function's length
    // #lizard forgives(length, cyclomatic_complexity)
    ErrorCode error;

    switch (command->code) {
        case kCmdKI:
            setIMU_KI(command->parameter.float_value);
            break;

        case kCmdKP:
            setIMU_KP(command->parameter.float_value);
            break;

        case kCmdAlignmentEnable:
            setIMUalignmentCheckEnabled((uint8_t)command->parameter.int_value);
            break;

        case kCmdLogLevel:
            setLogLevel((ErrorLevel)command->parameter.int_value);
            break;

        case kCmdOrientation:
            setDisplayOrientation((uint8_t)command->parameter.int_value);
            triggerHardwareEvent(kEventOrientation);
            break;

        case kCmdBatteryCharge:
            setBatteryChargeStatus((uint8_t)command->parameter.int_value);
            triggerHardwareEvent(kEventBatteryStatus);
            break;

        case kCmdToggleScreen:
            triggerHardwareEvent(kEventToggleScreen);
            break;

        case kCmdLedColour:
            LEDsetColourHex(command->parameter.int_value);
            break;

        case kCmdLedEffect:
            LEDsetEffect((uint8_t)command->parameter.int_value, kSlowblinkPeriod_ms);
            break;

        case kCmdErrorCode:
            error = (ErrorCode){.dword = command->parameter.int_value};
            setLastErrorCode(error);
            triggerHardwareEvent(kEventErrorCode);
            break;

        case kCmdBatteryOff:
            error = turnSystemOff();
            setLastErrorCode(error);
            break;

        case kCmdBatteryPercent:
        case kCmdToggleHold:
        case kCmdToggleZero:
        case kCmdHelp:
        case kCmdNoBehaviour:
        default:
            return;
    }
}

/**
 * Send the whole command tree to serial
 *
 * @retval 0 Success
 * @retval 1 Root node is nullptr
 * @retval 2 Error while sending a tree node
 * @retval 3 Command tree is too deep
 */
static ErrorCode dumpScpiCommandTree(void) {
    TraversalFrame traversal_stack[kSCPImaxTreeDepth];
    uint8_t stack_depth = 0U;

    const char example[kExampleStringSize] = "Example -> :log:level 1\n";
    logSerial(kMaxErrorLevel, "%s", example);

    const Node* root_node = getRootNode();
    if (!root_node) {
        return createErrorCode(kDumpTree, 1, kErrorError);
    }

    traversal_stack[0].node = root_node;
    traversal_stack[0].next_child_index = 0U;
    stack_depth = 1U;

    while (stack_depth > 0U) {
        TraversalFrame* current_frame = nullptr;
        const Node* current_node = nullptr;

        current_frame = &traversal_stack[stack_depth - 1U];
        current_node = current_frame->node;

        // if current node has no child, print it
        if (current_frame->next_child_index == 0U) {
            ErrorCode error_code = sendScpiTreeLine(current_node, (uint8_t)(stack_depth - 1U));
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
 * @retval 1 Error while formatting the indentation
 */
static ErrorCode sendScpiTreeLine(const Node* node, uint8_t depth) {
    // Ignore the node if root
    if (depth == 0U) {
        return kSuccessCode;
    }

    char indentation[kSCPImaxIndextation];
    ErrorCode error_code = formatIndentation(depth, indentation);
    EXIT_ON_ERROR(error_code, kSendLine, 1)

    logSerial(kMaxErrorLevel, "%s:%s", indentation, node->scpi.long_name);
    return kSuccessCode;
}

/**
 * Format the buffer with the indentation corresponding of the depth in the tree
 *
 * @param depth Node depth
 * @param[out] out_buffer Buffer into which store the indentation
 * @return Success
 */
static ErrorCode formatIndentation(uint8_t depth, char out_buffer[kSCPImaxIndextation]) {
    // Starting index at 1 to ignore root
    uint8_t index = 0;
    for (index = 0U; index < (depth * kSCPIindentationWidth); index++) {
        out_buffer[index] = ' ';
    }
    out_buffer[index] = '\0';

    return kSuccessCode;
}
