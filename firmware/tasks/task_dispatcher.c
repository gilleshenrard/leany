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
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stddef.h>
#include <stdint.h>
#include <task.h>

#include "battery.h"
#include "errorstack.h"
#include "generic_command.inc"
#include "hardware_events.h"
#include "led.h"
#include "orientation.inc"
#include "task_imu.h"
#include "task_serial.h"
#include "task_ui.h"

enum {
    kStackSize = 150U,      ///< Amount of words in the task stack
    kTaskLowPriority = 8U,  ///< FreeRTOS number for a low priority task
    kEventDelayMS = 20U,    ///< Number of milliseconds for hardware events delay
    kMutexTimeoutMs = 10U,  ///< Maximum number of milliseconds before considering a mutex timeout
};

//utility tasks
static void runDispatchertask(void* argument);
static void transmitEventsToUI(void);
static void handleZeroingEvent(const GenericCommand* command);
static void handleZeroingCancelEvent(const GenericCommand* command);
static void handleHoldingEvent(uint8_t* holding, const GenericCommand* command);
static uint8_t handleSerialCommandEvent(GenericCommand* command);
static void handleSerialReadCommandEvent(const GenericCommand* command);
static void handleSerialWriteCommandEvent(const GenericCommand* command);
static void handleBatteryStatusEvent(const GenericCommand* command);

static SemaphoreHandle_t events_mutex = NULL;  ///< Mutex used to protect events coming from the dispatcher
static ErrorCode last_error = {.dword = 0};    ///< Last error detected

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create the messages queue and events dispatcher FreeRTOS task
 *
 * @return Success
 */
ErrorCode createMessageDispatchertask(void) {
    static StackType_t task_stack[kStackSize] = {0};    // Buffer used as the task stack
    static StaticTask_t task_state = {0};               // Task state variables
    static StaticSemaphore_t events_mutex_state = {0};  ///< UI mutex state variables

    //create a semaphore to protect events
    events_mutex = xSemaphoreCreateMutexStatic(&events_mutex_state);
    configASSERT(events_mutex);

    // create the static task
    TaskHandle_t task_handle = xTaskCreateStatic(runDispatchertask, "Dispatch task", kStackSize, NULL, kTaskLowPriority,
                                                 task_stack, &task_state);
    configASSERT(task_handle);

    return (kSuccessCode);
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

    GenericCommand command;
    uint8_t holding = 0;

    while (1) {
        //if no new event received, loopback
        if (!waitForHardwareEvents(kEventDelayMS)) {
            continue;
        }

        transmitEventsToUI();

        //check if a serial command has been received
        command = (GenericCommand){0};
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
static void handleZeroingEvent(const GenericCommand* command) {
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
static void handleZeroingCancelEvent(const GenericCommand* command) {
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
static void handleHoldingEvent(uint8_t* holding, const GenericCommand* command) {
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
static uint8_t handleSerialCommandEvent(GenericCommand* command) {
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
static void handleBatteryStatusEvent(const GenericCommand* command) {
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
static void handleSerialReadCommandEvent(const GenericCommand* command) {
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

        case kCmdErrorCode:
        case kCmdBatteryPercent:
        case kCmdBatteryCharge:
        case kCmdToggleHold:
        case kCmdHelp:
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
static void handleSerialWriteCommandEvent(const GenericCommand* command) {
    // A large switch is the most straightforward way to handle serial write commands.
    // Therefore, Lizard linter can ignore this function's length
    // #lizard forgives(length)
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
            setLogLevel(command->parameter.int_value);
            break;

        case kCmdOrientation:
            setDisplayOrientation((uint8_t)command->parameter.int_value);
            triggerHardwareEvent(kEventOrientation);
            break;

        case kCmdBatteryPercent:
            if (setBatteryPercentage((uint8_t)command->parameter.int_value)) {
                triggerHardwareEvent(kEventBatteryStatus);
            }
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

        case kCmdToggleHold:
        case kCmdToggleZero:
        case kCmdHelp:
        case kCmdNoBehaviour:
        default:
            return;
    }
}
