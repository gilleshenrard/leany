/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file task_ui.c
 * @brief Implement the display UI
 * @author Gilles Henrard
 */
#include "task_ui.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <hardware_events.h>
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f1xx_hal_def.h>
#include <task.h>

#include "errorstack.h"
#include "screen_error.h"
#include "screen_main.h"
#include "screen_system.h"
#include "st7735s.h"
#include "task_dispatcher.h"

enum {
    kStackSize = 300U,       ///< Amount of words in the task stack
    kTaskLowPriority = 8U,   ///< FreeRTOS number for a low priority task
    kUIqueueLength = 50U,    ///< Number of slots available in the UI queue
    kUImessageDelayMS = 2U,  ///< Number of milliseconds for ui messages delays
    kFillBackground = 1U,    ///< fillBackground() function code
    kPrintVertLine = 2U,     ///< printRectangle() function code
    kPrintLabel = 3,         ///< printLabel() function
    kMutexTimeoutMs = 10U,   ///< Maximum number of milliseconds before considering a mutex timeout
    kAnimationSleepMs = 5U,  ///< Minimum wait in [ms] between two update loops
};

/**
 * Screens
 */
typedef enum {
    kScreenMain = 0,  ///< Main screen
    kScreenSystem,    ///< System information screen
    kScreenError,     ///< Error screen
} Screen;

_Static_assert((kNbEvents < UINT8_MAX), "Too many events. Event might overflow");

static void runUItask(void* argument);
static ErrorCode treatHardwareEvents(void);
static uint8_t copyEvents(uint8_t message_flags[kNbEvents]);
static ErrorCode treatToggleScreenMessage(const uint8_t message_flags[kNbEvents]);
static ErrorCode treatErrorCodeMessage(const uint8_t message_flags[kNbEvents]);

static volatile TaskHandle_t task_handle = NULL;  ///< handle of the FreeRTOS task
static uint8_t message_received[kNbEvents];       ///< Flag indicating whether a UI message was received
static Screen current_screen = kScreenMain;       ///< Current screen displayed and updated
static SemaphoreHandle_t ui_mutex = NULL;         ///< Mutex used to protect UI resources
static SemaphoreHandle_t events_mutex = NULL;     ///< Mutex used to protect events coming from the dispatcher

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create the UI FreeRTOS task
 *
 * @return Success
 */
ErrorCode createUItask(void) {
    static StackType_t task_stack[kStackSize] = {0};    ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};               ///< Task state variables}
    static StaticSemaphore_t ui_mutex_state = {0};      ///< UI mutex state variables
    static StaticSemaphore_t events_mutex_state = {0};  ///< UI mutex state variables

    //create a semaphore to protect UI resources
    ui_mutex = xSemaphoreCreateMutexStatic(&ui_mutex_state);
    configASSERT(ui_mutex);

    //create a semaphore to protect events
    events_mutex = xSemaphoreCreateMutexStatic(&events_mutex_state);
    configASSERT(events_mutex);

    // create the static task
    task_handle = xTaskCreateStatic(runUItask, "UI task", kStackSize, NULL, kTaskLowPriority, task_stack, &task_state);
    configASSERT(task_handle);

    return (kSuccessCode);
}

/**
 * Dispatch an event to the UI
 *
 * @param event Hardware event
 * @retval 1 Message dispatched
 * @retval 0 Timeout or error while dispatching the message
 */
uint8_t dispatchEventToUI(const Event event) {
    if (event >= kNbEvents) {
        return 0;
    }

    if (!events_mutex) {
        return 0;
    }

    if (xSemaphoreTake(events_mutex, pdMS_TO_TICKS(kMutexTimeoutMs)) == pdFALSE) {
        return 0;
    }

    message_received[event] = 1;
    (void)xSemaphoreGive(events_mutex);

    return 1;
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Run the UI task
 *
 * @param argument Unused
 */
static void runUItask(void* argument) {
    UNUSED(argument);
    ErrorCode result;

    attachUItask(task_handle);
    result = configureST7735S();
    if (isError(result)) {
        Error_Handler();
    }

    result = setupMainScreen();
    if (isError(result)) {
        Error_Handler();
    }

    //turn on backlight
    turnBacklightON();

    TickType_t previous_tick = 0;  //force first update as soon as possible
    while (1) {
        //if on error screen, run its animation and keep task alive
        if (current_screen == kScreenError) {
            (void)updateErrorAnimation();
            vTaskDelay(pdMS_TO_TICKS(kAnimationSleepMs));
            continue;
        }

        //Wait until a specific amount of milliseconds passed since last update
        static const uint8_t kRefreshDelayMS = 30U;
        vTaskDelayUntil(&previous_tick, pdMS_TO_TICKS(kRefreshDelayMS));

        result = treatHardwareEvents();
        if (isError(result)) {
            Error_Handler();
        }
    }
}

/**
 * Copy the events triggered by the dispatcher to a local buffer
 *
 * @param[out] message_flags Buffer to which copy the events
 * @retval 1 Copy success
 * @retval 0 Unable to retrieve the hardware events
 */
static uint8_t copyEvents(uint8_t message_flags[kNbEvents]) {
    if (!events_mutex) {
        return 0;
    }

    if (xSemaphoreTake(events_mutex, pdMS_TO_TICKS(kMutexTimeoutMs)) == pdFALSE) {
        return 0;
    }

    for (uint8_t message = 0; message < (uint8_t)kNbEvents; message++) {
        message_flags[message] = message_received[message];
        message_received[message] = 0;
    }

    (void)xSemaphoreGive(events_mutex);
    return 1;
}

/**
 * Treat switch screen requests
 *
 * @param message_flags Array of flags indicating which messages were received
 * @retval 0 Success
 * @retval 1 Error while switching screens
 */
static ErrorCode treatToggleScreenMessage(const uint8_t message_flags[kNbEvents]) {
    if (!message_flags[kEventToggleScreen]) {
        return kSuccessCode;
    }

    ErrorCode result = kSuccessCode;
    if (current_screen == kScreenMain) {
        current_screen = kScreenSystem;
        result = setupSystemScreen();
    } else {
        current_screen = kScreenMain;
        result = setupMainScreen();
    }

    EXIT_ON_ERROR(result, 1, kErrorError)

    return kSuccessCode;
}

/**
 * Treat hardware events coming from the dispatcher
 *
 * @retval 0 Success
 * @retval 1 Error while treating Toggle Screen messages
 * @retval 2 Error while treating Error Code messages
 * @retval 3 Error while treating messages addresses to the current screen
 */
static ErrorCode treatHardwareEvents(void) {
    ErrorCode result;

    //gather the events sent by the dispatcher
    uint8_t latest_flags[kNbEvents];
    if (!copyEvents(latest_flags)) {
        return kSuccessCode;
    }

    //treat the main/system screen toggle messages
    result = treatToggleScreenMessage(latest_flags);
    EXIT_ON_ERROR(result, 1, 1);

    //treat error code messages -> error screen
    result = treatErrorCodeMessage(latest_flags);
    EXIT_ON_ERROR(result, 1, 2);

    //treat additional messages depending on the current screen
    switch (current_screen) {
        case kScreenMain:
            result = treatMainScreenMessages(latest_flags);
            break;

        case kScreenSystem:
            result = treatSystemScreenMessages(latest_flags);
            break;

        case kScreenError:
        default:
            break;
    }

    EXIT_ON_ERROR(result, 1, 3);

    return kSuccessCode;
}

/**
 * Treat switch screen requests
 *
 * @param message_flags Array of flags indicating which messages were received
 * @retval 0 Success
 * @retval 1 Error while switching screens
 */
static ErrorCode treatErrorCodeMessage(const uint8_t message_flags[kNbEvents]) {
    //Message flags array is null
    if (!message_flags[kEventErrorCode]) {
        return kSuccessCode;
    }

    //retrieving the current error and release mutex
    ErrorCode error = kSuccessCode;
    if (!getLastErrorCode(&error)) {
        return kSuccessCode;
    }
    if (!isError(error)) {
        return kSuccessCode;
    }

    //trigger the error screen
    current_screen = kScreenError;
    ErrorCode result = setupErrorScreen(&error);
    EXIT_ON_ERROR(result, 1, kErrorError)

    return kSuccessCode;
}
