/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file hardware_events.c
 * @brief Implement the hardware events group
 * @author Gilles Henrard
 * @date 27/07/2025
 */
#include "hardware_events.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <event_groups.h>
#include <portmacro.h>
#include <projdefs.h>
#include <stdint.h>

//NOLINTNEXTLINE (cppcoreguidelines-avoid-magic-numbers)
_Static_assert((uint8_t)(kNbEvents < 32U), "Maximum 31 hardware events can be defined");

static EventGroupHandle_t hardware_events_group = (void*)0;  ///< Hardware events group handle
static EventBits_t latest_events = 0;                        ///< Latest hardware events monitored

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Create the hardware events group
 * @details Only one instance will be created during the application's lifecycle
 */
void createHardwareEventsGroup(void) {
    static StaticEventGroup_t group_state;

    //if instance already created, exit
    if (hardware_events_group) {
        return;
    }

    hardware_events_group = xEventGroupCreateStatic(&group_state);
    configASSERT(hardware_events_group);
}

/**
 * Trigger a hardware event
 *
 * @param event Event to trigger
 */
void triggerHardwareEvent(Event event) {
    if (!hardware_events_group) {
        return;
    }

    if (event >= kNbEvents) {
        return;
    }

    (void)xEventGroupSetBits(hardware_events_group, (1U << event));
}

/**
 * Check if a hardware event has been triggered
 *
 * @param event Event to check 
 * @retval 1 Event triggered
 * @retval 0 Event not triggered
 */
uint8_t isHardwareEventTriggered(Event event) {
    if (event >= kNbEvents) {
        return 0;
    }

    return (latest_events & (1U << event)) != 0;
}

/**
 * Wait for hardware events to occur.
 *
 * @param max_wait_time_ms  Maximum number of milliseconds to wait for events
 * @retval 1 At least one new event occurred during the wait (rising edge)
 * @retval 0 No new event occurred (all returned events were already pending before the wait)
 *
 * @note This function only reports "new" events (rising edges). 
 *       If an event bit was already set before the wait, and no new events
 *       occurred during the wait, the function will return 0 even though 
 *       the caller will have consumed the old event.
 */

uint8_t waitForHardwareEvents(TickType_t max_wait_time_ms) {
    if (!hardware_events_group) {
        return 0;
    }

    const EventBits_t all_events_mask = (1U << (EventBits_t)kNbEvents) - 1U;
    const EventBits_t old_events = xEventGroupGetBits(hardware_events_group);
    latest_events =
        xEventGroupWaitBits(hardware_events_group, all_events_mask, pdTRUE, pdFALSE, pdMS_TO_TICKS(max_wait_time_ms));

    //check if the events changed and if any bit has a rising edge
    return (((latest_events ^ old_events) & latest_events) != 0);
}
