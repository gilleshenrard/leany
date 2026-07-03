/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file hardware_events.c
 * @brief Implement the hardware events group
 * @author Gilles Henrard
 */
#include "hardware_events.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <event_groups.h>
#include <portmacro.h>
#include <projdefs.h>
#include <stddef.h>

_Static_assert((kNbEvents < 31U), "Too many events declared");  // NOLINT (cppcoreguidelines-avoid-magic-numbers)

static inline EventBits_t eventToBitmask(Event event);

static EventGroupHandle_t hardware_events_group = nullptr;  ///< Hardware events group handle
static EventBits_t latest_events = 0;                       ///< Latest hardware events monitored

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Create the hardware events group
 */
void createHardwareEventsGroup(void) {
    static StaticEventGroup_t group_state;

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

    (void)xEventGroupSetBits(hardware_events_group, eventToBitmask(event));
}

/**
 * Check if a hardware event has been triggered
 *
 * @param event Event to check 
 * @retval true Event triggered
 * @retval false Event not triggered
 */
bool isHardwareEventTriggered(Event event) { return ((latest_events & eventToBitmask(event)) != 0); }

/**
 * Wait for a hardware event to occur
 *
 * @param max_wait_time_ms  Maximum number of milliseconds to wait for an event
 * @retval true New events triggered
 * @retval false No new event triggered
 */
bool waitForHardwareEvents(TickType_t max_wait_time_ms) {
    if (!hardware_events_group) {
        return false;
    }

    const EventBits_t events = (1U << (EventBits_t)kNbEvents) - 1U;
    latest_events =
        xEventGroupWaitBits(hardware_events_group, events, pdFALSE, pdFALSE, pdMS_TO_TICKS(max_wait_time_ms));

    return (latest_events != 0U);
}

/**
 * Clear all the triggered hardware event flags
 */
void clearHardwareEvents(void) { (void)xEventGroupClearBits(hardware_events_group, latest_events); }

/**
 * Get the bitmask corresponding to a hardware event
 *
 * @param event Event for which get the bitmask
 * @return Bitmask
 */
static inline EventBits_t eventToBitmask(Event event) { return (1U << (EventBits_t)event); }
