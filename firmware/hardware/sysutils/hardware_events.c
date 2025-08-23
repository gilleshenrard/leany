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

static EventGroupHandle_t hardware_events_group = (void*)0;  ///< Hardware events group handle
static EventBits_t latest_events = 0;                        ///< Latest hardware events monitored

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

    (void)xEventGroupSetBits(hardware_events_group, (1U << event));
}

/**
 * Check if a hardware event has been triggered
 *
 * @param event Event to check 
 * @retval 1 Event triggered
 * @retval 0 Event not triggered
 */
uint8_t isHardwareEventTriggered(Event event) { return (latest_events & (1U << event)) != 0; }

/**
 * Wait for a hardware event to occur
 *
 * @param max_wait_time_ms  Maximum number of milliseconds to wait for an event
 * @retval 1 New events triggered
 * @retval 0 No new event triggered
 */
uint8_t waitForHardwareEvents(TickType_t max_wait_time_ms) {
    if (!hardware_events_group) {
        return 0;
    }

    const EventBits_t events = (1U << (EventBits_t)kNbEvents) - 1U;
    const EventBits_t current_bits = xEventGroupGetBits(hardware_events_group);
    latest_events =
        xEventGroupWaitBits(hardware_events_group, events, pdTRUE, pdFALSE, pdMS_TO_TICKS(max_wait_time_ms));

    return (latest_events != current_bits);
}
