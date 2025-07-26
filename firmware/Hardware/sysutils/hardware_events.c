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

static EventGroupHandle_t hardware_events_group = (void*)0;  ///< Hardware events group handle

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
 * @param events Bitwise value of the events to trigger
 */
void triggerHardwareEvent(EventBits_t events) {
    if (!hardware_events_group) {
        return;
    }

    (void)xEventGroupSetBits(hardware_events_group, events);
}

/**
 * Wait for a hardware event to occur
 *
 * @param events            Events to wait for
 * @param max_wait_time_ms  Maximum number of milliseconds to wait for an event
 * @return Events triggered
 */
EventBits_t waitForHardwareEvents(EventBits_t events, TickType_t max_wait_time_ms) {
    if (!hardware_events_group) {
        return 0;
    }

    return xEventGroupWaitBits(hardware_events_group, events, pdTRUE, pdFALSE, pdMS_TO_TICKS(max_wait_time_ms));
}
