/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HARDWARE_SYSUTILS_HARDWARE_EVENTS_H
#define HARDWARE_SYSUTILS_HARDWARE_EVENTS_H
#include <FreeRTOS.h>
#include <event_groups.h>

typedef enum {
    kEventXValue = 1U << 0U,      ///< Event : X value update
    kEventYValue = 1U << 1U,      ///< Event : Y value update
    kEventZero = 1U << 2U,        ///< Event : Zero down measurements
    kEventCancelZero = 1U << 3U,  ///< Event : Cancel measurements zeroing
} Event;

void createHardwareEventsGroup(void);
void triggerHardwareEvent(EventBits_t events);
EventBits_t waitForHardwareEvents(EventBits_t events, TickType_t max_wait_time_ms);
#endif
