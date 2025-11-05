/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HARDWARE_SYSUTILS_HARDWARE_EVENTS_H
#define HARDWARE_SYSUTILS_HARDWARE_EVENTS_H
#include <FreeRTOS.h>
#include <stdint.h>

// NOLINTBEGIN (readability-enum-initial-value)
typedef enum {
    kEventXValue = 0U,         ///< Event : X value update
    kEventYValue = 1U,         ///< Event : Y value update
    kEventZero = 2U,           ///< Event : Zero down measurements
    kEventCancelZero = 3U,     ///< Event : Cancel measurements zeroing
    kEventHold = 4U,           ///< Event : Enable measurements holding
    kEventSerialCommand = 5U,  ///< Event : Command received over serial connection
    kNbEvents,                 ///< Number of events handled
} Event;
// NOLINTEND (readability-enum-initial-value)

void createHardwareEventsGroup(void);
void triggerHardwareEvent(Event event);
uint8_t isHardwareEventTriggered(Event event);
uint8_t waitForHardwareEvents(TickType_t max_wait_time_ms);
#endif
