/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HARDWARE_SYSUTILS_HARDWARE_EVENTS_H
#define HARDWARE_SYSUTILS_HARDWARE_EVENTS_H
#include <FreeRTOS.h>
#include <portmacro.h>
#include <stdint.h>

// NOLINTBEGIN (readability-enum-initial-value)
typedef enum {
    kEventAngle = 0U,          ///< Event : Angle value change
    kEventZero = 1U,           ///< Event : Zero down measurements
    kEventCancelZero = 2U,     ///< Event : Cancel measurements zeroing
    kEventHold = 3U,           ///< Event : Enable measurements holding
    kEventSerialCommand = 4U,  ///< Event : Command received over serial connection
    kEventOrientation = 5U,    ///< Event : Display orientation changed
    kEventTemperature = 6U,    ///< Event : MCU internal temperature changed
    kEventToggleScreen = 7U,   ///< Event : Toggle screen requested
    kEventBatteryStatus = 8U,  ///< Event : Battery status update
    kEventErrorCode = 9U,      ///< Event : Error detected
    kNbEvents,                 ///< Number of events handled
} Event;
// NOLINTEND (readability-enum-initial-value)

void createHardwareEventsGroup(void);
void triggerHardwareEvent(Event event);
uint8_t isHardwareEventTriggered(Event event);
uint8_t waitForHardwareEvents(TickType_t max_wait_time_ms);
void clearHardwareEvents(void);
#endif
