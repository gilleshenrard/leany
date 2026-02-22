/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file task_battery.h
 * @author Gilles Henrard
 */
#ifndef HARDWARE_BATTERY_BATTERY_H
#define HARDWARE_BATTERY_BATTERY_H
#include "errorstack.h"

/**
 * Status of a battery
 */
typedef struct {
    uint8_t level_percents;  ///< Battery level in [%]
    uint8_t charging;        ///< Flag indicating whether the battery is charging
} BatteryStatus;

void chargerInterruptTriggered(void);
ErrorCode createBatteryTask(void);
ErrorCode getBatteryStatus(BatteryStatus* status);
uint8_t setBatteryPercentage(uint8_t percentage);
void setBatteryChargeStatus(uint8_t status);
#endif
