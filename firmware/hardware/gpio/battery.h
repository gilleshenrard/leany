/*
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_GPIO_BATTERY_H
#define HARDWARE_GPIO_BATTERY_H
#include <stdint.h>

#include "errorstack.h"

/**
 * Status of a battery
 */
typedef struct {
    uint8_t level_percents;  ///< Battery level in [%]
    uint8_t charging;        ///< Flag indicating whether the battery is charging
} BatteryStatus;

ErrorCode getBatteryStatus(BatteryStatus* status);
uint8_t setBatteryPercentage(uint8_t percentage);
void setBatteryChargeStatus(uint8_t status);

#endif
