/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_BATTERY_BATTERY_H
#define HARDWARE_BATTERY_BATTERY_H
#include "errorstack.h"

void chargerInterruptTriggered(void);
ErrorCode createBatteryTask(void);
#endif
