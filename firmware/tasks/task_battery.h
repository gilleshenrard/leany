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

void chargerInterruptTriggered(void);
ErrorCode createBatteryTask(void);
#endif
