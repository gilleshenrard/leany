/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HARDWARE_SENSOR_BMI270
#define HARDWARE_SENSOR_BMI270
#include <main.h>
#include "sensorfusion.h"

void    bmi270InterruptTriggered(uint8_t interruptPin);
void    createBMI270Task(void);
int16_t getAngleDegreesTenths(axis_e axis);
uint8_t anglesChanged(void);
void    bmi270ZeroDown(void);
void    bmi270CancelZeroing(void);
#endif
