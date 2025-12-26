/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TASKS_TASK_IMU_H
#define TASKS_TASK_IMU_H
#include <stdint.h>

#include "errorstack.h"
#include "orientation.inc"
#include "sensorfusion.h"

// IMU functions
void createIMUtask(void);
void IMUinterruptTriggered(uint8_t interrupt_pin);
int16_t getAngleDegreesTenths(Axis axis);
uint8_t anglesChanged(void);
void IMUzeroDown(void);
void IMUcancelZeroing(void);
uint8_t isIMUzeroed(void);
float getIMU_KP(void);
float getIMU_KI(void);
uint8_t isIMUalignmentCheckEnabled(void);
void setIMU_KI(float value);
void setIMU_KP(float value);
void setIMUalignmentCheckEnabled(uint8_t value);
uint8_t toggleIMU_hold(void);
uint8_t isIMUmeasurementsHolding(void);
ErrorCode setDisplayOrientation(Orientation new_orientation);
ErrorCode getDisplayOrientation(Orientation* orientation);

#endif
