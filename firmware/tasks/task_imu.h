/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file task_imu.h
 * @author Gilles Henrard
 */
#ifndef TASKS_TASK_IMU_H
#define TASKS_TASK_IMU_H
#include <stdint.h>

#include "errorstack.h"
#include "orientation.h"
#include "sensorfusion.h"

// IMU functions
void createIMUtask(void);
void IMUinterruptTriggered(uint8_t interrupt_pin);
int16_t getAngleDegreesTenths(Axis axis);
void IMUzeroDown(void);
void IMUcancelZeroing(void);
bool isIMUzeroed(void);
float getIMU_KP(void);
float getIMU_KI(void);
bool isIMUalignmentCheckEnabled(void);
void setIMU_KI(float value);
void setIMU_KP(float value);
void setIMUalignmentCheckEnabled(bool value);
bool toggleIMU_hold(void);
bool isIMUmeasurementsHolding(void);
ErrorCode setDisplayOrientation(Orientation new_orientation);
ErrorCode getDisplayOrientation(Orientation* orientation);

#endif
