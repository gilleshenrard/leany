/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HARDWARE_IMU_IMU_H
#define HARDWARE_IMU_IMU_H
#include <stdint.h>

#include "errorstack.h"
#include "sensorfusion.h"

// IMU functions
void createIMUtask(void);
void IMUinterruptTriggered(uint8_t interrupt_pin);
int16_t getAngleDegreesTenths(Axis axis);
uint8_t anglesChanged(void);
void IMUzeroDown(void);
void IMUcancelZeroing(void);
float getIMU_KP(void);
float getIMU_KI(void);
uint8_t isIMUalignmentCheckEnabled(void);
void setIMU_KI(float value);
void setIMU_KP(float value);
void setIMUalignmentCheckEnabled(uint8_t value);

/****************************************************************************************************************/
/****************************************************************************************************************/
// Generic API prototypes all IMUs must implement

/**
 * Check whether SPI works and the correct device ID can be retrieved
 *
 * @retval 0 Success
 * @return IMU-specific error code
 */
extern ErrorCode IMUcheckDeviceID(void);

/**
 * Request an IMU soft reset
 *
 * @retval 0 Success
 * @return IMU-specific error code
 */
extern ErrorCode IMUsoftReset(void);

/**
 * Process IMU-specific initialisation steps before configuration
 *
 * @retval 0 Success
 * @return IMU-specific error code
 */
extern ErrorCode IMUinitialise(void);

/**
 * Write the configuration registers of the IMU
 *
 * @retval 0 Success
 * @return IMU-specific error code
 */
extern ErrorCode IMUconfigure(void);

/**
 * Get the latest IMU accelerations/angle rates/temperature/sensor tick sample
 *
 * @param[out] sample Latest sample to retrieve
 * @retval 0 Success
 * @return IMU-specific error code
 */
extern ErrorCode IMUgetSample(IMUsample* sample);

/**
 * Initialise the time fields of the Mahony filter's context
 *
 * @param[out] filter_context Mahony filter's current context
 */
extern void IMUsetupTimebase(MahonyContext* filter_context);

/**
 * Run the accelerometer self-test procedure
 *
 * @retval 0 Success
 * @return Self-test error code
 */
extern ErrorCode selfTestAccelerometer(void);

/**
 * Run the gyroscope self-test procedure
 *
 * @retval 0 Success
 * @return Self-test error code
 */
extern ErrorCode selfTestGyroscope(void);

/**
 * Get the number of samples to ignore after configuration
 *
 * @return Number of samples to ignore 
 */
extern uint8_t getNbSamplesToIgnore(void);

#endif
