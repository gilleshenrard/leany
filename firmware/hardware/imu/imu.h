/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_IMU_IMU_H
#define HARDWARE_IMU_IMU_H
#include "errorstack.h"
#include "sensorfusion.h"

/****************************************************************************************************************/
/****************************************************************************************************************/
// Generic API prototypes all IMUs must implement

/**
 * Check whether SPI works and the correct device ID can be retrieved
 *
 * @return IMU-specific error code, 0 on success
 */
extern ErrorCode IMUcheckDeviceID(void);

/**
 * Request an IMU soft reset
 *
 * @return IMU-specific error code, 0 on success
 */
extern ErrorCode IMUsoftReset(void);

/**
 * Process IMU-specific initialisation steps before configuration
 *
 * @return IMU-specific error code, 0 on success
 */
extern ErrorCode IMUinitialise(void);

/**
 * Write the configuration registers of the IMU
 *
 * @return IMU-specific error code, 0 on success
 */
extern ErrorCode IMUconfigure(void);

/**
 * @fn ErrorCode IMUgetSample(IMUsample* sample)
 *
 * Get the latest IMU accelerations, angular rates, temperature,
 * and sensor tick sample.
 *
 * @param[out] sample Latest sample to retrieve
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
 * @return Self-test error code, 0 on success
 */
extern ErrorCode selfTestAccelerometer(void);

/**
 * Run the gyroscope self-test procedure
 *
 * @return Self-test error code, 0 on success
 */
extern ErrorCode selfTestGyroscope(void);

/**
 * Get the number of samples to ignore after configuration
 *
 * @return Number of samples to ignore 
 */
extern uint8_t getNbSamplesToIgnore(void);

#endif
