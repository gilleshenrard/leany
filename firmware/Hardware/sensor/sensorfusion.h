/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HARDWARE_SENSOR_SENSORFUSION_H
#define HARDWARE_SENSOR_SENSORFUSION_H
#include <stdint.h>

enum {
    kQuaternionAlignment = 16U,  ///< Memory alignment of the quaternion structure
    kContextAlignment = 64U,     ///< Memory alignment of the mahony context structure
    kTimeDeltaAlignment = 16U,   ///< Memory alignment of the time delta structure
};

/**
 * Enumeration of the axis of which to get measurements
 */
typedef enum { kXaxis = 0, kYaxis, kZaxis, kNBaxis } Axis;

/**
 * Structure defining time delta
 */
typedef struct {
    uint32_t current_tick;     ///< Current sensor tick value
    uint32_t previous_tick;    ///< Tick value at the last update
    uint32_t max_tick;         ///< Maximum sensor tick value
    float resolution_seconds;  ///< Tick resolution in [s]
} __attribute__((aligned(kTimeDeltaAlignment))) TimeDelta;

/**
 * Structure defining a quaternion
 */
typedef struct {
    float q0;  ///< Scalar value
    float q1;  ///< Value which multiplies the unit vector along the X axis
    float q2;  ///< Value which multiplies the unit vector along the Y axis
    float q3;  ///< Value which multiplies the unit vector along the Z axis
} __attribute__((aligned(kQuaternionAlignment))) Quaternion;

/**
 * Structure defining a mahony filter context
 */
typedef struct {
    Quaternion attitude;             ///< Current attitude quaternion
    TimeDelta dt;                    ///< Time delta between updates
    float error_integrals[kNBaxis];  ///< Array containing the integrated errors
    float kp;                        ///< PI filter proportional gain
    float ki;                        ///< PI filter integral gain
    uint8_t bad_acceleration_count;  ///< Number of bad accelerometer norms since reset
    uint8_t bad_quaternion_count;    ///< Number of bad quaternion norms since reset
} __attribute__((aligned(kContextAlignment))) MahonyContext;

static const float kProportionalGain = 2.5F;  ///< Propotional gain (KP) of the Mahony filter
static const float kIntegralGain = 0.5F;      ///< Integral gain (KI) of the Mahony filter

void resetMahonyFilter(MahonyContext* context);
void updateMahonyFilter(MahonyContext* context, const float accelerometer_g[kNBaxis],
                        const float gyroscope_radps[kNBaxis]);
float angleAlongAxis(const MahonyContext* context, Axis axis);
float getAttitudeAngle(const MahonyContext* context);

#endif
