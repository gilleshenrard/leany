/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file sensorfusion.h
 * @author Gilles Henrard
 */
#ifndef HARDWARE_SENSOR_SENSORFUSION_H
#define HARDWARE_SENSOR_SENSORFUSION_H
#include <stdint.h>

enum {
    kQuaternionAlignment = 16U,    ///< Memory alignment of the quaternion structure
    kContextAlignment = 64U,       ///< Memory alignment of the mahony context structure
    kTimeDeltaAlignment = 16U,     ///< Memory alignment of the time delta structure
    kSampleStructAlignment = 32U,  ///< Memory alignment of the IMU sample structure
};

/**
 * Enumeration of the axis of which to get measurements
 */
typedef enum {
    kXaxis = 0,  ///< X axis
    kYaxis,      ///< Y axis
    kZaxis,      ///< Z axis
    kNBaxis      ///< Number of axis
} Axis;

/**
 * Structure defining time delta
 */
typedef struct {
    uint32_t current_tick;      ///< Current sensor tick value
    uint32_t previous_tick;     ///< Tick value at the last update
    uint32_t max_tick;          ///< Maximum sensor tick value
    float tick_period_seconds;  ///< Tick resolution in [s]
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
    uint8_t align_check_enabled;     ///< Check valid alignment between estimates and accelerometer?
    uint8_t bad_acceleration_count;  ///< Number of bad accelerometer norms since reset
    uint8_t bad_quaternion_count;    ///< Number of bad quaternion norms since reset
} __attribute__((aligned(kContextAlignment))) MahonyContext;

/* Sample struct */
typedef struct {
    float accelerometer_g[kNBaxis];  ///< Accelerometer measurements in [G] (9.81 m/s²)
    float gyroscope_radps[kNBaxis];  ///< Gyroscope measurements in [rad/s]
    uint32_t latest_tick;            ///< Latest sensor tick value
} __attribute((aligned(kSampleStructAlignment))) IMUsample;

static const float kProportionalGain = 2.5F;  ///< Propotional gain (KP) of the Mahony filter
static const float kIntegralGain = 0.5F;      ///< Integral gain (KI) of the Mahony filter

void resetMahonyFilter(MahonyContext* context);
void updateMahonyFilter(MahonyContext* context, const IMUsample* sample);
float angleAlongAxis(const MahonyContext* context, Axis axis);
float getAttitudeAngle(const MahonyContext* context);

#endif
