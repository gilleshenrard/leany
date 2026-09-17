/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file mahony.h
 * @author Gilles Henrard
 */
#ifndef APP_SENSOR_FUSION_MAHONY_H
#define APP_SENSOR_FUSION_MAHONY_H
#include <stdint.h>

enum : uint8_t {
    kQuaternionAlignment = 16U,    ///< Memory alignment of the quaternion structure
    kContextAlignment = 128U,      ///< Memory alignment of the mahony context structure
    kTimeDeltaAlignment = 16U,     ///< Memory alignment of the time delta structure
    kSampleStructAlignment = 32U,  ///< Memory alignment of the IMU sample structure
    kMahonyStateAlignment = 32U,   ///< Memory alignment of the IMU sample structure
};

/**
 * Enumeration of the axis of which to get measurements
 */
typedef enum : uint8_t {
    kXaxis = 0,  ///< X axis
    kYaxis,      ///< Y axis
    kZaxis,      ///< Z axis
    kNBaxis      ///< Number of axis
} Axis;

/**
 * Reason why the filter was reset
 */
typedef enum : uint8_t {
    kNone = 0,              ///< The filter did not reset
    kDTinvalid = 1,         ///< Took too much/not enough time between valid updates
    kQuaternionNanInf = 2,  ///< The final quaternion norm was NaN or Inf
} ResetCause;

/**
 * Structure defining time delta
 */
typedef struct {
    uint32_t last_sampled_tick;  ///< IMU internal tick at which the last sample was taken
    uint32_t last_valid_tick;    ///< IMU internal tick at which the filter was successfully updated
    uint32_t max_tick;           ///< Maximum sensor tick value
    float tick_period_seconds;   ///< Tick resolution in [s]
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

typedef struct {
    float error_integrals[kNBaxis];  ///< Array containing the integrated errors
    float weighed_kp;                ///< PI filter proportional gain used in the filter after trust-weighing
    float weighed_ki;                ///< PI filter integral gain used in the filter after trust-weighing
    float trust_weight;              ///< Weight used as a trust level on kP and kI
    ResetCause last_reset_cause;     ///< Last cause for the filter to reset
    float norm_abs_deviation;        ///< Absolute value of the norm's current deviation from 1
    bool manual_pure_gyro;           ///< Whether to manually disable acceleration-based error correction
} __attribute__((aligned(kMahonyStateAlignment))) MahonyState;

/**
 * Structure defining a mahony filter context
 */
typedef struct {
    MahonyState state;           ///< Filter state variables
    Quaternion attitude;         ///< Current attitude quaternion
    TimeDelta dt;                ///< Time delta between updates
    float base_kp;               ///< PI filter proportional gain used as a base value before weighing
    float base_ki;               ///< PI filter integral gain used as a base value before weighing
    float min_alignment_cosine;  ///< Minimum accepted cosine of the angle between estimated vector and actual accel.
    float max_norm_epsilon;      ///< Maximum deviation of a norm around 1
    float min_kp_trust_factor;   ///< Minimum trust level of kP
} __attribute__((aligned(kContextAlignment))) MahonyContext;

/* Sample struct */
typedef struct {
    float accelerometer_g[kNBaxis];  ///< Accelerometer measurements in [G] (9.81 m/s²)
    float gyroscope_radps[kNBaxis];  ///< Gyroscope measurements in [rad/s]
    uint32_t imu_tick;               ///< IMU internal tick at which the sample was taken
} __attribute((aligned(kSampleStructAlignment))) IMUsample;

static constexpr float kProportionalGain = 25.0F;  ///< Propotional gain (KP) of the Mahony filter
static constexpr float kIntegralGain = 5.0F;       ///< Integral gain (KI) of the Mahony filter
static constexpr float kMaxNormEpsilon = 0.15F;    ///< Maximum norm deviation of 15%
static constexpr float kMinKpTrustFactor = 0.2F;   ///< Minimum kP trust of 20%
static constexpr float kMinAlignCosine = 0.9659F;  ///< Min vectors alignment cosine of 15°

void resetMahonyFilter(MahonyContext* context);
bool updateMahonyFilter(MahonyContext* context, const IMUsample* sample);
float angleAlongAxis(const Quaternion* attitude, Axis axis);
float getAttitudeAngle(const Quaternion* attitude);
float linearInterpolation(float raw_value, float min_raw, float min_output, float max_raw, float max_output);

#endif
