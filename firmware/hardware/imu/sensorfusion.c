/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file sensorfusion.c
 * @brief Mahony filter implementation for 6DoF attitude estimation using gyroscope and accelerometer.
 *
 * @details
 * This file implements a simplified Mahony filter to estimate orientation using gyroscope and
 * accelerometer data. It uses a quaternion representation of attitude to avoid gimbal lock
 * and ensure numerical stability.
 *
 * The Mahony filter operates by:
 * 1. Estimating the direction of gravity using the current quaternion.
 * 2. Comparing it to the gravity direction measured by the accelerometer.
 * 3. Computing the error between these two directions using a cross-product.
 * 4. Feeding this error through a PI (proportional-integral) controller.
 * 5. Applying the resulting correction to the raw gyroscope measurements.
 * 6. Integrating the corrected angular rate into the quaternion over time.
 *
 * It is lightweight and suitable for systems without a magnetometer, where heading drift
 * is acceptable or can be corrected by other means.
 *
 * ## Basic Usage
 *
 * ```c
 * MahonyContext filterContext;
 *
 * // Set the quaternion to a unit orientation (no rotation) and clear the integral terms
 * resetMahonyFilter(&filterContext);
 *
 * while (1) {
 *     IMUsample sample;
 *     readIMU(&sample);  // User-provided sensor read
 *
 *     // Update the orientation quaternion, the integral error terms and update timestamps
 *     updateMahonyFilter(&filterContext, &sample);
 * }
 * ```
 *
 * ## References
 * - Mahony, R., Hamel, T., & Pflimlin, J.-M. (2008). Nonlinear Complementary Filters on the Special
 *   Orthogonal Group. *IEEE Transactions on Automatic Control*, 53(5), 1203–1218.
 *   DOI: [10.1109/TAC.2008.923738](https://doi.org/10.1109/TAC.2008.923738)
 *
 * @author Gilles Henrard
 */
#include "sensorfusion.h"

#include <math.h>
#include <stdint.h>

#define FORCE_INLINE_SILENT __attribute((always_inline))  ///< Macro used to workaround Doxygen issues with __attribute
#if defined(__DOXYGEN__) && __DOXYGEN__ == 1
#define FORCE_INLINE_SILENT
#endif

// utility functions
static inline FORCE_INLINE_SILENT float half(float number);
static inline FORCE_INLINE_SILENT float twice(float number);
static inline FORCE_INLINE_SILENT float squared(float number);
static inline FORCE_INLINE_SILENT float normaliseArray(float array[3]);
static inline FORCE_INLINE_SILENT float normaliseQuaternion(Quaternion* quaternion);
static inline FORCE_INLINE_SILENT float clamp(float value, float max_absolute_value);
static inline FORCE_INLINE_SILENT float getDT(const TimeDelta* delta);
static inline FORCE_INLINE_SILENT uint8_t isDTvalid(float delta_seconds);
static uint8_t alignmentValid(const float accelerometer_normalised[kNBaxis], const float estimates_normalised[kNBaxis]);
static void computeGravityError(float errors[kNBaxis], const float accelerometer_g[kNBaxis],
                                const float body_estimates[kNBaxis]);
static void integrateGyroMeasurements(Quaternion* current_attitude, const float corrected_gyro[kNBaxis],
                                      float period_sec);
static uint8_t validateNorm(MahonyContext* context, float norm, uint8_t* bad_norm_counter);
static void computeEstimates(const MahonyContext* context, float body_estimates[kNBaxis]);
static void applyProportionate(const MahonyContext* context, float corrected_gyro_radps[kNBaxis],
                               const IMUsample* sample, const float errors[kNBaxis]);

//constants
static const float kCloseToZero = 1e-3F;            ///< Value used to compare floats to 0
static const float kMinAlignmentCosine = 0.9659F;   ///< cosine value for 15°, used as a maximum alignment angle
static const float kMaxAlignmentCosine = 1.00001F;  ///< maximum alignment angle cosine acceptable
static const float kMaxNormEpsilon = 0.15F;         ///< Maximum deviation of a norm around 1
static const float kMaxIntegralError = 0.3F;        ///< Maximum integral error absolute value accepted
static const float kMinValidDTseconds = 1e-6F;      ///< Minimum acceptable timespan between updates
static const float kMaxValidDTseconds = 0.5F;       ///< Maximum acceptable timespan between updates
static const uint8_t kMaxBadCounts = 5U;            ///< Maximum number of bad accel. or quatern. counts before reset

/*********************************************************************************************************************************/
// Mahony filter's publicly accessible functions
/*********************************************************************************************************************************/

/**
 * Reset the quaternion and integral values used by the Mahony filter
 *
 * @param[out] context Current Mahony filter context
 */
void resetMahonyFilter(MahonyContext* context) {
    //if no context provided, exit
    if (!context) {
        return;
    }

    context->attitude = (Quaternion){.q0 = 1.0F, .q1 = 0.0F, .q2 = 0.0F, .q3 = 0.0F};
    context->error_integrals[kXaxis] = 0.0F;
    context->error_integrals[kYaxis] = 0.0F;
    context->error_integrals[kZaxis] = 0.0F;
    context->bad_acceleration_count = 0;
    context->bad_quaternion_count = 0;
    context->dt.previous_tick = context->dt.current_tick;
}

/**
 * Update the quaternion, integral values and update timestamps used by the Mahony filter with fresh ones
 *
 * @param[out] context  Current Mahony filter context
 * @param[in] sample    Latest IMU sample measured
 */
void updateMahonyFilter(MahonyContext* context, const IMUsample* sample) {
    //if no pointer provided, exit
    if (!context || !sample) {
        return;
    }

    //normalise accelerometer vectors to unit length, to avoid drift
    float normalised_accelerometer[kNBaxis] = {sample->accelerometer_g[0], sample->accelerometer_g[1],
                                               sample->accelerometer_g[2]};
    const float acceleration_norm = normaliseArray(normalised_accelerometer);
    if (!validateNorm(context, acceleration_norm, &context->bad_acceleration_count)) {
        return;
    }

    //if dT out of reasonable bounds, reset the filter
    const float period_sec = getDT(&context->dt);
    if (!isDTvalid(period_sec)) {
        resetMahonyFilter(context);
    }

    //estimate the current body frame gravity vectors from the current orientation quaternion
    float body_estimates[kNBaxis];
    computeEstimates(context, body_estimates);

    //Abort update if validation is enabled and a strong linear motion is detected
    if (context->align_check_enabled && !alignmentValid(normalised_accelerometer, body_estimates)) {
        return;
    }

    //compute the error rotation vectors, which will be used to realign the estimations to the measured vectors
    float errors[kNBaxis] = {0.0F, 0.0F, 0.0F};
    computeGravityError(errors, normalised_accelerometer, body_estimates);

    //apply the proportion term to error vectors and add them to the gyroscope measurements
    float corrected_gyro_radps[kNBaxis];
    applyProportionate(context, corrected_gyro_radps, sample, errors);

    //apply the clamped integral term to error vectors and add them to the gyroscope measurements
    // (avoid if gain is 0 to avoid integral windup due to float approximate 0.0F)
    uint8_t axis = 0;
    while ((context->ki > kCloseToZero) && (axis < kNBaxis)) {
        context->error_integrals[axis] += (context->ki * errors[axis] * period_sec);
        context->error_integrals[axis] = clamp(context->error_integrals[axis], kMaxIntegralError);
        corrected_gyro_radps[axis] += context->error_integrals[axis];
        axis++;
    }

    //integrate the corrected gyroscope data into the current attitude quaternion
    integrateGyroMeasurements(&context->attitude, corrected_gyro_radps, period_sec);

    //normalise the current attitude quaternion to avoid drift
    const float quaterion_norm = normaliseQuaternion(&context->attitude);
    if (!validateNorm(context, quaterion_norm, &context->bad_quaternion_count)) {
        return;
    }

    //update the last update tick on success
    context->dt.previous_tick = context->dt.current_tick;
}

/**
 * Get the current angle in [rad] along an axis
 * @note Yaw angle (around the Z axis) will always return 0, due to the absence of a magnetometer
 *
 * @param context Current Mahony filter context
 * @param axis    Axis along which getting the angle
 * @return Angle in [rad] if X or Y axis requested, 0 otherwise
 */
float angleAlongAxis(const MahonyContext* context, Axis axis) {
    float raw_sine = 0.0F;

    //if no context provided, exit
    if (!context) {
        return 0.0F;
    }

    switch (axis) {
        case kXaxis:  //roll
            return atan2f(
                (twice((context->attitude.q0 * context->attitude.q1) + (context->attitude.q2 * context->attitude.q3))),
                1.0F - (twice((context->attitude.q1 * context->attitude.q1) +
                              (context->attitude.q2 * context->attitude.q2))));

        case kYaxis:  //pitch
            raw_sine =
                twice((context->attitude.q1 * context->attitude.q3) - (context->attitude.q0 * context->attitude.q2));
            return asinf(clamp(raw_sine, 1.0F));  //make sure to clamp the sin value between [-1, 1]

        case kZaxis:
        case kNBaxis:
        default:
            return 0.0F;
    };
}

/**
 * Get the angle in [rad] along the current quaternion attitude axis
 *
 * @param context Current Mahony filter context
 * @return Angle in [rad]
 */
float getAttitudeAngle(const MahonyContext* context) {
    //if no context provided, exit
    if (!context) {
        return 0.0F;
    }

    const float safe_cosine = clamp(context->attitude.q0, 1.0F);  //make sure to clamp the cos value between [-1, 1]
    return twice(acosf(safe_cosine));
}

/*********************************************************************************************************************************/
// Internal functions
/*********************************************************************************************************************************/

/**
 * Compute the half-value of a number
 * @details Mainly used for readability purposes
 * 
 * @param number Number to half
 * @return Half the value of number
 */
static inline FORCE_INLINE_SILENT float half(const float number) {
    return 0.5F * number;  // NOLINT(*-magic-numbers)
}

/**
 * Compute the double of a number
 * @details Mainly used for readability purposes
 * 
 * @param number Number to double
 * @return Double the value of number
 */
static inline FORCE_INLINE_SILENT float twice(const float number) {
    return 2.0F * number;  // NOLINT(*-magic-numbers)
}

/**
 * Compute the squared value of a number
 * @details Mainly used for readability purposes
 * 
 * @param number Number to square
 * @return Number squared
 */
static inline FORCE_INLINE_SILENT float squared(const float number) { return number * number; }

/**
 * Normalise an array of vectors
 *
 * @param array Array to normalise
 * @return Norm value
 */
static inline FORCE_INLINE_SILENT float normaliseArray(float array[3]) {
    const float norm = sqrtf(squared(array[0U]) + squared(array[1U]) + squared(array[2U]));
    if (norm < kCloseToZero) {
        return norm;
    }

    array[kXaxis] /= norm;
    array[kYaxis] /= norm;
    array[kZaxis] /= norm;
    return norm;
}

/**
 * Normalise a quaternion
 *
 * @param quaternion Quaternion to normalise
 * @return Norm value
 */
static inline FORCE_INLINE_SILENT float normaliseQuaternion(Quaternion* quaternion) {
    float norm =
        sqrtf(squared(quaternion->q0) + squared(quaternion->q1) + squared(quaternion->q2) + squared(quaternion->q3));
    if (norm < kCloseToZero) {
        return norm;
    }

    quaternion->q0 /= norm;
    quaternion->q1 /= norm;
    quaternion->q2 /= norm;
    quaternion->q3 /= norm;
    return norm;
}

/**
 * Clamp a value to the symmetric range [-max_absolute_value, +max_absolute_value]
 *
 * @param value Value to clamp
 * @param max_absolute_value Absolute maximum magnitude of the output
 * @return Clamped value
 */
static inline FORCE_INLINE_SILENT float clamp(const float value, const float max_absolute_value) {
    return fmaxf(-max_absolute_value, fminf(max_absolute_value, value));
}

/**
 * Compute the elapsed time in [s] between current and previous timestamps
 *
 * @param delta Time delta structure
 * @return time delta in [s]
 */
static inline FORCE_INLINE_SILENT float getDT(const TimeDelta* delta) {
    //compute the time delta and avoid issues with the overflow after maxTick
    const uint32_t delta_ticks = (delta->current_tick - delta->previous_tick) & delta->max_tick;
    return ((float)delta_ticks * delta->tick_period_seconds);
}

/**
 * Check whether a time delta is between 1us and 0.5s
 *
 * @param delta_seconds Time delta in [s]
 * @retval 0 Time delta is out of bounds
 * @retval 1 Time delta is within bounds
 */
static inline FORCE_INLINE_SILENT uint8_t isDTvalid(float delta_seconds) {
    return ((delta_seconds > kMinValidDTseconds) && (delta_seconds < kMaxValidDTseconds));
}

/**
 * Check if the linear acceleration vector measured in the 3D space aligns well enough with the estimates vector
 *
 * @param accelerometer_normalised Accelerometer vectors, normalised to unit length
 * @param estimates_normalised Estimates vectors, normalised to unit length
 * @retval 1 Estimates are close enough to the accelerometer measurements
 * @retval 0 The angle between vectors is too wide (sign of large linear acceleration)
 */
static uint8_t alignmentValid(const float accelerometer_normalised[kNBaxis],
                              const float estimates_normalised[kNBaxis]) {
    //This is done with the means of a dot product between vectors.
    //As the vectors are normalised, their dot product gives the cosine of the angle between them.
    const float dot_product = (accelerometer_normalised[kXaxis] * estimates_normalised[kXaxis]) +
                              (accelerometer_normalised[kYaxis] * estimates_normalised[kYaxis]) +
                              (accelerometer_normalised[kZaxis] * estimates_normalised[kZaxis]);

    return ((dot_product >= kMinAlignmentCosine) && (dot_product <= kMaxAlignmentCosine));
}

/**
 * Compute the 3D error vector between measured and estimated gravity.
 * @details Uses the cross product of the normalized accelerometer vector and the estimated gravity vector (from quaternion)
 * to compute the direction and magnitude of the orientation error. This error is used to correct the gyroscope bias.
 *
 * @param accelerometer_g Normalized accelerometer reading (measured gravity direction in [G] (9.81 m/s²)).
 * @param body_estimates Estimated gravity direction derived from current orientation quaternion.
 * @param[out] errors Error orientation vector (body frame).
 */
static void computeGravityError(float errors[kNBaxis], const float accelerometer_g[kNBaxis],
                                const float body_estimates[kNBaxis]) {
    errors[kXaxis] =
        ((accelerometer_g[kYaxis] * body_estimates[kZaxis]) - (accelerometer_g[kZaxis] * body_estimates[kYaxis]));
    errors[kYaxis] =
        ((accelerometer_g[kZaxis] * body_estimates[kXaxis]) - (accelerometer_g[kXaxis] * body_estimates[kZaxis]));
    errors[kZaxis] =
        ((accelerometer_g[kXaxis] * body_estimates[kYaxis]) - (accelerometer_g[kYaxis] * body_estimates[kXaxis]));
}

/**
 * Integrate corrected gyroscope measurements and apply them to the current attitude quaternion
 *
 * @param current_attitude Current attitude quaternion to update
 * @param corrected_gyro Gyroscope measurements, corrected with the PI filter and error rotation vectors
 * @param period_sec Period between now and the last update, in [s]
 */
static void integrateGyroMeasurements(Quaternion* current_attitude, const float corrected_gyro[kNBaxis],
                                      float period_sec) {
    //compute the derivative quaternion, composed of the current attitude and the gyroscope measurements

    const float q0 = current_attitude->q0;  //NOLINT(readability-identifier-length)
    const float q1 = current_attitude->q1;  //NOLINT(readability-identifier-length)
    const float q2 = current_attitude->q2;  //NOLINT(readability-identifier-length)
    const float q3 = current_attitude->q3;  //NOLINT(readability-identifier-length)

    const Quaternion rate_of_change = {
        .q0 = half((-q1 * corrected_gyro[kXaxis]) - (q2 * corrected_gyro[kYaxis]) - (q3 * corrected_gyro[kZaxis])),
        .q1 = half((q0 * corrected_gyro[kXaxis]) + (q2 * corrected_gyro[kZaxis]) - (q3 * corrected_gyro[kYaxis])),
        .q2 = half((q0 * corrected_gyro[kYaxis]) - (q1 * corrected_gyro[kZaxis]) + (q3 * corrected_gyro[kXaxis])),
        .q3 = half((q0 * corrected_gyro[kZaxis]) + (q1 * corrected_gyro[kYaxis]) - (q2 * corrected_gyro[kXaxis]))};

    //integrate the current quaternion with the change rate
    current_attitude->q0 += (rate_of_change.q0 * period_sec);
    current_attitude->q1 += (rate_of_change.q1 * period_sec);
    current_attitude->q2 += (rate_of_change.q2 * period_sec);
    current_attitude->q3 += (rate_of_change.q3 * period_sec);
}

/**
 * Check if a norm provided is within a valid range
 * @details An invalid norm increments a counter which, if too high, will trigger a filter reset
 *
 * @param context Filter context
 * @param norm Norm to validate
 * @param bad_norm_counter Counter used to see if the filter should be reseted
 * @retval 1 Norm valid
 * @retval 0 Norm invalid
 */
static uint8_t validateNorm(MahonyContext* context, const float norm, uint8_t* bad_norm_counter) {
    if ((norm < (1.0F - kMaxNormEpsilon)) || (norm > (1.0F + kMaxNormEpsilon))) {
        if (++(*bad_norm_counter) >= kMaxBadCounts) {
            resetMahonyFilter(context);
        }
        return 0;
    }
    *bad_norm_counter = 0;
    return 1;
}

/**
 * Compute body estimates from the current context quaternion
 *
 * @param context Current Mahony filter's context
 * @param[out] body_estimates Array to fill with the estimates
 */
static void computeEstimates(const MahonyContext* context, float body_estimates[kNBaxis]) {
    body_estimates[kXaxis] =
        twice((context->attitude.q1 * context->attitude.q3) - (context->attitude.q0 * context->attitude.q2));
    body_estimates[kYaxis] =
        twice((context->attitude.q0 * context->attitude.q1) + (context->attitude.q2 * context->attitude.q3));
    body_estimates[kZaxis] = squared(context->attitude.q0) - squared(context->attitude.q1) -
                             squared(context->attitude.q2) + squared(context->attitude.q3);
}

/**
 * Apply the filter's PID proportionate member to the current sample gyroscope values
 *
 * @param context Current Mahony filter's context
 * @param[out] corrected_gyro_radps Array to fill with the filtered values
 * @param sample Current sample from which get the gyro values
 * @param errors Error vectors to apply
 */
static void applyProportionate(const MahonyContext* context, float corrected_gyro_radps[kNBaxis],
                               const IMUsample* sample, const float errors[kNBaxis]) {
    corrected_gyro_radps[kXaxis] = (sample->gyroscope_radps[kXaxis] + (context->kp * errors[kXaxis]));
    corrected_gyro_radps[kYaxis] = (sample->gyroscope_radps[kYaxis] + (context->kp * errors[kYaxis]));
    corrected_gyro_radps[kZaxis] = (sample->gyroscope_radps[kZaxis] + (context->kp * errors[kZaxis]));
}
