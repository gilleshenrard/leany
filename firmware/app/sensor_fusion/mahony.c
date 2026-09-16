/**
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 *
 * @file mahony.c
 * @brief Mahony filter implementation for 6DoF attitude estimation using gyroscope and accelerometer.
 * @author Gilles Henrard
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
 * - R. Mahony, T. Hamel and J. -M. Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal Group,"
 *
 *   IEEE Transactions on Automatic Control, vol. 53, no. 5, pp. 1203-1218, June 2008
 *
 *   DOI: [10.1109/TAC.2008.923738](https://ieeexplore.ieee.org/document/4608934)
 */
#include "mahony.h"

#include <math.h>
#include <stdint.h>

// macros
#define FORCE_INLINE_SILENT __attribute((always_inline))  ///< Macro used to workaround Doxygen issues with __attribute

// utility functions
static inline FORCE_INLINE_SILENT float half(float number);
static inline FORCE_INLINE_SILENT float twice(float number);
static inline FORCE_INLINE_SILENT float squared(float number);
static inline FORCE_INLINE_SILENT float normaliseArray(float array[kNBaxis]);
static inline FORCE_INLINE_SILENT float normaliseQuaternion(Quaternion* quaternion);
static inline FORCE_INLINE_SILENT float clamp_absolute(float value, float max_absolute_value);
static inline FORCE_INLINE_SILENT float clamp_min_max(float value, float min_value, float max_value);
static inline FORCE_INLINE_SILENT float absoluteValue(float value);
static inline FORCE_INLINE_SILENT float computeDTseconds(const TimeDelta* delta);
static inline FORCE_INLINE_SILENT uint8_t isDTvalid(float delta_seconds);
static bool alignmentValid(const float accelerometer_normalised[kNBaxis], const float estimates_normalised[kNBaxis]);
static void computeGravityError(float errors[kNBaxis], const float accelerometer_g[kNBaxis],
                                const float body_estimates[kNBaxis]);
static void integrateGyroQuaternion(Quaternion* current_attitude, const float corrected_gyro[kNBaxis],
                                    float timedelta_seconds);
static bool normValid(float norm);
static void estimateOrientation(const Quaternion* attitude, float body_estimates[kNBaxis]);
static void applyProportionateErrors(float corrected_gyro_radps[kNBaxis], const IMUsample* sample,
                                     const float errors[kNBaxis], float trusted_kp);
static void accumulateIntegralErrors(float error_integrals[kNBaxis], const float errors[kNBaxis],
                                     float timedelta_seconds, float trusted_ki);
static void applyIntegralErrors(const float error_integrals[kNBaxis], float corrected_gyro_radps[kNBaxis]);
static float getNormalisedVectorsAngleCosine(const float normalised_first[kNBaxis],
                                             const float normalised_second[kNBaxis]);
static float linearInterpolation(float raw_value, float min_raw, float min_output, float max_raw, float max_output);
static void applyTrustToCoefficients(MahonyContext* context, const float accelerometer_normalised[kNBaxis],
                                     const float estimates_normalised[kNBaxis], float acceleration_norm);

//constants
static constexpr float kCloseToZero = 1e-3F;            ///< Value used to compare floats to 0
static constexpr float kMinAlignmentCosine = 0.9659F;   ///< cosine value for 15°, used as a maximum alignment angle
static constexpr float kMaxAlignmentCosine = 1.00001F;  ///< maximum alignment angle cosine acceptable
static constexpr float kMaxNormEpsilon = 0.15F;         ///< Maximum deviation of a norm around 1
static constexpr float kMinValidDTseconds = 1e-6F;      ///< Minimum acceptable timespan between updates
static constexpr float kMaxValidDTseconds = 0.5F;       ///< Maximum acceptable timespan between updates
static constexpr float kMinKpTrustFraction = 0.2F;      ///< Minimum trust level of kP

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
    context->weighed_kp = 0.0F;
    context->weighed_ki = 0.0F;
    context->trust_weight = 0.0F;
    context->dt.last_valid_tick = context->dt.last_sampled_tick;
    context->last_reset_cause = kNone;
}

/**
 * Update the quaternion, integral values and update timestamps used by the Mahony filter with fresh ones
 *
 * @param[out] context  Current Mahony filter context
 * @param[in] sample    Latest IMU sample measured
 * @retval true Filter updated
 * @retval false Filter not updated due to error
 */
bool updateMahonyFilter(MahonyContext* context, const IMUsample* sample) {
    //if no pointer provided, exit
    if (!context || !sample) {
        return false;
    }

    //if dT out of reasonable bounds, reset the filter
    const float timedelta_seconds = computeDTseconds(&context->dt);
    if (!isDTvalid(timedelta_seconds)) {
        resetMahonyFilter(context);
        context->last_reset_cause = kDTinvalid;
        return true;
    }

    //normalise accelerometer vectors to unit length, to avoid drift
    float normalised_accelerometer[kNBaxis] = {[kXaxis] = sample->accelerometer_g[kXaxis],
                                               [kYaxis] = sample->accelerometer_g[kYaxis],
                                               [kZaxis] = sample->accelerometer_g[kZaxis]};
    const float acceleration_norm = normaliseArray(normalised_accelerometer);
    if (!normValid(acceleration_norm)) {
        return false;
    }

    //estimate the current body frame gravity vectors from the current orientation quaternion
    float body_estimates[kNBaxis];
    estimateOrientation(&context->attitude, body_estimates);

    //Abort update if validation is enabled and a strong linear motion is detected
    if (context->alignment_check_enabled && !alignmentValid(normalised_accelerometer, body_estimates)) {
        return false;
    }

    applyTrustToCoefficients(context, normalised_accelerometer, body_estimates, acceleration_norm);

    //compute the error rotation vectors, which will be used to realign the estimations to the measured vectors
    float errors[kNBaxis] = {0.0F, 0.0F, 0.0F};
    computeGravityError(errors, normalised_accelerometer, body_estimates);

    //apply the proportion and integral terms to error vectors
    float corrected_gyro_radps[kNBaxis];
    applyProportionateErrors(corrected_gyro_radps, sample, errors, context->weighed_kp);
    accumulateIntegralErrors(context->error_integrals, errors, timedelta_seconds, context->weighed_ki);
    applyIntegralErrors(context->error_integrals, corrected_gyro_radps);

    //integrate the corrected gyroscope data into the current attitude quaternion
    integrateGyroQuaternion(&context->attitude, corrected_gyro_radps, timedelta_seconds);

    //normalise the current attitude quaternion to avoid drift
    const float quaterion_norm = normaliseQuaternion(&context->attitude);
    if (isnan(quaterion_norm) || isinf(quaterion_norm)) {
        resetMahonyFilter(context);
        context->last_reset_cause = kQuaternionNanInf;
        return false;
    }

    //update the last update tick on success
    context->dt.last_valid_tick = context->dt.last_sampled_tick;
    return true;
}

/**
 * Get the current angle in [rad] along an axis
 * @note Yaw angle (around the Z axis) will always return 0, due to the absence of a magnetometer implementation
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
            return asinf(clamp_absolute(raw_sine, 1.0F));  //make sure to clamp the sin value between [-1, 1]

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
// cppcheck-suppress unusedFunction
float getAttitudeAngle(const MahonyContext* context) {
    //if no context provided, exit
    if (!context) {
        return 0.0F;
    }

    const float safe_cosine =
        clamp_absolute(context->attitude.q0, 1.0F);  //make sure to clamp the cos value between [-1, 1]
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
 * @param[out] array Array to normalise
 * @return Norm value
 */
static inline FORCE_INLINE_SILENT float normaliseArray(float array[kNBaxis]) {
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
 * @param[out] quaternion Quaternion to normalise
 * @return Norm value
 */
static inline FORCE_INLINE_SILENT float normaliseQuaternion(Quaternion* quaternion) {
    const float norm =
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
static inline FORCE_INLINE_SILENT float clamp_absolute(const float value, const float max_absolute_value) {
    return fmaxf(-max_absolute_value, fminf(max_absolute_value, value));
}

/**
 * Clamp a value to the range [min_value, max_value]
 *
 * @param value Value to clamp
 * @param min_value Minimum value
 * @param max_value Maximum value
 * @return Clamped value
 */
static inline FORCE_INLINE_SILENT float clamp_min_max(const float value, const float min_value, const float max_value) {
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/**
 * Compute the absolute value of a floating point number
 *
 * @param value Raw value
 * @return Absolute value
 */
static inline FORCE_INLINE_SILENT float absoluteValue(const float value) { return ((value >= 0.0F) ? value : -value); }

/**
 * Compute the elapsed time in [s] between current and previous timestamps
 *
 * @param delta Time delta structure
 * @return time delta in [s]
 */
static inline FORCE_INLINE_SILENT float computeDTseconds(const TimeDelta* delta) {
    //compute the time delta and avoid issues with the overflow after maxTick
    const uint32_t delta_ticks = (delta->last_sampled_tick - delta->last_valid_tick) & delta->max_tick;
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
 * @retval true Estimates are close enough to the accelerometer measurements
 * @retval false The angle between vectors is too wide (sign of large linear acceleration)
 */
static bool alignmentValid(const float accelerometer_normalised[kNBaxis], const float estimates_normalised[kNBaxis]) {
    const float dot_product = getNormalisedVectorsAngleCosine(accelerometer_normalised, estimates_normalised);
    return (bool)((dot_product >= kMinAlignmentCosine) && (dot_product <= kMaxAlignmentCosine));
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
 * @param[out] current_attitude Current attitude quaternion to update
 * @param corrected_gyro Gyroscope measurements, corrected with the PI filter and error rotation vectors
 * @param timedelta_seconds Period between now and the last update, in [s]
 */
static void integrateGyroQuaternion(Quaternion* current_attitude, const float corrected_gyro[kNBaxis],
                                    float timedelta_seconds) {
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
    current_attitude->q0 += (rate_of_change.q0 * timedelta_seconds);
    current_attitude->q1 += (rate_of_change.q1 * timedelta_seconds);
    current_attitude->q2 += (rate_of_change.q2 * timedelta_seconds);
    current_attitude->q3 += (rate_of_change.q3 * timedelta_seconds);
}

/**
 * Check if a norm provided is within a valid range
 *
 * @param norm Norm to validate
 * @retval true Norm valid
 * @retval false Norm invalid
 */
static bool normValid(const float norm) {
    return (bool)((norm > (1.0F - kMaxNormEpsilon)) && (norm < (1.0F + kMaxNormEpsilon)));
}

/**
 * Compute body estimates from the current context quaternion
 *
 * @param attitude Current Mahony filter's attitude quaternion
 * @param[out] body_estimates Array to fill with the estimates
 */
static void estimateOrientation(const Quaternion* attitude, float body_estimates[kNBaxis]) {
    body_estimates[kXaxis] = twice((attitude->q1 * attitude->q3) - (attitude->q0 * attitude->q2));
    body_estimates[kYaxis] = twice((attitude->q0 * attitude->q1) + (attitude->q2 * attitude->q3));
    body_estimates[kZaxis] =
        squared(attitude->q0) - squared(attitude->q1) - squared(attitude->q2) + squared(attitude->q3);
}

/**
 * Apply the filter's PI proportionate term to the current sample gyroscope values
 *
 * @param[out] corrected_gyro_radps Array to fill with the filtered values
 * @param sample Current sample from which get the gyro values
 * @param errors Error vectors to apply
 * @param trusted_kp Proportional gain, weighted by current measurement trust
 */
static void applyProportionateErrors(float corrected_gyro_radps[kNBaxis], const IMUsample* sample,
                                     const float errors[kNBaxis], const float trusted_kp) {
    corrected_gyro_radps[kXaxis] = (sample->gyroscope_radps[kXaxis] + (trusted_kp * errors[kXaxis]));
    corrected_gyro_radps[kYaxis] = (sample->gyroscope_radps[kYaxis] + (trusted_kp * errors[kYaxis]));
    corrected_gyro_radps[kZaxis] = (sample->gyroscope_radps[kZaxis] + (trusted_kp * errors[kZaxis]));
}

/**
 * Accumulate the new errors with the integral term into the error integrals array
 *
 * @param[out] error_integrals Integral error terms to update
 * @param errors Error orientation vector (body frame)
 * @param timedelta_seconds Time delta since last update
 * @param trusted_ki Integral gain, weighted by current measurement trust
 */
static void accumulateIntegralErrors(float error_integrals[kNBaxis], const float errors[kNBaxis],
                                     float timedelta_seconds, const float trusted_ki) {
    // Avoid if gain is 0 to avoid integrals pollution due to float approximating 0.0F
    if ((trusted_ki <= kCloseToZero)) {
        return;
    }

    for (uint8_t axis = 0; axis < kNBaxis; axis++) {
        error_integrals[axis] += (trusted_ki * errors[axis] * timedelta_seconds);
        error_integrals[axis] = clamp_absolute(error_integrals[axis], kMaxIntegralError);
    }
}

/**
 * Apply the filter's PI integral term to the current sample gyroscope values
 *
 * @param error_integrals Current integral error terms
 * @param[out] corrected_gyro_radps Array to correct
 */
static void applyIntegralErrors(const float error_integrals[kNBaxis], float corrected_gyro_radps[kNBaxis]) {
    for (uint8_t axis = 0; axis < kNBaxis; axis++) {
        corrected_gyro_radps[axis] += error_integrals[axis];
    }
}

/**
 * Get the cosine of the angle between two normalised vectors
 *
 * @param normalised_first First vector
 * @param normalised_second Second vector
 * @return Cosine of the angle between vectors
 */
static float getNormalisedVectorsAngleCosine(const float normalised_first[kNBaxis],
                                             const float normalised_second[kNBaxis]) {
    //This is done with the means of a dot product between vectors.
    //As the vectors are normalised, their dot product gives the cosine of the angle between them.
    const float dot_product = (normalised_first[kXaxis] * normalised_second[kXaxis]) +
                              (normalised_first[kYaxis] * normalised_second[kYaxis]) +
                              (normalised_first[kZaxis] * normalised_second[kZaxis]);

    return dot_product;
}

/**
 * Interpolate a value 
 *
 * @param raw_value Raw value to interpolate
 * @param min_raw Minimum raw value to interpolate
 * @param min_output Minimum value to output
 * @param max_raw Maximum raw value to interpolate
 * @param max_output Maximum value to output
 * @return Interpolated value
 */
// NOLINTNEXTLINE (bugprone-easily-swappable-parameters)
static float linearInterpolation(float raw_value, const float min_raw, const float min_output, const float max_raw,
                                 const float max_output) {
    const float delta_x = (max_raw - min_raw);
    if ((delta_x <= kCloseToZero) && (delta_x >= -kCloseToZero)) {
        return INFINITY;
    }

    raw_value = clamp_min_max(raw_value, min_raw, max_raw);

    const float slope = (max_output - min_output) / delta_x;
    return min_output + ((raw_value - min_raw) * slope);
}

/**
 * Modulate kP and kI coefficients according to a calculated trust level
 *
 * @param context Filter context
 * @param accelerometer_normalised Normalised acceleration vector
 * @param estimates_normalised Normalised estimated body attitude vector
 * @param acceleration_norm Norm of the acceleration
 */
static void applyTrustToCoefficients(MahonyContext* context, const float accelerometer_normalised[kNBaxis],
                                     const float estimates_normalised[kNBaxis], const float acceleration_norm) {
    if (!context) {
        return;
    }

    context->weighed_ki = context->base_ki;
    context->weighed_kp = context->base_kp;

    const float norm_absolute_deviation = absoluteValue(acceleration_norm - 1.0F);
    const float trusted_norm = linearInterpolation(norm_absolute_deviation, 0.0F, 1.0F, kMaxNormEpsilon, 0.0F);
    if (isinf(trusted_norm)) {
        return;
    }

    const float alignment_cosine = getNormalisedVectorsAngleCosine(accelerometer_normalised, estimates_normalised);
    const float trusted_alignment = linearInterpolation(alignment_cosine, kMinAlignmentCosine, 0.0F, 1.0F, 1.0F);
    if (isinf(trusted_alignment)) {
        return;
    }

    context->trust_weight = (trusted_norm * trusted_alignment);
    context->weighed_ki *= context->trust_weight;
    context->weighed_kp *= (kMinKpTrustFraction + ((1.0F - kMinKpTrustFraction) * context->trust_weight));
}
