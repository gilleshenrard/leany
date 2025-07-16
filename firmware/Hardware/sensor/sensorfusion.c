// SPDX-License-Identifier: MIT
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
 * mahonycontext_t filterContext;
 *
 * // Set the quaternion to a unit orientation (no rotation) and clear the integral terms
 * resetMahonyFilter(&filterContext);
 *
 * while (1) {
 *     float accel[NB_AXIS], gyro[NB_AXIS];
 *     readIMU(accel, gyro);  // User-provided sensor read
 *
 *     // Update the orientation quaternion, the integral error terms and update timestamps
 *     updateMahonyFilter(&filterContext, accel, gyro);
 * }
 * ```
 *
 * ## References
 * - Mahony, R., Hamel, T., & Pflimlin, J.-M. (2008). Nonlinear Complementary Filters on the Special
 *   Orthogonal Group. *IEEE Transactions on Automatic Control*, 53(5), 1203–1218.
 *   DOI: [10.1109/TAC.2008.923738](https://doi.org/10.1109/TAC.2008.923738)
 *
 * @author Gilles Henrard
 * @date 17/07/2025
 */
#include "sensorfusion.h"
#include <math.h>
#include <stdint.h>

#define FORCE_INLINE_SILENT __attribute((always_inline))  ///< Macro used to workaround Doxygen issues with __attribute
#if defined(__DOXYGEN__) && __DOXYGEN__ == 1
#define FORCE_INLINE_SILENT
#endif

// utility functions
static inline FORCE_INLINE_SILENT float   half(float number);
static inline FORCE_INLINE_SILENT float   twice(float number);
static inline FORCE_INLINE_SILENT float   squared(float number);
static inline FORCE_INLINE_SILENT uint8_t normValid(float norm);
static inline FORCE_INLINE_SILENT float   normaliseArray(float array[3]);
static inline FORCE_INLINE_SILENT float   normaliseQuaternion(quaternion_t* quaternion);
static inline FORCE_INLINE_SILENT float   clamp(float value, float maxAbsoluteValue);
static inline FORCE_INLINE_SILENT float   getDT(const timedelta_t* delta);
static inline FORCE_INLINE_SILENT uint8_t isDTvalid(float deltaSeconds);
static uint8_t alignmentValid(const float accelerometerNormalised[NB_AXIS], const float estimatesNormalised[NB_AXIS]);
static void    computeErrors(float errors[NB_AXIS], const float accelerometer_G[NB_AXIS],
                             const float bodyEstimates[NB_AXIS]);
static void    integrateGyroMeasurements(quaternion_t* currentAttitude, const float correctedGyro[NB_AXIS],
                                         float dtPeriod_sec);

//constants
static const float   CLOSE_TO_ZERO       = 1e-3F;     ///< Value used to compare floats to 0
static const float   MINIMUM_ALIGNCOSINE = 0.9659F;   ///< cosine value for 15°, used as a maximum alignment angle
static const float   MAXIMUM_ALIGNCOSINE = 1.00001F;  ///< maximum alignment angle cosine acceptable
static const float   MAXIMUM_NORMEPSILON = 0.15F;     ///< Maximum deviation of a norm around 1
static const float   MAX_INTEGRAL_ERROR  = 0.3F;      ///< Maximum integral error absolute value accepted
static const float   MIN_VALID_DT_SEC    = 1e-6F;     ///< Minimum acceptable timespan between updates
static const float   MAX_VALID_DT_SEC    = 0.5F;      ///< Maximum acceptable timespan between updates
static const uint8_t MAX_BAD_COUNTS      = 5U;        ///< Maximum number of bad accel. or quatern. counts before reset

/*********************************************************************************************************************************/
// Publicly accessible functions
/*********************************************************************************************************************************/

/**
 * Reset the quaternion and integral values used by the Mahony filter
 *
 * @param[out] context Current Mahony filter context
 */
void resetMahonyFilter(mahonycontext_t* context) {
    //if no context provided, exit
    if(!context) {
        return;
    }

    context->attitude               = (quaternion_t){.q0 = 1.0F, .q1 = 0.0F, .q2 = 0.0F, .q3 = 0.0F};
    context->errorIntegrals[X_AXIS] = 0.0F;
    context->errorIntegrals[Y_AXIS] = 0.0F;
    context->errorIntegrals[Z_AXIS] = 0.0F;
    context->badAccelerationCount   = 0;
    context->badQuaternionCount     = 0;
    context->dt.previousTick        = context->dt.currentTick;
}

/**
 * Update the quaternion, integral values and update timestamps used by the Mahony filter with fresh ones
 *
 * @param[out] context          Current Mahony filter context
 * @param[in] accelerometer_G   Accelerometer measurements in [G] (m/s²)
 * @param[in] gyroscope_radps   Gyroscope measurements in [rad/s]
 */
void updateMahonyFilter(mahonycontext_t* context,
                        const float      accelerometer_G[NB_AXIS],  //NOLINT(bugprone-easily-swappable-parameters)
                        const float      gyroscope_radps[NB_AXIS]) {
    //if no pointer provided, exit
    if(!context || !accelerometer_G || !gyroscope_radps) {
        return;
    }

    //normalise accelerometer vectors to unit length, to avoid drift
    float       normalisedAccelerometer[NB_AXIS] = {accelerometer_G[0], accelerometer_G[1], accelerometer_G[2]};
    const float accelNorm                        = normaliseArray(normalisedAccelerometer);
    if(accelNorm < CLOSE_TO_ZERO) {
        if(++context->badAccelerationCount >= MAX_BAD_COUNTS) {
            resetMahonyFilter(context);
        }
        return;
    }
    context->badAccelerationCount = 0;

    //if dT out of reasonable bounds, reset the filter
    const float dtPeriod_sec = getDT(&context->dt);
    if(!isDTvalid(dtPeriod_sec)) {
        resetMahonyFilter(context);
    }

    //estimate the current body frame gravity vectors from the current orientation quaternion
    float bodyEstimates[NB_AXIS];
    bodyEstimates[X_AXIS] =
        twice((context->attitude.q1 * context->attitude.q3) - (context->attitude.q0 * context->attitude.q2));
    bodyEstimates[Y_AXIS] =
        twice((context->attitude.q0 * context->attitude.q1) + (context->attitude.q2 * context->attitude.q3));
    bodyEstimates[Z_AXIS] = squared(context->attitude.q0) - squared(context->attitude.q1)
                            - squared(context->attitude.q2) + squared(context->attitude.q3);

    //Abort update if strong linear motion is detected
    if(!alignmentValid(normalisedAccelerometer, bodyEstimates) || !normValid(accelNorm)) {
        return;
    }

    //compute the error rotation vectors, which will be used to realign the estimations to the measured vectors
    float errors[NB_AXIS] = {0.0F, 0.0F, 0.0F};
    computeErrors(errors, normalisedAccelerometer, bodyEstimates);

    //apply the proportion term to error vectors and add them to the gyroscope measurements
    float correctedGyro_radps[NB_AXIS] = {[X_AXIS] = (gyroscope_radps[X_AXIS] + (context->KP * errors[X_AXIS])),
                                          [Y_AXIS] = (gyroscope_radps[Y_AXIS] + (context->KP * errors[Y_AXIS])),
                                          [Z_AXIS] = (gyroscope_radps[Z_AXIS] + (context->KP * errors[Z_AXIS]))};

    //apply the clamped integral term to error vectors and add them to the gyroscope measurements
    // (avoid if gain is 0 to avoid integral windup due to float approximate 0.0F)
    uint8_t axis = 0;
    while((context->KI > CLOSE_TO_ZERO) && (axis < NB_AXIS)) {
        context->errorIntegrals[axis] += (context->KI * errors[axis] * dtPeriod_sec);
        context->errorIntegrals[axis] = clamp(context->errorIntegrals[axis], MAX_INTEGRAL_ERROR);
        correctedGyro_radps[axis] += context->errorIntegrals[axis];
        axis++;
    }

    //integrate the corrected gyroscope data into the current attitude quaternion
    integrateGyroMeasurements(&context->attitude, correctedGyro_radps, dtPeriod_sec);

    //normalise the current attitude quaternion to avoid drift
    const float quaterionNorm = normaliseQuaternion(&context->attitude);
    if(quaterionNorm < CLOSE_TO_ZERO) {
        if(++context->badQuaternionCount >= MAX_BAD_COUNTS) {
            resetMahonyFilter(context);
        }
        return;
    }
    context->badQuaternionCount = 0;

    //update the last update tick on success
    context->dt.previousTick = context->dt.currentTick;
}

/**
 * Get the current angle in [rad] along an axis
 * @note Yaw angle (around the Z axis) will always return 0, due to the absence of a magnetometer
 *
 * @param context Current Mahony filter context
 * @param axis    Axis along which getting the angle
 * @return Angle in [rad] if X or Y axis requested, 0 otherwise
 */
float angleAlongAxis(const mahonycontext_t* context, axis_e axis) {
    float rawSine = 0.0F;

    //if no context provided, exit
    if(!context) {
        return 0.0F;
    }

    switch(axis) {
        case X_AXIS:  //roll
            return atan2f(
                (twice((context->attitude.q0 * context->attitude.q1) + (context->attitude.q2 * context->attitude.q3))),
                1.0F
                    - (twice((context->attitude.q1 * context->attitude.q1)
                             + (context->attitude.q2 * context->attitude.q2))));

        case Y_AXIS:  //pitch
            rawSine =
                twice((context->attitude.q1 * context->attitude.q3) - (context->attitude.q0 * context->attitude.q2));
            return asinf(clamp(rawSine, 1.0F));  //make sure to clamp the sin value between [-1, 1]

        case Z_AXIS:
        case NB_AXIS:
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
float getAttitudeAngle(const mahonycontext_t* context) {
    //if no context provided, exit
    if(!context) {
        return 0.0F;
    }

    const float safeCos = clamp(context->attitude.q0, 1.0F);  //make sure to clamp the cos value between [-1, 1]
    return twice(acosf(safeCos));
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
static inline FORCE_INLINE_SILENT float squared(const float number) {
    return number * number;
}

/**
 * Normalise an array of vectors
 *
 * @param array Array to normalise
 * @return Norm value
 */
static inline FORCE_INLINE_SILENT float normaliseArray(float array[3]) {
    const float norm = sqrtf(squared(array[0U]) + squared(array[1U]) + squared(array[2U]));
    if(norm < CLOSE_TO_ZERO) {
        return norm;
    }

    array[X_AXIS] /= norm;
    array[Y_AXIS] /= norm;
    array[Z_AXIS] /= norm;
    return norm;
}

/**
 * Normalise a quaternion
 *
 * @param quaternion Quaternion to normalise
 * @return Norm value
 */
static inline FORCE_INLINE_SILENT float normaliseQuaternion(quaternion_t* quaternion) {
    float norm =
        sqrtf(squared(quaternion->q0) + squared(quaternion->q1) + squared(quaternion->q2) + squared(quaternion->q3));
    if(norm < CLOSE_TO_ZERO) {
        return norm;
    }

    quaternion->q0 /= norm;
    quaternion->q1 /= norm;
    quaternion->q2 /= norm;
    quaternion->q3 /= norm;
    return norm;
}

/**
 * Clamp a value to the symmetric range [-maxAbsoluteValue, +maxAbsoluteValue]
 *
 * @param value Value to clamp
 * @param maxAbsoluteValue Absolute maximum magnitude of the output
 * @return Clamped value
 */
static inline FORCE_INLINE_SILENT float clamp(const float value, const float maxAbsoluteValue) {
    return fmaxf(-maxAbsoluteValue, fminf(maxAbsoluteValue, value));
}

/**
 * Check if a normalised value is within a valid range
 * @details A norm should always be as close to 1 as possible.
 *
 * @param norm Norm to check
 * @retval 1 The norm is valid
 * @retval 0 The norm is out of the valid range (sign of large linear acceleration)
 */
static inline FORCE_INLINE_SILENT uint8_t normValid(const float norm) {
    return ((norm >= (1.0F - MAXIMUM_NORMEPSILON)) && (norm <= (1.0F + MAXIMUM_NORMEPSILON)));
}

/**
 * Compute the elapsed time in [s] between current and previous timestamps
 *
 * @param delta Time delta structure
 * @return time delta in [s]
 */
static inline FORCE_INLINE_SILENT float getDT(const timedelta_t* delta) {
    //compute the time delta and avoid issues with the overflow after maxTick
    const uint32_t delta_ticks = (delta->currentTick - delta->previousTick) & delta->maxTick;
    return ((float)delta_ticks * delta->resolutionSeconds);
}

/**
 * Check whether a time delta is between 1us and 0.5s
 *
 * @param deltaSeconds Time delta in [s]
 * @retval 0 Time delta is out of bounds
 * @retval 1 Time delta is within bounds
 */
static inline FORCE_INLINE_SILENT uint8_t isDTvalid(float deltaSeconds) {
    return ((deltaSeconds > MIN_VALID_DT_SEC) && (deltaSeconds < MAX_VALID_DT_SEC));
}

/**
 * Check if the linear acceleration vector measured in the 3D space aligns well enough with the estimates vector
 *
 * @param accelerometerNormalised Accelerometer vectors, normalised to unit length
 * @param estimatesNormalised Estimates vectors, normalised to unit length
 * @retval 1 Estimates are close enough to the accelerometer measurements
 * @retval 0 The angle between vectors is too wide (sign of large linear acceleration)
 */
static uint8_t alignmentValid(const float accelerometerNormalised[NB_AXIS], const float estimatesNormalised[NB_AXIS]) {
    //This is done with the means of a dot product between vectors.
    //As the vectors are normalised, their dot product gives the cosine of the angle between them.
    const float dotProduct = (accelerometerNormalised[X_AXIS] * estimatesNormalised[X_AXIS])
                             + (accelerometerNormalised[Y_AXIS] * estimatesNormalised[Y_AXIS])
                             + (accelerometerNormalised[Z_AXIS] * estimatesNormalised[Z_AXIS]);

    return ((dotProduct > MINIMUM_ALIGNCOSINE) && (dotProduct < MAXIMUM_ALIGNCOSINE));
}

/**
 * Compute the 3D error vector between measured and estimated gravity.
 * @details Uses the cross product of the normalized accelerometer vector and the estimated gravity vector (from quaternion)
 * to compute the direction and magnitude of the orientation error. This error is used to correct the gyroscope bias.
 *
 * @param accelerometer_G Normalized accelerometer reading (measured gravity direction in [G]).
 * @param bodyEstimates Estimated gravity direction derived from current orientation quaternion.
 * @param[out] errors Error orientation vector (body frame).
 */
static void computeErrors(float errors[NB_AXIS], const float accelerometer_G[NB_AXIS],
                          const float bodyEstimates[NB_AXIS]) {
    errors[X_AXIS] =
        ((accelerometer_G[Y_AXIS] * bodyEstimates[Z_AXIS]) - (accelerometer_G[Z_AXIS] * bodyEstimates[Y_AXIS]));
    errors[Y_AXIS] =
        ((accelerometer_G[Z_AXIS] * bodyEstimates[X_AXIS]) - (accelerometer_G[X_AXIS] * bodyEstimates[Z_AXIS]));
    errors[Z_AXIS] =
        ((accelerometer_G[X_AXIS] * bodyEstimates[Y_AXIS]) - (accelerometer_G[Y_AXIS] * bodyEstimates[X_AXIS]));
}

/**
 * Integrate corrected gyroscope measurements and apply them to the current attitude quaternion
 *
 * @param currentAttitude Current attitude quaternion to update
 * @param correctedGyro Gyroscope measurements, corrected with the PI filter and error rotation vectors
 * @param dtPeriod_sec Period between now and the last update, in [s]
 */
static void integrateGyroMeasurements(quaternion_t* currentAttitude, const float correctedGyro[NB_AXIS],
                                      float dtPeriod_sec) {
    //compute the derivative quaternion, composed of the current attitude and the gyroscope measurements

    const float q0 = currentAttitude->q0;  //NOLINT(readability-identifier-length)
    const float q1 = currentAttitude->q1;  //NOLINT(readability-identifier-length)
    const float q2 = currentAttitude->q2;  //NOLINT(readability-identifier-length)
    const float q3 = currentAttitude->q3;  //NOLINT(readability-identifier-length)

    const quaternion_t rateChange = {
        .q0 = half((-q1 * correctedGyro[X_AXIS]) - (q2 * correctedGyro[Y_AXIS]) - (q3 * correctedGyro[Z_AXIS])),
        .q1 = half((q0 * correctedGyro[X_AXIS]) + (q2 * correctedGyro[Z_AXIS]) - (q3 * correctedGyro[Y_AXIS])),
        .q2 = half((q0 * correctedGyro[Y_AXIS]) - (q1 * correctedGyro[Z_AXIS]) + (q3 * correctedGyro[X_AXIS])),
        .q3 = half((q0 * correctedGyro[Z_AXIS]) + (q1 * correctedGyro[Y_AXIS]) - (q2 * correctedGyro[X_AXIS]))};

    //integrate the current quaternion with the change rate
    currentAttitude->q0 += (rateChange.q0 * dtPeriod_sec);
    currentAttitude->q1 += (rateChange.q1 * dtPeriod_sec);
    currentAttitude->q2 += (rateChange.q2 * dtPeriod_sec);
    currentAttitude->q3 += (rateChange.q3 * dtPeriod_sec);
}
