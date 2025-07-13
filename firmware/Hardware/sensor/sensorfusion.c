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
 * quaternion_t attitude;
 * float integratedErrors[NB_AXIS];
 *
 * // Set the quaternion to a unit orientation (no rotation) and clear the integral terms
 * resetMahonyFilter(&attitude, integratedErrors);
 *
 * while (1) {
 *     float accel[NB_AXIS], gyro[NB_AXIS];
 *     readIMU(accel, gyro);  // User-provided sensor read
 *
 *     // Update both the orientation quaternion and the integral error terms
 *     updateMahonyFilter(&attitude, integratedErrors, accel, gyro, dt);
 * }
 * ```
 *
 * ## References
 * - Mahony, R., Hamel, T., & Pflimlin, J.-M. (2008). Nonlinear Complementary Filters on the Special
 *   Orthogonal Group. *IEEE Transactions on Automatic Control*, 53(5), 1203–1218.
 *   DOI: [10.1109/TAC.2008.923738](https://doi.org/10.1109/TAC.2008.923738)
 *
 * @author Gilles Henrard
 * @date 14/07/2025
 */
#include "sensorfusion.h"
#include <math.h>
#include <stdint.h>
#include "memsBMI270.h"

// utility functions
static inline __attribute((always_inline)) float half(float number);
static inline __attribute((always_inline)) float twice(float number);
static inline __attribute((always_inline)) float squared(float number);
static inline uint8_t                            normValid(float norm);
static inline __attribute((always_inline)) float getArrayNorm(const float array[3]);
static inline __attribute((always_inline)) float getQuaternionNorm(const quaternion_t* quaternion);
static inline uint8_t                            alignmentValid(const float accelerometerNormalised[NB_AXIS],
                                                                const float estimatesNormalised[NB_AXIS]);
static void computeErrors(float errors[NB_AXIS], const float accelerometer_G[NB_AXIS],
                          const float bodyEstimates[NB_AXIS]);
static void integrateGyroMeasurements(quaternion_t* currentAttitude, const float correctedGyro[NB_AXIS],
                                      float dtPeriod_sec);

//constants
static const float PROPORTIONAL_GAIN = 2.5F;    ///< Propotional gain (KP) of the Mahony filter
static const float INTEGRAL_GAIN     = 0.5F;    ///< Integral gain (KI) of the Mahony filter
static const float NO_INTEGRAL       = 0.001F;  ///< Value below which the integral gain is considered inexistant
static const float CLOSE_TO_ZERO     = 1e-3F;   ///< Value used to compare floats to 0

/*************************************************************************************************/
/*************************************************************************************************/

/**
 * Reset the quaternion and integral values used by the Mahony filter
 *
 * @param[out] currentAttitude Current attitude quaternion
 * @param[out] integratedErrors Array of current error integrated vector for each axis
 */
void resetMahonyFilter(quaternion_t* currentAttitude, float integratedErrors[NB_AXIS]) {
    *currentAttitude         = (quaternion_t){.q0 = 1.0F, .q1 = 0.0F, .q2 = 0.0F, .q3 = 0.0F};
    integratedErrors[X_AXIS] = integratedErrors[Y_AXIS] = integratedErrors[Z_AXIS] = 0.0F;
}

/**
 * Update the quaternion and integral values used by the Mahony filter with fresh ones
 *
 * @warning accelerometer_G[] values are internally normalised and cannot therefore be reused as is after this function
 *
 * @param[out] currentAttitude   Current attitude quaternion
 * @param[out] integratedErrors  Array of current error integrated vector for each axis
 * @param[in] accelerometer_G   Accelerometer measurements in [G] (m/s²)
 * @param[in] gyroscope_radps   Gyroscope measurements in [rad/s]
 * @param[in] dtPeriod_sec      Period between two filter updates in [s]
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void updateMahonyFilter(quaternion_t* currentAttitude, float integratedErrors[NB_AXIS], float accelerometer_G[NB_AXIS],
                        const float gyroscope_radps[NB_AXIS], float dtPeriod_sec) {
    //normalise accelerometer vectors to unit length, to avoid drift
    float norm = getArrayNorm(accelerometer_G);
    if(norm < CLOSE_TO_ZERO) {
        return;
    }
    accelerometer_G[X_AXIS] /= norm;
    accelerometer_G[Y_AXIS] /= norm;
    accelerometer_G[Z_AXIS] /= norm;

    //estimate the current body frame gravity vectors from the current orientation quaternion
    float bodyEstimates[NB_AXIS];
    bodyEstimates[X_AXIS] =
        twice((currentAttitude->q1 * currentAttitude->q3) - (currentAttitude->q0 * currentAttitude->q2));
    bodyEstimates[Y_AXIS] =
        twice((currentAttitude->q0 * currentAttitude->q1) + (currentAttitude->q2 * currentAttitude->q3));
    bodyEstimates[Z_AXIS] = squared(currentAttitude->q0) - squared(currentAttitude->q1) - squared(currentAttitude->q2)
                            + squared(currentAttitude->q3);

    //compute the error rotation vectors, which will be used to realign the estimations to the measured vectors
    //  (Only do it if no linear motion has been detected)
    float errors[NB_AXIS] = {0.0F, 0.0F, 0.0F};
    if(alignmentValid(accelerometer_G, bodyEstimates) && normValid(norm)) {
        computeErrors(errors, accelerometer_G, bodyEstimates);

        //apply the integral gain to the error vectors
        // (avoid if gain is 0 to avoid integral windup due to float 0.0F)
        if(INTEGRAL_GAIN > NO_INTEGRAL) {
            integratedErrors[X_AXIS] += (INTEGRAL_GAIN * errors[X_AXIS] * dtPeriod_sec);
            integratedErrors[Y_AXIS] += (INTEGRAL_GAIN * errors[Y_AXIS] * dtPeriod_sec);
            integratedErrors[Z_AXIS] += (INTEGRAL_GAIN * errors[Z_AXIS] * dtPeriod_sec);
        }
    }

    //apply the PI-filtered error vectors to the gyroscope measurements
    const float correctedGyro_radps[NB_AXIS] = {
        [X_AXIS] = (gyroscope_radps[X_AXIS] + (PROPORTIONAL_GAIN * errors[X_AXIS]) + integratedErrors[X_AXIS]),
        [Y_AXIS] = (gyroscope_radps[Y_AXIS] + (PROPORTIONAL_GAIN * errors[Y_AXIS]) + integratedErrors[Y_AXIS]),
        [Z_AXIS] = (gyroscope_radps[Z_AXIS] + (PROPORTIONAL_GAIN * errors[Z_AXIS]) + integratedErrors[Z_AXIS])};

    //integrate the corrected gyroscope data into the current attitude quaternion
    integrateGyroMeasurements(currentAttitude, correctedGyro_radps, dtPeriod_sec);

    //normalise the current attitude quaternion to avoid drift
    norm = getQuaternionNorm(currentAttitude);
    if(norm < CLOSE_TO_ZERO) {
        return;
    }
    currentAttitude->q0 /= norm;
    currentAttitude->q1 /= norm;
    currentAttitude->q2 /= norm;
    currentAttitude->q3 /= norm;
}

/**
 * Get the current angle in [rad] along an axis
 * @note Yaw angle (around the Z axis) will always return 0, due to the absence of a magnetometer
 *
 * @param currentAttitude   Current attitude quaternion
 * @param axis              Axis along which getting the angle
 * @return Angle in [rad]
 */
float angleAlongAxis(const quaternion_t* currentAttitude, axis_e axis) {
    float rawSin  = 0.0F;
    float safeSin = 0.0F;

    switch(axis) {
        case X_AXIS:  //roll
            return atan2f(
                (twice((currentAttitude->q0 * currentAttitude->q1) + (currentAttitude->q2 * currentAttitude->q3))),
                1.0F
                    - (twice((currentAttitude->q1 * currentAttitude->q1)
                             + (currentAttitude->q2 * currentAttitude->q2))));

        case Y_AXIS:  //pitch
            rawSin  = twice((currentAttitude->q1 * currentAttitude->q3) - (currentAttitude->q0 * currentAttitude->q2));
            safeSin = fmaxf(-1.0F, fminf(1.0F, rawSin));  //make sure to clamp the sin value between [-1, 1]
            return asinf(safeSin);

        case Z_AXIS:
        case NB_AXIS:
        default:
            return 0.0F;
    };
}

/**
 * Get the angle in [rad] along the current quaternion attitude axis
 *
 * @param currentAttitude   Current attitude quaternion
 * @return Angle in [rad]
 */
float getAttitudeAngle(const quaternion_t* currentAttitude) {
    const float safeCos = fmaxf(-1.0F, fminf(1.0F, currentAttitude->q0));
    return twice(acosf(safeCos));
}

/*************************************************************************************************/
/*************************************************************************************************/

/**
 * Compute the half-value of a number
 * @details Mainly used for readability purposes
 * 
 * @param number Number to half
 * @return Half the value of number
 */
static inline __attribute((always_inline)) float half(const float number) {
    return 0.5F * number;  // NOLINT(*-magic-numbers)
}

/**
 * Compute the double of a number
 * @details Mainly used for readability purposes
 * 
 * @param number Number to double
 * @return Double the value of number
 */
static inline __attribute((always_inline)) float twice(const float number) {
    return 2.0F * number;  // NOLINT(*-magic-numbers)
}

/**
 * Compute the squared value of a number
 * @details Mainly used for readability purposes
 * 
 * @param number Number to square
 * @return Number squared
 */
static inline __attribute((always_inline)) float squared(const float number) {
    return number * number;
}

/**
 * Get the norm of an array of vectors
 *
 * @param array Array of which get the norm
 * @return Norm value
 */
static inline __attribute((always_inline)) float getArrayNorm(const float array[3]) {
    return sqrtf(squared(array[0U]) + squared(array[1U]) + squared(array[2U]));
}

/**
 * Get the norm of a quaternion
 *
 * @param array Quaternion of which get the norm
 * @return Norm value
 */
static inline __attribute((always_inline)) float getQuaternionNorm(const quaternion_t* quaternion) {
    return sqrtf(squared(quaternion->q0) + squared(quaternion->q1) + squared(quaternion->q2) + squared(quaternion->q3));
}

/**
 * Check if a normalised value is within a valid range
 * @details A norm should always be as close to 1 as possible.
 *
 * @param norm Norm to check
 * @retval 1 The norm is valid
 * @retval 0 The norm is out of the valid range (sign of large linear acceleration)
 */
static inline uint8_t normValid(const float norm) {
    //limit values used are empirical
    return ((norm >= 0.85F) && (norm <= 1.15F));  // NOLINT(*-magic-numbers)
}

/**
 * Check if the linear acceleration measured on all axis align well enough with the estimations
 * @details This is done with the means of a dot product between vectors.
 * As the vectors are normalised, their dot product gives the cosine of the angles between them.
 *
 * @param accelerometerNormalised Accelerometer vectors, normalised to unit length
 * @param estimatesNormalised Estimates vectors, normalised to unit length
 * @retval 1 Estimates are close enough to the accelerometer measurements
 * @retval 0 The angle between vectors is too wide (sign of large linear acceleration)
 */
static inline uint8_t alignmentValid(const float accelerometerNormalised[NB_AXIS],
                                     const float estimatesNormalised[NB_AXIS]) {
    const float MINIMUM_COSINE = 0.9659F;  ///< cosine value for 15°, used as a maximum angle difference

    const float dotProduct = (accelerometerNormalised[X_AXIS] * estimatesNormalised[X_AXIS])
                             + (accelerometerNormalised[Y_AXIS] * estimatesNormalised[Y_AXIS])
                             + (accelerometerNormalised[Z_AXIS] * estimatesNormalised[Z_AXIS]);

    return (dotProduct > MINIMUM_COSINE);
}

/**
 * @brief Computes the 3D error vector between measured and estimated gravity.
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
 * Integrate corrected gyroscope measurements and apply it to the current attitude quaternion
 *
 * @param currentAttitude Current attitude quaternion to update
 * @param correctedGyro Gyroscope measurements, corrected with the PI filter and error rotation vectors
 * @param dtPeriod_sec Period between now and the last update, in [s]
 */
static void integrateGyroMeasurements(quaternion_t* currentAttitude, const float correctedGyro[NB_AXIS],
                                      float dtPeriod_sec) {
    //compute the derivative quaternion, composed of the current attitude and the gyroscope measurements
    const quaternion_t rateChange = {
        .q0 = half((-currentAttitude->q1 * correctedGyro[X_AXIS]) - (currentAttitude->q2 * correctedGyro[Y_AXIS])
                   - (currentAttitude->q3 * correctedGyro[Z_AXIS])),
        .q1 = half((currentAttitude->q0 * correctedGyro[X_AXIS]) + (currentAttitude->q2 * correctedGyro[Z_AXIS])
                   - (currentAttitude->q3 * correctedGyro[Y_AXIS])),
        .q2 = half((currentAttitude->q0 * correctedGyro[Y_AXIS]) - (currentAttitude->q1 * correctedGyro[Z_AXIS])
                   + (currentAttitude->q3 * correctedGyro[X_AXIS])),
        .q3 = half((currentAttitude->q0 * correctedGyro[Z_AXIS]) + (currentAttitude->q1 * correctedGyro[Y_AXIS])
                   - (currentAttitude->q2 * correctedGyro[X_AXIS]))};

    //integrate the current quaternion with the change rate
    currentAttitude->q0 += (rateChange.q0 * dtPeriod_sec);
    currentAttitude->q1 += (rateChange.q1 * dtPeriod_sec);
    currentAttitude->q2 += (rateChange.q2 * dtPeriod_sec);
    currentAttitude->q3 += (rateChange.q3 * dtPeriod_sec);
}
