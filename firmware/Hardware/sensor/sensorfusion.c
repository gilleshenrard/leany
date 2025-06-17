/**
 * @file sensorfusion.c
 * @brief Mahony sensor fusion algorithm (quaternion-based)
 * @details Computes 3D orientation from gyroscope and accelerometer data using the Mahony complementary filter. 
 * The filter uses a quaternion to represent orientation and applies a proportional-integral correction to 
 * estimate attitude accurately without a magnetometer.
 * @date 17/06/2025
 */
#include "sensorfusion.h"
#include <math.h>
#include <stdint.h>
#include "memsBMI270.h"

enum {
    QUATER_ALIGNMENT = 16U,  ///< Memory alignment of the quaternion structure
};

/**
 * Structure defining a quaternion
 */
typedef struct {
    float q0;  ///< Scalar value
    float q1;  ///< Value which multiplies the unit vector along the X axis
    float q2;  ///< Value which multiplies the unit vector along the Y axis
    float q3;  ///< Value which multiplies the unit vector along the Z axis
} __attribute__((aligned(QUATER_ALIGNMENT))) quaternion_t;

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

// global variables
static quaternion_t currentAttit;               ///< Quaternion representing the current attitude (axis and angle)
static float        integratedErrors[NB_AXIS];  ///< Integrated feedback errors

/*************************************************************************************************/
/*************************************************************************************************/

/**
 * Reset the quaternion and integral values used by the Mahony filter
 */
void resetMahonyFilter(void) {
    currentAttit             = (quaternion_t){.q0 = 1.0F, .q1 = 0.0F, .q2 = 0.0F, .q3 = 0.0F};
    integratedErrors[X_AXIS] = integratedErrors[Y_AXIS] = integratedErrors[Z_AXIS] = 0.0F;
}

/**
 * Update the quaternion and integral values used by the Mahony filter with fresh ones
 *
 * @param[in] accelerometer_G   Accelerometer measurements in [G] (m/s²)
 * @param[in] gyroscope_radps   Gyroscope measurements in [rad/s]
 * @param[in] dtPeriod_sec      Period between two filter updates in [s]
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void updateMahonyFilter(float accelerometer_G[], const float gyroscope_radps[], const float dtPeriod_sec) {
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
    bodyEstimates[X_AXIS] = twice((currentAttit.q1 * currentAttit.q3) - (currentAttit.q0 * currentAttit.q2));
    bodyEstimates[Y_AXIS] = twice((currentAttit.q0 * currentAttit.q1) + (currentAttit.q2 * currentAttit.q3));
    bodyEstimates[Z_AXIS] =
        squared(currentAttit.q0) - squared(currentAttit.q1) - squared(currentAttit.q2) + squared(currentAttit.q3);

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
    integrateGyroMeasurements(&currentAttit, correctedGyro_radps, dtPeriod_sec);

    //normalise the current attitude quaternion to avoid drift
    norm = getQuaternionNorm(&currentAttit);
    if(norm < CLOSE_TO_ZERO) {
        return;
    }
    currentAttit.q0 /= norm;
    currentAttit.q1 /= norm;
    currentAttit.q2 /= norm;
    currentAttit.q3 /= norm;
}

/**
 * Get the current angle in [rad] along an axis
 * @note Yaw angle (around the Z axis) will always return 0, due to the absence of a magnetometer
 *
 * @param axis Axis along which getting the angle
 * @return Angle in [rad]
 */
float angleAlongAxis(axis_e axis) {
    float rawSin  = 0.0F;
    float safeSin = 0.0F;

    switch(axis) {
        case X_AXIS:  //roll
            return atan2f((twice((currentAttit.q0 * currentAttit.q1) + (currentAttit.q2 * currentAttit.q3))),
                          1.0F - (twice((currentAttit.q1 * currentAttit.q1) + (currentAttit.q2 * currentAttit.q2))));

        case Y_AXIS:  //pitch
            rawSin  = twice((currentAttit.q1 * currentAttit.q3) - (currentAttit.q0 * currentAttit.q2));
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
 * @return Angle in [rad]
 */
float getAttitudeAngle(void) {
    return twice(acosf(fmaxf(-1.0F, fminf(1.0F, currentAttit.q0))));
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
