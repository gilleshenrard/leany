#include "sensorfusion.h"
#include <math.h>
#include <stdint.h>

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
static inline uint8_t                            normInvalid(float norm);

//constants
static const float PROPORTIONAL_GAIN = 2.5F;    ///< Propotional gain (KP) of the Mahony filter
static const float INTEGRAL_GAIN     = 0.5F;    ///< Integral gain (KI) of the Mahony filter
static const float NO_INTEGRAL       = 0.001F;  ///< Value below which the integral gain is considered inexistant
const float MIN_ALIGNMENT = 0.96F;  ///< Threshold for acceptable align. between accel and estimates (cos(15°) ≈ 0.9659)

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
void updateMahonyFilter(float accelerometer_G[], float gyroscope_radps[], const float dtPeriod_sec) {
    //normalise accelerometer measurements
    float norm =
        sqrtf(squared(accelerometer_G[X_AXIS]) + squared(accelerometer_G[Y_AXIS]) + squared(accelerometer_G[Z_AXIS]));
    accelerometer_G[X_AXIS] /= norm;
    accelerometer_G[Y_AXIS] /= norm;
    accelerometer_G[Z_AXIS] /= norm;

    //estimate the current body frame gravity vector from the current orientation quaternion
    const float xBodyEstimate = twice((currentAttit.q1 * currentAttit.q3) - (currentAttit.q0 * currentAttit.q2));
    const float yBodyEstimate = twice((currentAttit.q0 * currentAttit.q1) + (currentAttit.q2 * currentAttit.q3));
    const float zBodyEstimate =
        squared(currentAttit.q0) - squared(currentAttit.q1) - squared(currentAttit.q2) + squared(currentAttit.q3);

    //compute the dot product of the accelerometer measurements and the body estimations
    const float dotProduct = (accelerometer_G[X_AXIS] * xBodyEstimate) + (accelerometer_G[Y_AXIS] * yBodyEstimate)
                             + (accelerometer_G[Z_AXIS] * zBodyEstimate);

    //compute the error rotation vectors, which align estimations to the gravity measured by accelerometer
    // This is done through a cross-product
    // Only do it if no linear motion has been detected (i.e. accel aligns well enough with estimates and accel norm is of proper magnitude)
    float xError = 0.0F;
    float yError = 0.0F;
    float zError = 0.0F;
    if((dotProduct > MIN_ALIGNMENT) && !normInvalid(norm)) {
        xError = ((accelerometer_G[Y_AXIS] * zBodyEstimate) - (accelerometer_G[Z_AXIS] * yBodyEstimate));
        yError = ((accelerometer_G[Z_AXIS] * xBodyEstimate) - (accelerometer_G[X_AXIS] * zBodyEstimate));
        zError = ((accelerometer_G[X_AXIS] * yBodyEstimate) - (accelerometer_G[Y_AXIS] * xBodyEstimate));
    }

    //apply the integral gain to the error measured
    // (avoid if gain is 0 to avoid integral windup due to float 0.0F)
    if(INTEGRAL_GAIN > NO_INTEGRAL) {
        integratedErrors[X_AXIS] += (INTEGRAL_GAIN * xError * dtPeriod_sec);
        integratedErrors[Y_AXIS] += (INTEGRAL_GAIN * yError * dtPeriod_sec);
        integratedErrors[Z_AXIS] += (INTEGRAL_GAIN * zError * dtPeriod_sec);
    }

    //apply the PI-filtered error to the gyroscope measurements
    const float correctedGyro_radps[NB_AXIS] = {
        [X_AXIS] = (gyroscope_radps[X_AXIS] + (PROPORTIONAL_GAIN * xError) + integratedErrors[X_AXIS]),
        [Y_AXIS] = (gyroscope_radps[Y_AXIS] + (PROPORTIONAL_GAIN * yError) + integratedErrors[Y_AXIS]),
        [Z_AXIS] = (gyroscope_radps[Z_AXIS] + (PROPORTIONAL_GAIN * zError) + integratedErrors[Z_AXIS])};

    //compute the derivative quaternion (rate of change since last update)
    const quaternion_t rateChange = {
        .q0 = half((-currentAttit.q1 * correctedGyro_radps[X_AXIS]) - (currentAttit.q2 * correctedGyro_radps[Y_AXIS])
                   - (currentAttit.q3 * correctedGyro_radps[Z_AXIS])),
        .q1 = half((currentAttit.q0 * correctedGyro_radps[X_AXIS]) + (currentAttit.q2 * correctedGyro_radps[Z_AXIS])
                   - (currentAttit.q3 * correctedGyro_radps[Y_AXIS])),
        .q2 = half((currentAttit.q0 * correctedGyro_radps[Y_AXIS]) - (currentAttit.q1 * correctedGyro_radps[Z_AXIS])
                   + (currentAttit.q3 * correctedGyro_radps[X_AXIS])),
        .q3 = half((currentAttit.q0 * correctedGyro_radps[Z_AXIS]) + (currentAttit.q1 * correctedGyro_radps[Y_AXIS])
                   - (currentAttit.q2 * correctedGyro_radps[X_AXIS]))};

    //integrate the current quaternion with the change rate
    currentAttit.q0 += (rateChange.q0 * dtPeriod_sec);
    currentAttit.q1 += (rateChange.q1 * dtPeriod_sec);
    currentAttit.q2 += (rateChange.q2 * dtPeriod_sec);
    currentAttit.q3 += (rateChange.q3 * dtPeriod_sec);

    //normalise the current attitude quaternion
    norm = sqrtf(squared(currentAttit.q0) + squared(currentAttit.q1) + squared(currentAttit.q2)
                 + squared(currentAttit.q3));
    currentAttit.q0 /= norm;
    currentAttit.q1 /= norm;
    currentAttit.q2 /= norm;
    currentAttit.q3 /= norm;
}

/**
 * Get the current angle in [rad] along an axis
 *
 * @param axis Axis along which getting the angle
 * @return Angle in [rad]
 */
float angleAlongAxis(axis_e axis) {
    float raw  = 0.0F;
    float safe = 0.0F;
    switch(axis) {
        case X_AXIS:  //roll
            return atan2f((twice((currentAttit.q0 * currentAttit.q1) + (currentAttit.q2 * currentAttit.q3))),
                          1.0F - (twice((currentAttit.q1 * currentAttit.q1) + (currentAttit.q2 * currentAttit.q2))));
            break;

        case Y_AXIS:  //pitch
            raw  = twice((currentAttit.q1 * currentAttit.q3) - (currentAttit.q0 * currentAttit.q2));
            safe = fmaxf(-1.0F, fminf(1.0F, raw));  //make sure to clamp the asin() value between [-1, 1]
            return asinf(safe);
            break;

        case Z_AXIS:
        case NB_AXIS:
        default:
            return 0.0F;
            break;
    };
}

/**
 * Get the angle in [rad] along the current quaternion attitude axis
 *
 * @return Angle in [rad]
 */
float getAttitudeAngle(void) {
    return twice(acosf(currentAttit.q0));
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
 * Check if a norm is invalid, i.e. outside of [0.85, 1.15]
 *
 * @param norm Norm to check
 * @retval 0 The norm is valid
 * @retval 1 The norm is invalid
 */
static inline uint8_t normInvalid(const float norm) {
    return ((norm < 0.85F) || (norm > 1.15F));  // NOLINT(*-magic-numbers)
}
