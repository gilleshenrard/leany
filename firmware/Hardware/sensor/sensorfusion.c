/**
 * @file sensorfusion.c
 * @details Implement a sensor fusion algorithm, to mix up accelerometer and gyroscope measurements
 * @author Gilles Henrard
 * @date 14/06/2025
 */
#include "sensorfusion.h"
#include <math.h>
#include <stdint.h>
#include "memsBMI270.h"

enum {
    QUATER_ALIGNMENT = 16U,  ///< Memory alignment of the quaternion structure
    GAIN_ALIGNMENT   = 16U,  ///< Memory alignment of the quaternion structure
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

/**
 * Structure depicting the values of a scheduled gain
 */
typedef struct {
    float slow;       ///< Gain to apply on a slow movement
    float fast;       ///< Gain to apply on a fast movement
    float threshold;  ///< Threshold
} __attribute__((aligned(GAIN_ALIGNMENT))) scheduledgain_t;

/**
 * Enumeration of gain parameters used in sensor fusion PID
 */
typedef enum {
    KP = 0,   ///< Proportional gain
    KI,       ///< Integral gain
    NB_GAINS  ///< Number of gain parameters
} pidgain_e;

// utility functions
static inline __attribute((always_inline)) float half(float number);
static inline __attribute((always_inline)) float twice(float number);
static inline __attribute((always_inline)) float squared(float number);
static inline uint8_t                            normInvalid(float norm);
static inline float                              computeScheduledGain(float rate, scheduledgain_t* gains);

//constants
static const float CLOSE_TO_ZERO       = 1e-3F;    ///< Value used to compare floats to 0
const float        MIN_ALIGNMENT_15DEG = 0.9659F;  ///< Threshold for acceptable align. between accel and estimates

/**
 * Array of scheduled gains used
 */
static const scheduledgain_t scheduledGains[NB_GAINS] = {
    [KP] = {.slow = 0.5F, .fast = 2.5F,  .threshold = 1.5F},
    [KI] = {.slow = 0.5F, .fast = 0.5F, .threshold = 0.35F},
};

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
    //normalise accelerometer measurements
    float norm =
        sqrtf(squared(accelerometer_G[X_AXIS]) + squared(accelerometer_G[Y_AXIS]) + squared(accelerometer_G[Z_AXIS]));
    if(norm < CLOSE_TO_ZERO) {
        return;
    }
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
    if((dotProduct > MIN_ALIGNMENT_15DEG) && !normInvalid(norm)) {
        xError = ((accelerometer_G[Y_AXIS] * zBodyEstimate) - (accelerometer_G[Z_AXIS] * yBodyEstimate));
        yError = ((accelerometer_G[Z_AXIS] * xBodyEstimate) - (accelerometer_G[X_AXIS] * zBodyEstimate));
        zError = ((accelerometer_G[X_AXIS] * yBodyEstimate) - (accelerometer_G[Y_AXIS] * xBodyEstimate));
    }

    norm =
        sqrtf(squared(gyroscope_radps[X_AXIS]) + squared(gyroscope_radps[Y_AXIS]) + squared(gyroscope_radps[Z_AXIS]));
    const float proportionalGain = computeScheduledGain(norm, (scheduledgain_t*)&scheduledGains[KP]);
    const float integralGain =
        ((norm < scheduledGains[KI].threshold) ? computeScheduledGain(norm, (scheduledgain_t*)&scheduledGains[KI])
                                               : 0.0F);

    //apply the integral gain to the error measured
    // (avoid if gain is 0 to avoid integral windup due to float 0.0F)
    if(integralGain > CLOSE_TO_ZERO) {
        integratedErrors[X_AXIS] += (integralGain * xError * dtPeriod_sec);
        integratedErrors[Y_AXIS] += (integralGain * yError * dtPeriod_sec);
        integratedErrors[Z_AXIS] += (integralGain * zError * dtPeriod_sec);
    }

    //apply the PI-filtered error to the gyroscope measurements
    const float correctedGyro_radps[NB_AXIS] = {
        [X_AXIS] = (gyroscope_radps[X_AXIS] + (proportionalGain * xError) + integratedErrors[X_AXIS]),
        [Y_AXIS] = (gyroscope_radps[Y_AXIS] + (proportionalGain * yError) + integratedErrors[Y_AXIS]),
        [Z_AXIS] = (gyroscope_radps[Z_AXIS] + (proportionalGain * zError) + integratedErrors[Z_AXIS])};

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
            break;

        case Y_AXIS:  //pitch
            rawSin  = twice((currentAttit.q1 * currentAttit.q3) - (currentAttit.q0 * currentAttit.q2));
            safeSin = fmaxf(-1.0F, fminf(1.0F, rawSin));  //make sure to clamp the sin value between [-1, 1]
            return asinf(safeSin);
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
 * @details This is used to filter out linear accelerations, and keep only rotary ones
 *
 * @param norm Norm to check
 * @retval 0 The norm is valid
 * @retval 1 The norm is invalid
 */
static inline uint8_t normInvalid(const float norm) {
    return ((norm < 0.85F) || (norm > 1.15F));  // NOLINT(*-magic-numbers)
}

/**
 * Apply gain scheduling to a measured rate
 * @details The rate can be either a linear acceleration in [G], or an angular rate in [rad/s].
 *  Depending on the rate measured, a gain will be computed in a min/max range.
 *  This allows for low gains when static or slowly moving (more precision, less jitter),
 *  and more reactivity when moving fast
 *
 * @param rate Rate measured on an instrument (accelerometer or gyroscope)
 * @param gains Scheduled gains to use
 * @return Scheduled KI or KP gain
 */
static inline float computeScheduledGain(float rate, scheduledgain_t* gains) {
    //make sure the rate does not cross the threshold
    if(rate >= gains->threshold) {
        return gains->fast;
    }

    //compute the scaled gain
    const float scaledGain = gains->slow + ((rate / gains->threshold) * (gains->fast - gains->slow));
    return ((scaledGain > gains->fast) ? gains->fast : scaledGain);
}
