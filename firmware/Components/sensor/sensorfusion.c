#include "sensorfusion.h"
#include <math.h>
#include "FreeRTOS.h"  // IWYU pragma: keep
#include "memsBMI270.h"
#include "semphr.h"

/**
 * @brief Compute a complementary filter on accelerometer/gyroscope values
 * 
 * @param[in] accelerometer_G    Array of acceleration values in [G] on all axis
 * @param[in] gyroscope_radps       Array of gyroscope values  in [rad/s] on X and Y axis
 * @param[out] filteredAngles_rad     Array of final angle values in [rad] on X and Y axis
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void complementaryFilter(const float accelerometer_G[], const float gyroscope_radps[], float filteredAngles_rad[]) {
    const float ANGLE_DELTA_MINIMUM   = 0.001745329F;  //0.1° in radians
    const float alpha                 = 0.005F;    ///< Proportion applied to the gyro. and accel. in the final result
    const float dtPeriod_sec          = 0.000625F;  ///< Time period between two updates (BMI270 config. at 1600Hz)
    float       AccelEstimatedX_rad   = 0.0F;      ///< Estimated accelerator angle on the X axis in [rad]
    float       AccelEstimatedY_rad   = 0.0F;      ///< Estimated accelerator angle on the Y axis in [rad]
    float       eulerAngleRateX_radps = 0.0F;  ///< Euler angle rate (with reference to Earth) around X axis in rad/s
    float       eulerAngleRateY_radps = 0.0F;  ///< Euler angle rate (with reference to Earth) around Y axis in rad/s

    //calculate the accelerometer roll (X) angle estimations
    AccelEstimatedX_rad = asinf(accelerometer_G[X_AXIS]);

    //calculate the accelerometer pitch (Y) angle estimations + be careful around 90° angles
    if(fabsf(accelerometer_G[Z_AXIS]) >= ANGLE_DELTA_MINIMUM) {
        AccelEstimatedY_rad = atanf(accelerometer_G[Y_AXIS] / accelerometer_G[Z_AXIS]);
    } else {
        const float HALF_PI = 1.57079632679489661923F;
        const float sign    = ((accelerometer_G[Y_AXIS] > 0.0F) ? 1.0F : -1.0F);
        AccelEstimatedY_rad = sign * HALF_PI;
    }

    //Transform gyroscope rates (reference is the solid body) to Euler rates (reference is Earth)
    eulerAngleRateX_radps =
        gyroscope_radps[X_AXIS]
        + (sinf(filteredAngles_rad[X_AXIS]) * tanf(filteredAngles_rad[Y_AXIS]) * gyroscope_radps[Y_AXIS])
        + (cosf(filteredAngles_rad[X_AXIS]) * tanf(filteredAngles_rad[Y_AXIS]) * gyroscope_radps[Z_AXIS]);

    eulerAngleRateY_radps = (cosf(filteredAngles_rad[X_AXIS]) * gyroscope_radps[Y_AXIS])
                            - (sinf(filteredAngles_rad[X_AXIS]) * gyroscope_radps[Z_AXIS]);

    //combine accelerometer estimates with Euler angle rates estimates
    filteredAngles_rad[X_AXIS] =
        ((1.0F - alpha) * (filteredAngles_rad[X_AXIS] + (eulerAngleRateX_radps * dtPeriod_sec)))
        + (alpha * AccelEstimatedX_rad);
    filteredAngles_rad[Y_AXIS] =
        ((1.0F - alpha) * (filteredAngles_rad[Y_AXIS] + (eulerAngleRateY_radps * dtPeriod_sec)))
        + (alpha * AccelEstimatedY_rad);
}
