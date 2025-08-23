/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "complementaryfilter.h"

#include <math.h>

#define FORCE_INLINE_SILENT __attribute((always_inline))  ///< Macro used to workaround Doxygen issues with __attribute
#if defined(__DOXYGEN__) && __DOXYGEN__ == 1
#define FORCE_INLINE_SILENT
#endif

static inline FORCE_INLINE_SILENT float clamp(float value, float max_absolute_value);

/**
 * @brief Compute a complementary filter on accelerometer/gyroscope values
 * 
 * @param[in] sample                    Latest IMU sample
 * @param[out] filtered_angles_rad      Array of final angle values in [rad] on X and Y axis
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void complementaryFilter(const IMUsample* sample, float filtered_angles_rad[]) {
    static const float kAlpha = 0.02F;  ///< Proportion applied to the gyro. and accel. in the final result
    static const float kDtPeriodSeconds = 0.00240385F;  ///< Time period between two updates (LSM6DSO config. at 416Hz)
    float accelx_astimated_rad = 0.0F;                  ///< Estimated accelerator angle on the X axis in [rad]
    float accely_astimated_rad = 0.0F;                  ///< Estimated accelerator angle on the Y axis in [rad]
    float euler_angleratex_radps = 0.0F;  ///< Euler angle rate (with reference to Earth) around X axis in rad/s
    float euler_angleratey_radps = 0.0F;  ///< Euler angle rate (with reference to Earth) around Y axis in rad/s

    //calculate the accelerometer angle estimations
    accelx_astimated_rad = asinf(clamp(sample->accelerometer_g[kXaxis], 1.0F));
    accely_astimated_rad = atanf(clamp(sample->accelerometer_g[kYaxis] / sample->accelerometer_g[kZaxis], 1.0F));

    //take the measurements mutex before updating angle values
    // if (xSemaphoreTake(anglesMutex, pdMS_TO_TICKS(SPI_TIMEOUT_MS)) == pdFALSE) {
    //     return;
    // }

    //Transform gyroscope rates (reference is the solid body) to Euler rates (reference is Earth)
    euler_angleratex_radps =
        sample->gyroscope_radps[kXaxis] +
        (sinf(filtered_angles_rad[kXaxis]) * tanf(filtered_angles_rad[kYaxis]) * sample->gyroscope_radps[kYaxis]) +
        (cosf(filtered_angles_rad[kXaxis]) * tanf(filtered_angles_rad[kYaxis]) * sample->gyroscope_radps[kZaxis]);

    euler_angleratey_radps = (cosf(filtered_angles_rad[kXaxis]) * sample->gyroscope_radps[kYaxis]) -
                             (sinf(filtered_angles_rad[kXaxis]) * sample->gyroscope_radps[kZaxis]);

    //combine accelerometer estimates with Euler angle rates estimates
    filtered_angles_rad[kXaxis] =
        ((1.0F - kAlpha) * (filtered_angles_rad[kXaxis] + (euler_angleratex_radps * kDtPeriodSeconds))) +
        (kAlpha * accelx_astimated_rad);
    filtered_angles_rad[kYaxis] =
        ((1.0F - kAlpha) * (filtered_angles_rad[kYaxis] + (euler_angleratey_radps * kDtPeriodSeconds))) +
        (kAlpha * accely_astimated_rad);

    //release the mutex
    // xSemaphoreGive(anglesMutex);
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
