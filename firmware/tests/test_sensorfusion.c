/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file test_sensorfusion.c
 * @brief Unit tests for the Mahony AHRS filter using simulated IMU inputs.
 */

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>
#include <unity_internals.h>

#include "sensorfusion.h"

enum {
    kConvergenceSteps = 2000U,   ///< Number of steps for the filter to reach a stable attitude from identity
    kStepsIn1second = 100U,      ///< Steps representing exactly 1 second at 100 Hz
    kAlignmentCheckSteps = 10U,  ///< Number of steps in which alignment test is done
    kHighRateSteps = 50U,        ///< Number of steps in which high rate test is done
};

// NOLINTBEGIN (misc-use-internal-linkage)
void test_controller_no_shift_at_rest(void);
void test_gyro_integration_accumulates_correctly(void);
void test_normalisation_prevents_drift_under_sustained_input(void);
void test_alignment_check_freezes_update_on_lateral_accel(void);
void test_integration_stable_at_high_angular_rate(void);
// NOLINTEND

static void iterate_filter(MahonyContext* filter_context, const IMUsample* sample, uint32_t steps);
static float quat_norm(const Quaternion* quat);

static const float kNormTolerance = 0.005F;          ///< Tolerance for quaternion norm comparisons
static const float kAngleTolerance_rad = 0.05F;      ///< Tolerance for angle comparisons in [rad] (~3 degrees)
static const float kTickPeriod_sec = 0.01F;          ///< Simulated tick period in [s]: 10ms -> 100 Hz update rate
static const float kPI_F = 3.14159265358979323846F;  ///< Pi, as a float value

/**
 * All uint32_t bits set, used as max_tick.
 * getDT() masks with this value, making it a transparent modulo-wrap mask.
 */
#define MAX_TICK_VALUE (UINT32_MAX)

static MahonyContext context;                                             ///< Filter context used during tests
static uint32_t current_tick;                                             ///< Simulated application tick
static const IMUsample kPureGravity = {.accelerometer_g[kZaxis] = 1.0F};  ///< Sample pointing towards gravity

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Tests runner
 *
 * @return UNITY_END result 
 */
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_controller_no_shift_at_rest);
    RUN_TEST(test_gyro_integration_accumulates_correctly);
    RUN_TEST(test_normalisation_prevents_drift_under_sustained_input);
    RUN_TEST(test_alignment_check_freezes_update_on_lateral_accel);
    RUN_TEST(test_integration_stable_at_high_angular_rate);
    return UNITY_END();
}

/**
 * Initialise the filter context to a clean, known state before each test.
 *
 * @internal
 * resetMahonyFilter() only resets the quaternion, error integrals, and bad counters.
 * All other fields must be set explicitly here.
 */
void setUp(void) {
    (void)memset(&context, 0, sizeof(context));  // NOLINT (DeprecatedOrUnsafeBufferHandling)
    resetMahonyFilter(&context);

    context.kp = kProportionalGain;
    context.ki = kIntegralGain;
    context.dt.tick_period_seconds = kTickPeriod_sec;
    context.dt.max_tick = MAX_TICK_VALUE;
    context.dt.current_tick = 0U;
    context.dt.previous_tick = 0U;
    context.align_check_enabled = 0U;

    current_tick = 1U;
}

/**
 * Free up the resources used during each test
 */
void tearDown(void) {}

/**
 * Test that the PI controller induces no attitude shift at rest.
 *
 * @details
 * This is achieved by feeding 2000 steps of pure static gravity ([0, 0, 1G],
 * zero gyro) into a freshly reset filter. Under these conditions, the
 * cross-product error between measured and estimated gravity is zero from the
 * first step, so the PI controller has nothing to correct. Example: measured
 * = [0,0,1], estimated = [0,0,1] -> error = [0,0,1] × [0,0,1] = [0,0,0].
 *
 * @internal
 * Exercises the kp and ki correction paths in updateMahonyFilter(). Both are
 * skipped implicitly when the error vector is zero, not by a branch - so this
 * test verifies no spurious drift accumulates through floating-point
 * approximation of zero over 2000 iterations.
 */
void test_controller_no_shift_at_rest(void) {
    iterate_filter(&context, &kPureGravity, kConvergenceSteps);

    //make sure the attitude still points to pure gravity (within acceptable range)
    TEST_ASSERT_FLOAT_WITHIN(kAngleTolerance_rad, 0.0F, angleAlongAxis(&context, kXaxis));
    TEST_ASSERT_FLOAT_WITHIN(kAngleTolerance_rad, 0.0F, angleAlongAxis(&context, kYaxis));
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&context.attitude));
}

/**
 * Test that gyroscope integration accumulates angle correctly over time.
 *
 * @details
 * This is achieved by zeroing kp and ki, turning the filter into a pure
 * integrator, then feeding a constant roll rate of Pi/2 rad/s for exactly
 * 1 second (100 steps × 0.01s). Under these conditions the accumulated angle
 * is analytically predictable.
 *
 * @par Example
 * gyro_x = Pi/2 rad/s, dt = 0.01s, 100 steps -> roll = Pi/2 rad/s × 1s = Pi/2 rad
 *
 * @internal
 * Exercises integrateGyroMeasurements() in isolation. With kp and ki zeroed,
 * applyProportionate() adds zero correction and the integral branch is
 * skipped entirely, so the only active path is the quaternion derivative
 * and the Euler integration step.
 */
void test_gyro_integration_accumulates_correctly(void) {
    const float rate_90degrees_in_1sec = (kPI_F * 0.5F);

    // Switch to zero correction gains: the filter becomes a pure integrator
    context.kp = 0.0F;
    context.ki = 0.0F;

    // Apply roll rate for exactly 1 second
    const IMUsample roll_rate = {
        .accelerometer_g[kZaxis] = 1.0F,
        .gyroscope_radps[kXaxis] = rate_90degrees_in_1sec,
    };
    iterate_filter(&context, &roll_rate, kStepsIn1second);

    //make sure the final angle and norm are correct (within acceptable range)
    TEST_ASSERT_FLOAT_WITHIN(kAngleTolerance_rad, rate_90degrees_in_1sec, angleAlongAxis(&context, kXaxis));
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&context.attitude));
}

/**
 * Test that quaternion normalisation prevents norm drift under sustained arbitrary input.
 *
 * @details
 * This is achieved by feeding 2000 steps of off-axis acceleration and
 * multi-axis gyro into the filter. Under these conditions, first-order Euler
 * integration introduces a small norm error at every step that, left
 * uncorrected, would accumulate into significant drift.
 *
 * @par Example
 * accel = [0.1, 0.2, 0.95G], gyro = [0.1, -0.2, 0.3 rad/s] → norm must
 * remain within [0.995, 1.005] after 2000 steps
 *
 * @internal
 * Exercises normaliseQuaternion() under sustained non-trivial input.
 * Removing or breaking that call would cause this test to fail within
 * a few hundred iterations.
 */
void test_normalisation_prevents_drift_under_sustained_input(void) {
    const IMUsample skewed = {
        // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
        .accelerometer_g = {0.1F, 0.2F, 0.95F},
        .gyroscope_radps = {0.1F, -0.2F, 0.3F},
        // NOLINTEND
    };
    iterate_filter(&context, &skewed, kConvergenceSteps);

    //make sure the norm value is still within acceptable range
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&context.attitude));
}

/**
 * Test that the alignment check freezes quaternion updates on lateral acceleration.
 *
 * @details
 * This is achieved by enabling align_check_enabled after convergence, then
 * feeding 10 steps of a unit-norm but horizontally biased acceleration vector.
 * Under these conditions the dot product between measured and estimated gravity
 * falls below kMinAlignmentCosine, indicating linear motion rather than
 * gravity, and updateMahonyFilter() must return without touching the quaternion.
 *
 * @par Example
 * accel = [0.6, 0, 0.8G] → norm = 1.0 (passes validateNorm),
 * dot([0.6, 0, 0.8], [0, 0, 1]) = 0.8 < 0.9659 (fails alignmentValid) →
 * quaternion unchanged
 *
 * @internal
 * Exercises the alignmentValid() early-return path in updateMahonyFilter(),
 * which is only reached when align_check_enabled is set and the acceleration
 * norm is valid. A 5G lateral shock would be rejected earlier by validateNorm()
 * and would never reach alignmentValid().
 */
void test_alignment_check_freezes_update_on_lateral_accel(void) {
    iterate_filter(&context, &kPureGravity, kConvergenceSteps);

    context.align_check_enabled = 1U;

    // Snapshot the quaternion before injecting the misaligned samples
    const Quaternion attitude_before = context.attitude;

    /*
     * Feed 10 samples with a unit-norm but horizontally biased accel.
     * norm([0.6, 0, 0.8]) = sqrt(0.36 + 0 + 0.64) = 1.0 -> passes validateNorm.
     * dot([0.6, 0, 0.8], [0, 0, 1]) = 0.8 < 0.9659 -> fails alignmentValid.
     */
    const IMUsample unit_norm_values = {
        .accelerometer_g = {0.6F, 0.0F, 0.8F},
    };
    iterate_filter(&context, &unit_norm_values, kAlignmentCheckSteps);

    // The quaternion must be bit-identical (no update happened)
    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q0, context.attitude.q0);
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q1, context.attitude.q1);
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q2, context.attitude.q2);
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q3, context.attitude.q3);
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&context.attitude));
    // NOLINTEND
}

/**
 * Test that the quaternion integration remains stable at high angular rates.
 *
 * @details
 * This is achieved by feeding 50 steps of violent pitch rotation (4Pi rad/s, ~720°/s)
 * into a converged filter. Under these conditions, each Euler integration step adds
 * a large delta to the quaternion components before normalisation corrects it.
 * Example: at 4Pi rad/s with dt=0.01s, each step adds ~0.063 to a component before
 * normaliseQuaternion() pulls the norm back to 1.
 *
 * @internal
 * Exercises normaliseQuaternion() under near-worst-case integration stress. Also
 * verifies that the PI correction path in applyProportionate() does not freeze the
 * update under high rates - the pitch must change by at least 0.5 rad over 0.5s,
 * proving the filter is tracking rather than stalling.
 */
void test_integration_stable_at_high_angular_rate(void) {
    const float min_expected_pitch_change_rad = 0.5F;
    const float strong_gyro_radps = (4.0F * kPI_F);
    const float pitch_before = angleAlongAxis(&context, kYaxis);

    // Apply violent pitch rotation for 0.5 s (50 steps at 100 Hz)
    const IMUsample violent_pitch = {
        .accelerometer_g[kZaxis] = 1.0F,
        .gyroscope_radps[kYaxis] = strong_gyro_radps,
    };
    iterate_filter(&context, &violent_pitch, kHighRateSteps);

    const float pitch_after = angleAlongAxis(&context, kYaxis);

    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&context.attitude));

    // The pitch must have moved by at least 0.5 rad — the filter is tracking
    TEST_ASSERT_GREATER_THAN_FLOAT(min_expected_pitch_change_rad, fabsf(pitch_after - pitch_before));
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Feed N identical IMU samples into the filter.
 *
 * @details filter_context->dt.current_tick is set to the current current_tick value before
 * each call because updateMahonyFilter() derives elapsed time from
 * filter_context->dt.current_tick and filter_context->dt.previous_tick, not from sample->latest_tick.
 *
 * @param[out] filter_context Mahony filter context to update
 * @param[in] sample Constant IMU sample to feed
 * @param steps Number of update cycles to run.
 */
static void iterate_filter(MahonyContext* filter_context, const IMUsample* sample, uint32_t steps) {
    for (uint32_t i = 0U; i < steps; i++) {
        filter_context->dt.current_tick = current_tick;
        updateMahonyFilter(filter_context, sample);
        current_tick++;
    }
}

/**
 * Compute the Euclidean norm of the current attitude quaternion.
 *
 * @param quat Quaternion of which to compute the norm
 * @return Quaternion norm.
 */
static float quat_norm(const Quaternion* quat) {
    return sqrtf((quat->q0 * quat->q0) + (quat->q1 * quat->q1) + (quat->q2 * quat->q2) + (quat->q3 * quat->q3));
}