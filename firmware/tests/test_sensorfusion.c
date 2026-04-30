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

//private functions
static void iterate_filter(MahonyContext* filter_context, const IMUsample* sample, uint32_t steps);
static float quat_norm(const Quaternion* quat);
static uint8_t isContextReset(const MahonyContext* filter_context);
static void test_null_pointer_guards(void);
static void test_tick_handles_overflow(void);
static void test_bad_samples_counter_resets_correctly(void);
static void test_yaw_angle_returns_0(void);
static void test_correct_attitude_angle_calculation(void);
static void test_controller_no_shift_at_rest(void);
static void test_gyro_integration_accumulates_correctly(void);
static void test_normalisation_prevents_drift_under_sustained_input(void);
static void test_alignment_check_freezes_update_on_lateral_accel(void);
static void test_integration_stable_at_high_angular_rate(void);

//constants
static const float kNormTolerance = 0.005F;             ///< Tolerance for quaternion norm comparisons
static const float kAngleTolerance_rad = 0.05F;         ///< Tolerance for angle comparisons in [rad] (~3 degrees)
static const float kTickPeriod_sec = 0.01F;             ///< Simulated tick period in [s]: 10ms -> 100 Hz update rate
static const float kPI_F = 3.14159265358979323846F;     ///< Pi, as a float value
static const uint32_t kMaxTick = UINT32_MAX;            ///< Maximum value a system tick can take
static const float kStrongGyro_radps = (4.0F * kPI_F);  ///< Strong rotation speed

//state variables
static MahonyContext context;                                             ///< Filter context used during tests
static uint32_t current_tick;                                             ///< Simulated application tick
static const IMUsample kPureGravity = {.accelerometer_g[kZaxis] = 1.0F};  ///< Sample pointing towards gravity

/*********************************************************************************************************************************/
// PUBLIC FUNCTIONS
/*********************************************************************************************************************************/

/**
 * Tests runner
 *
 * @return UNITY_END result 
 */
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_null_pointer_guards);
    RUN_TEST(test_tick_handles_overflow);
    RUN_TEST(test_bad_samples_counter_resets_correctly);
    RUN_TEST(test_yaw_angle_returns_0);
    RUN_TEST(test_correct_attitude_angle_calculation);
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
    context.dt.max_tick = kMaxTick;
    context.dt.current_tick = 0U;
    context.dt.previous_tick = 0U;
    context.align_check_enabled = 0U;

    current_tick = 1U;
}

/**
 * Free up the resources used during each test
 */
void tearDown(void) {}

/*********************************************************************************************************************************/
// TEST FUNCTIONS
/*********************************************************************************************************************************/

/**
 * Test that NULL parameters do not modify the filter context.
 *
 * @details
 * This is achieved by calling updateMahonyFilter() three times: once with a
 * NULL context, once with a NULL sample, and once with valid parameters. Under
 * the first two conditions the context must remain bit-identical to its initial
 * state; under the third it must differ, proving the valid path does update.
 * Example: updateMahonyFilter(NULL, &sample) → context unchanged.
 *
 * @internal
 * Exercises the NULL guard at the entry of updateMahonyFilter(). The third
 * call verifies the guard does not over-reject valid input, making it a
 * combined positive and negative test.
 */
static void test_null_pointer_guards(void) {
    const MahonyContext old_context = context;
    const IMUsample violent_pitch = {
        .accelerometer_g[kZaxis] = 1.0F,
        .gyroscope_radps[kYaxis] = kStrongGyro_radps,
    };

    //test with a NULL context
    updateMahonyFilter(NULL, &violent_pitch);
    // NOLINTNEXTLINE (DeprecatedOrUnsafeBufferHandling)
    uint8_t equals = (memcmp(&old_context, &context, sizeof(MahonyContext)) == 0);
    TEST_ASSERT_TRUE_MESSAGE(equals, "NULL context failed");

    //test with a NULL sample
    updateMahonyFilter(&context, NULL);
    // NOLINTNEXTLINE (DeprecatedOrUnsafeBufferHandling)
    equals = (memcmp(&old_context, &context, sizeof(MahonyContext)) == 0);
    TEST_ASSERT_TRUE_MESSAGE(equals, "NULL sample failed");

    //test in normal conditions
    context.dt.current_tick++;
    updateMahonyFilter(&context, &violent_pitch);
    // NOLINTNEXTLINE (DeprecatedOrUnsafeBufferHandling)
    equals = (memcmp(&old_context, &context, sizeof(MahonyContext)) == 0);
    TEST_ASSERT_FALSE_MESSAGE(equals, "Normal update condition failed");
}

/**
 * Test that tick wraparound does not trigger a spurious filter reset.
 *
 * @details
 * This is achieved by setting previous_tick near UINT32_MAX and current_tick
 * near zero, then running one update. Under these conditions getDT() computes
 * the correct elapsed time via bitmask subtraction rather than overflowing.
 * Example: previous_tick = UINT32_MAX - 5, current_tick = 3 →
 * delta = (3 - (UINT32_MAX - 5)) & UINT32_MAX = 9 ticks = 0.09s (valid).
 * Also verifies that dT=0 and dT>5s each trigger a reset.
 *
 * @internal
 * Exercises the bitmask subtraction in getDT() and the bounds check in
 * isDTvalid(). The dT=0 and dT>5s sub-cases exercise the reset branch in
 * updateMahonyFilter(), which currently lacks a return after the reset call —
 * those sub-cases are expected to fail until that bug is fixed.
 */
static void test_tick_handles_overflow(void) {
    const IMUsample violent_pitch = {
        .accelerometer_g[kZaxis] = 1.0F,
        .gyroscope_radps[kYaxis] = kStrongGyro_radps,
    };

    // Test tick wraparound -> no reset
    // Tilt the quaternion slightly so a valid update and a reset produce
    // distinguishable states — identity after reset vs. rotated after update
    context.attitude.q1 = 0.1F;
    context.dt.previous_tick = (kMaxTick - 5U);  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
    context.dt.current_tick = 3U;
    updateMahonyFilter(&context, &violent_pitch);
    TEST_ASSERT_FALSE_MESSAGE(isContextReset(&context), "Correct wraparound context failed");

    //test dT = 0 ticks -> filter reset
    context.dt.previous_tick = context.dt.current_tick;
    updateMahonyFilter(&context, &violent_pitch);
    TEST_ASSERT_TRUE_MESSAGE(isContextReset(&context), "dT 0s context failed");

    //test dT > 5s -> filter reset
    const uint32_t tick_dt_5s = (uint32_t)(5.0F / kTickPeriod_sec);
    context.dt.previous_tick = 0;
    context.dt.current_tick = tick_dt_5s;
    updateMahonyFilter(&context, &violent_pitch);
    TEST_ASSERT_TRUE_MESSAGE(isContextReset(&context), "dT > 5s context failed");
}

/**
 * Test that the bad sample counter resets the filter at threshold and
 * clears on valid input before threshold.
 *
 * @details
 * This is achieved in two sub-cases. First, kMaxBadCounts consecutive
 * out-of-range acceleration samples are fed — the filter must reset to
 * identity on the final one. Second, (kMaxBadCounts - 1) bad samples followed
 * by one valid sample are fed — the counter must clear without triggering a
 * reset. Example: 5 bad samples → q0=1, integrals=0; 4 bad + 1 good →
 * filter not reset, bad_acceleration_count=0.
 *
 * @internal
 * Exercises the increment and threshold logic in validateNorm(), and the
 * counter-clear path on the first valid sample after a bad streak. The
 * second sub-case specifically guards against off-by-one errors in the
 * threshold comparison.
 */
static void test_bad_samples_counter_resets_correctly(void) {
    const IMUsample bad_sample = {.accelerometer_g[kXaxis] = 5.0F};

    //enable alignment check and feed the maximum number of bad acceleration values
    context.dt.current_tick = 1U;
    context.align_check_enabled = 1U;
    for (uint8_t attempt = 0; attempt < kMaxBadCounts; attempt++) {
        updateMahonyFilter(&context, &bad_sample);
    }
    TEST_ASSERT_TRUE_MESSAGE(isContextReset(&context), "Maximum bad attempts test failed");

    //reset the context and feed (max - 1) bad values, then one good
    setUp();
    context.dt.current_tick = 1U;
    context.align_check_enabled = 1U;
    for (uint8_t attempt = 0; attempt < (kMaxBadCounts - 1U); attempt++) {
        updateMahonyFilter(&context, &bad_sample);
    }
    updateMahonyFilter(&context, &kPureGravity);
    TEST_ASSERT_FALSE_MESSAGE(isContextReset(&context), "Recovery without reset failed");
}

/**
 * Test that yaw angle always returns zero regardless of orientation.
 *
 * @details
 * This is achieved by reading angleAlongAxis() on the Z axis from a freshly
 * reset context. Without a magnetometer, heading is undefined and must never
 * be estimated. Example: any quaternion → angleAlongAxis(kZaxis) = 0.0F.
 *
 * @internal
 * Exercises the kZaxis case in the switch statement of angleAlongAxis(),
 * which unconditionally returns 0.0F. Documents the absence of yaw
 * estimation as an explicit contract rather than an untested assumption.
 */
static void test_yaw_angle_returns_0(void) {
    // NOLINTNEXTLINE (cppcoreguidelines-avoid-magic-numbers)
    TEST_ASSERT_EQUAL_FLOAT(0.0F, angleAlongAxis(&context, kZaxis));
}

/**
 * Test that getAttitudeAngle returns zero at identity orientation.
 *
 * @details
 * This is achieved by calling getAttitudeAngle() on a freshly reset context.
 * After resetMahonyFilter(), q0=1 and the rotation angle around the attitude
 * axis is 2*acos(1) = 0. Example: q=[1,0,0,0] → getAttitudeAngle() = 0.0F.
 *
 * @internal
 * Exercises getAttitudeAngle() at its baseline boundary condition. Verifies
 * the acos path and the clamp applied to q0 do not introduce any offset at
 * the identity quaternion.
 */
static void test_correct_attitude_angle_calculation(void) {
    // NOLINTNEXTLINE (cppcoreguidelines-avoid-magic-numbers)
    TEST_ASSERT_EQUAL_FLOAT(0.0F, getAttitudeAngle(&context));
}

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
static void test_controller_no_shift_at_rest(void) {
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
static void test_gyro_integration_accumulates_correctly(void) {
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
static void test_normalisation_prevents_drift_under_sustained_input(void) {
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
static void test_alignment_check_freezes_update_on_lateral_accel(void) {
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
 * into a freshly reset filter. Under these conditions, each Euler integration step adds
 * a large delta to the quaternion components before normalisation corrects it.
 * Example: at 4Pi rad/s with dt=0.01s, each step adds ~0.063 to a component before
 * normaliseQuaternion() pulls the norm back to 1.
 *
 * @internal
 * Exercises normaliseQuaternion() under near-worst-case integration stress. Also
 * verifies that the PI correction path in applyProportionate() does not freeze the
 * update under high rates — the pitch must change by at least 0.1 rad over 0.5s.
 * This threshold was derived empirically with kp=2.5, ki=0.5: the accel correction
 * actively counters the rotation, limiting the actual pitch change to ~0.168 rad.
 */
static void test_integration_stable_at_high_angular_rate(void) {
    const float min_expected_pitch_change_rad = 0.1F;
    const float pitch_before = angleAlongAxis(&context, kYaxis);

    // Apply violent pitch rotation for 0.5 s (50 steps at 100 Hz)
    const IMUsample violent_pitch = {
        .accelerometer_g[kZaxis] = 1.0F,
        .gyroscope_radps[kYaxis] = kStrongGyro_radps,
    };
    iterate_filter(&context, &violent_pitch, kHighRateSteps);

    const float pitch_after = angleAlongAxis(&context, kYaxis);

    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&context.attitude));

    // The pitch must have moved by at least 0.5 rad — the filter is tracking
    TEST_ASSERT_GREATER_THAN_FLOAT(min_expected_pitch_change_rad, fabsf(pitch_after - pitch_before));
}

/*********************************************************************************************************************************/
// HELPER FUNCTIONS
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
 * @param[in] steps Number of update cycles to run.
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

/**
 * Check whether the filter context is in a reset state.
 *
 * @details
 * A reset context has: attitude quaternion = identity [1,0,0,0], all error
 * integrals = 0, and both bad sample counters = 0. These are exactly the
 * fields touched by resetMahonyFilter().
 *
 * @param filter_context Filter context to inspect
 * @retval 1 Context matches a reset state
 * @retval 0 Context has diverged from a reset state
 */
static uint8_t isContextReset(const MahonyContext* filter_context) {
    const float default_integrals[kNBaxis] = {0.0F, 0.0F, 0.0F};
    const Quaternion unit_quaternion = {.q0 = 1.0F, .q1 = 0.0F, .q2 = 0.0F, .q3 = 0.0F};

    // NOLINTBEGIN (DeprecatedOrUnsafeBufferHandling)
    const uint8_t quat_resetted = (memcmp(&filter_context->attitude, &unit_quaternion, sizeof(Quaternion)) == 0);
    const uint8_t integrals_resetted = (memcmp(&filter_context->error_integrals, &default_integrals, (kNBaxis * sizeof(float))) == 0);
    // NOLINTEND

    return (quat_resetted && integrals_resetted && (filter_context->bad_acceleration_count == 0) && (filter_context->bad_quaternion_count == 0));
}
