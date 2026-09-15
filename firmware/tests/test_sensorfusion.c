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

#include "mahony.h"

enum : uint16_t {
    kConvergenceSteps = 2000U,   ///< Number of steps for the filter to reach a stable attitude from identity
    kStepsIn1second = 100U,      ///< Steps representing exactly 1 second at 100 Hz
    kAlignmentCheckSteps = 10U,  ///< Number of steps in which alignment test is done
    kHighRateSteps = 50U,        ///< Number of steps in which high rate test is done
};

//private functions
static void iterate_filter(MahonyContext* filter_context, const IMUsample* sample, uint32_t steps);
static float quat_norm(const Quaternion* quat);
static bool isContextReset(const MahonyContext* filter_context);
static bool floats_bit_identical(float first, float second);
static void test_null_pointer_guards(void);
static void test_tick_handles_overflow(void);
static void test_bad_accel_samples_skipped_without_reset(void);
static void test_alignment_check_disabled_allows_update(void);
static void test_yaw_angle_returns_0(void);
static void test_correct_attitude_angle_calculation(void);
static void test_controller_no_shift_at_rest(void);
static void test_gyro_integration_accumulates_correctly(void);
static void test_normalisation_prevents_drift_under_sustained_input(void);
static void test_alignment_check_freezes_update_on_lateral_accel(void);
static void test_integration_stable_at_high_angular_rate(void);
static void test_integral_clamped_on_windup(void);
static void test_out_of_range_axis_returns_0(void);
static void test_bad_quaternion_norm_triggers_reset(void);

//constants
static constexpr float kNormTolerance = 0.005F;          ///< Tolerance for quaternion norm comparisons
static constexpr float kAngleTolerance_rad = 0.05F;      ///< Tolerance for angle comparisons in [rad] (~3 degrees)
static constexpr float kTickPeriod_sec = 0.01F;          ///< Simulated tick period in [s]: 10ms -> 100 Hz update rate
static constexpr float kPI_F = 3.14159265358979323846F;  ///< Pi, as a float value
static constexpr uint32_t kMaxTick = UINT32_MAX;         ///< Maximum value a system tick can take
static constexpr float kStrongGyro_radps = (4.0F * kPI_F);  ///< Strong rotation speed

//state variables
static MahonyContext context;                                                 ///< Filter context used during tests
static uint32_t current_tick;                                                 ///< Simulated application tick
static constexpr IMUsample kPureGravity = {.accelerometer_g[kZaxis] = 1.0F};  ///< Sample pointing towards gravity

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
    RUN_TEST(test_bad_accel_samples_skipped_without_reset);
    RUN_TEST(test_alignment_check_disabled_allows_update);
    RUN_TEST(test_yaw_angle_returns_0);
    RUN_TEST(test_correct_attitude_angle_calculation);
    RUN_TEST(test_controller_no_shift_at_rest);
    RUN_TEST(test_gyro_integration_accumulates_correctly);
    RUN_TEST(test_normalisation_prevents_drift_under_sustained_input);
    RUN_TEST(test_alignment_check_freezes_update_on_lateral_accel);
    RUN_TEST(test_integration_stable_at_high_angular_rate);
    RUN_TEST(test_integral_clamped_on_windup);
    RUN_TEST(test_out_of_range_axis_returns_0);
    RUN_TEST(test_bad_quaternion_norm_triggers_reset);
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
    context.dt.last_sampled_tick = 0U;
    context.dt.last_valid_tick = 0U;
    context.alignment_check_enabled = false;

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
 * Test that nullptr parameters do not modify the filter context.
 *
 * @details
 * This is achieved by calling updateMahonyFilter() three times: once with a
 * nullptr context, once with a nullptr sample, and once with valid parameters. Under
 * the first two conditions the context must remain bit-identical to its initial
 * state; under the third it must differ, proving the valid path does update.
 * Example: updateMahonyFilter(nullptr, &sample) → context unchanged.
 *
 * @internal
 * Exercises the nullptr guard at the entry of updateMahonyFilter(). The third
 * call verifies the guard does not over-reject valid input, making it a
 * combined positive and negative test.
 */
static void test_null_pointer_guards(void) {
    const MahonyContext old_context = context;
    const IMUsample violent_pitch = {
        .accelerometer_g[kZaxis] = 1.0F,
        .gyroscope_radps[kYaxis] = kStrongGyro_radps,
    };

    //test with a nullptr context
    updateMahonyFilter(nullptr, &violent_pitch);
    // NOLINTNEXTLINE (DeprecatedOrUnsafeBufferHandling)
    uint8_t equals = (memcmp(&old_context, &context, sizeof(MahonyContext)) == 0);
    TEST_ASSERT_TRUE_MESSAGE(equals, "nullptr context failed");

    //test with a nullptr sample
    updateMahonyFilter(&context, nullptr);
    // NOLINTNEXTLINE (DeprecatedOrUnsafeBufferHandling)
    equals = (memcmp(&old_context, &context, sizeof(MahonyContext)) == 0);
    TEST_ASSERT_TRUE_MESSAGE(equals, "nullptr sample failed");

    //test in normal conditions
    context.dt.last_sampled_tick++;
    updateMahonyFilter(&context, &violent_pitch);
    // NOLINTNEXTLINE (DeprecatedOrUnsafeBufferHandling)
    equals = (memcmp(&old_context, &context, sizeof(MahonyContext)) == 0);
    TEST_ASSERT_FALSE_MESSAGE(equals, "Normal update condition failed");

    //test getting an angles with a nullptr context
    // NOLINTBEGIN (readability-magic-numbers)
    TEST_ASSERT_EQUAL_FLOAT(0.0F, angleAlongAxis(nullptr, kXaxis));
    TEST_ASSERT_EQUAL_FLOAT(0.0F, getAttitudeAngle(nullptr));
    // NOLINTEND (readability-magic-numbers)
}

/**
 * Test that tick wraparound does not trigger a spurious filter reset.
 *
 * @details
 * This is achieved by setting last_valid_tick near UINT32_MAX and last_sampled_tick
 * near zero, then running one update. Under these conditions getDT() computes
 * the correct elapsed time via bitmask subtraction rather than overflowing.
 * Example: last_valid_tick = UINT32_MAX - 5, last_sampled_tick = 3 →
 * delta = (3 - (UINT32_MAX - 5)) & UINT32_MAX = 9 ticks = 0.09s (valid).
 * Also verifies that dT=0 and dT>5s each trigger a reset.
 */
static void test_tick_handles_overflow(void) {
    const IMUsample violent_pitch = {
        .accelerometer_g[kZaxis] = 1.0F,
        .gyroscope_radps[kYaxis] = kStrongGyro_radps,
    };

    // Test tick wraparound -> no reset
    // Tilt the quaternion slightly so a valid update and a reset produce
    // distinguishable states — identity after reset vs. rotated after update
    // NOLINTNEXTLINE (readability-magic-numbers)
    context.attitude.q1 = 0.1F;
    context.dt.last_valid_tick = (kMaxTick - 5U);  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
    context.dt.last_sampled_tick = 3U;
    updateMahonyFilter(&context, &violent_pitch);
    TEST_ASSERT_FALSE_MESSAGE(isContextReset(&context), "Correct wraparound context failed");

    //test dT = 0 ticks -> filter reset
    context.dt.last_valid_tick = context.dt.last_sampled_tick;
    updateMahonyFilter(&context, &violent_pitch);
    TEST_ASSERT_TRUE_MESSAGE(isContextReset(&context), "dT 0s context failed");

    //test dT > 5s -> filter reset
    const uint32_t tick_dt_5s = (uint32_t)(5.0F / kTickPeriod_sec);
    context.dt.last_valid_tick = 0;
    context.dt.last_sampled_tick = tick_dt_5s;
    updateMahonyFilter(&context, &violent_pitch);
    TEST_ASSERT_TRUE_MESSAGE(isContextReset(&context), "dT > 5s context failed");
}

/**
 * Test that consecutive invalid accelerometer samples are skipped without
 * ever resetting or otherwise touching the filter state, and that a
 * subsequent valid sample updates normally afterward.
 *
 * @details
 * This is achieved by feeding an out-of-range accelerometer sample for
 * several consecutive updates, then one valid sample. Under these
 * conditions the attitude and integrals must remain bit-identical during
 * the bad streak, since updateMahonyFilter() returns before touching them,
 * and the following valid sample must update as normal.
 * Example: accel=[5,0,0] -> norm=5.0, |5.0-1.0| > kMaxNormEpsilon ->
 * update skipped, attitude unchanged; accel=[0,0,1] -> norm=1.0 -> updates.
 *
 * @internal
 * Exercises the early-return path in updateMahonyFilter() guarded by
 * normValid(), now that it is a pure check with no counter or reset
 * attached. Replaces the old counter/threshold test, which asserted a
 * reset that no longer occurs on this path and only passed because the
 * untouched attitude happened to already be identity.
 */
static void test_bad_accel_samples_skipped_without_reset(void) {
    const IMUsample bad_sample = {.accelerometer_g[kXaxis] = 5.0F};  // NOLINT(readability-magic-numbers)
    const uint8_t bad_streak_length = 10U;                           // NOLINT(readability-magic-numbers)

    // tilt the attitude so "untouched" is distinguishable from "coincidentally identity"
    context.attitude.q1 = 0.1F;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
    const Quaternion attitude_before = context.attitude;
    context.alignment_check_enabled = true;
    context.dt.last_sampled_tick = 1U;

    iterate_filter(&context, &bad_sample, bad_streak_length);

    const bool unchanged = (bool)(floats_bit_identical(attitude_before.q0, context.attitude.q0) &&
                                  floats_bit_identical(attitude_before.q1, context.attitude.q1) &&
                                  floats_bit_identical(attitude_before.q2, context.attitude.q2) &&
                                  floats_bit_identical(attitude_before.q3, context.attitude.q3));
    TEST_ASSERT_TRUE_MESSAGE(unchanged, "Bad accel streak modified the attitude");

    const bool updated = updateMahonyFilter(&context, &kPureGravity);
    TEST_ASSERT_TRUE_MESSAGE(updated, "Valid sample after bad streak failed to update");
}

/**
 * @brief Test that disabling the alignment check allows misaligned acceleration updates.
 *
 * @details
 * This is achieved by feeding a unit-norm but horizontally biased acceleration
 * vector with alignment_check_enabled=0. Under these conditions validateNorm passes
 * and alignmentValid is never called, so the filter must update.
 * Example: accel=[0.6, 0, 0.8G] → norm=1.0 (passes validateNorm),
 * dot=0.8 < 0.9659 (would fail alignmentValid if enabled) → filter updates.
 *
 * @internal
 * Exercises the alignment_check_enabled guard in updateMahonyFilter(), which
 * short-circuits the alignmentValid() call entirely when cleared.
 * Complements test_alignment_check_freezes_update_on_lateral_accel.
 */
static void test_alignment_check_disabled_allows_update(void) {
    const IMUsample misaligned = {.accelerometer_g = {0.6F, 0.0F, 0.8F}};
    const Quaternion attitude_before = context.attitude;
    context.dt.last_sampled_tick = 1U;

    updateMahonyFilter(&context, &misaligned);
    // NOLINTNEXTLINE (readability-magic-numbers)
    TEST_ASSERT_NOT_EQUAL_FLOAT(attitude_before.q0, context.attitude.q0);
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
 * This is achieved by enabling alignment_check_enabled after convergence, then
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
 * which is only reached when alignment_check_enabled is set and the acceleration
 * norm is valid. A 5G lateral shock would be rejected earlier by validateNorm()
 * and would never reach alignmentValid().
 */
static void test_alignment_check_freezes_update_on_lateral_accel(void) {
    iterate_filter(&context, &kPureGravity, kConvergenceSteps);

    context.alignment_check_enabled = true;

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

    // The pitch must have moved by at least 0.1 rad — the filter is tracking
    TEST_ASSERT_GREATER_THAN_FLOAT(min_expected_pitch_change_rad, fabsf(pitch_after - pitch_before));
}

/**
 * Test that integral terms are clamped to prevent windup under sustained error.
 *
 * @details
 * This is achieved by zeroing kp and setting a high ki, then feeding a purely
 * lateral acceleration vector for 10 steps. Under these conditions the cross-product
 * error on the Y axis is large and non-zero from identity, driving the integral to
 * saturation within the first step.
 * Example: accel=[1,0,0], body_estimates=[0,0,1] → error[Y] = 0*0 - 1*1 = -1.0 →
 * integral[Y] += 100 * (-1.0) * 0.01s = -1.0 → clamped to -0.3.
 *
 * @internal
 * Exercises the clamp() call applied to each error_integrals[axis] inside
 * updateMahonyFilter(). kExpectedMaxIntegral mirrors the private kMaxIntegralError
 * constant in mahony.c — any change to that constant must be reflected here.
 * kp=0 removes the proportional path so only the integral branch is active.
 */
static void test_integral_clamped_on_windup(void) {
    // mirrors kMaxIntegralError in mahony.c
    static constexpr float kExpectedMaxIntegral = 0.3F;
    // ki > kMaxIntegralError / (|error| * dt) = 0.3 / (1.0 * 0.01) = 30 guarantees
    // saturation in one step; 100 gives safe margin
    static constexpr float kHighKi = 100.0F;  // NOLINT(cppcoreguidelines-avoid-magic-numbers)

    context.kp = 0.0F;
    context.ki = kHighKi;

    // Purely lateral: norm=1 (passes validateNorm), produces error[Y]=-1 from identity
    const IMUsample lateral = {
        .accelerometer_g = {1.0F, 0.0F, 0.0F},  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
    };
    iterate_filter(&context, &lateral, kAlignmentCheckSteps);

    // Y-axis integral must be saturated at -kExpectedMaxIntegral after first step
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, -kExpectedMaxIntegral, context.error_integrals[kYaxis]);
    // All integrals must remain within the symmetric clamp range
    for (uint8_t axis = 0U; axis < kNBaxis; axis++) {
        TEST_ASSERT_FLOAT_WITHIN(kExpectedMaxIntegral, 0.0F, context.error_integrals[axis]);
    }
}

/**
 * Test that an out-of-range axis value returns zero from angleAlongAxis.
 *
 * @details
 * This is achieved by passing kNBaxis to angleAlongAxis() on a freshly reset
 * context. kNBaxis is not a valid measurement axis and must never produce an
 * angle estimate. Example: angleAlongAxis(&ctx, kNBaxis) → 0.0F.
 *
 * @internal
 * Exercises the explicit kNBaxis case in the switch statement of
 * angleAlongAxis(), which exists alongside the default case to document the
 * no-op contract for sentinel values. Without this test, removing the case
 * would go unnoticed since default already covers it.
 */
static void test_out_of_range_axis_returns_0(void) {
    // NOLINTNEXTLINE (readability-magic-numbers)
    TEST_ASSERT_EQUAL_FLOAT(0.0F, angleAlongAxis(&context, kNBaxis));
}

/**
 * Test that a NaN or Inf attitude quaternion norm triggers an immediate,
 * unconditional filter reset.
 *
 * @details
 * This is achieved by feeding a gyroscope sample containing NaN, then
 * separately one containing Inf, each with an otherwise-valid gravity
 * accelerometer reading. Under these conditions the corrupted gyro value
 * propagates through the corrected rate of change into every quaternion
 * component, normaliseQuaternion() returns a non-finite norm, and the
 * filter must reset on this single occurrence rather than tolerating it.
 * Example: gyro_x=NaN -> corrected_gyro_radps[X]=NaN -> all four rate-of-
 * change terms NaN -> quaternion NaN -> norm NaN -> reset.
 *
 * @internal
 * Exercises the isnan()/isinf() branch in updateMahonyFilter(), reached via
 * a realistic corrupted-input path rather than direct quaternion
 * manipulation. Replaces the old counter-based test: with no debounce or
 * counter left on this path, a single bad sample must reset immediately.
 */
static void test_bad_quaternion_norm_triggers_reset(void) {
    const IMUsample nan_gyro = {
        .accelerometer_g[kZaxis] = 1.0F,
        .gyroscope_radps[kXaxis] = NAN,
    };
    const IMUsample inf_gyro = {
        .accelerometer_g[kZaxis] = 1.0F,
        .gyroscope_radps[kXaxis] = INFINITY,
    };

    // NaN sub-case
    context.attitude.q1 = 0.1F;  // NOLINT (readability-magic-numbers) non-identity, so reset is distinguishable
    context.dt.last_sampled_tick = 1U;
    bool updated = updateMahonyFilter(&context, &nan_gyro);
    TEST_ASSERT_FALSE_MESSAGE(updated, "Update did not report failure on NaN gyroscope input");
    TEST_ASSERT_TRUE_MESSAGE(isContextReset(&context), "NaN quaternion norm failed to trigger an immediate reset");

    // Inf sub-case, on a freshly re-tilted context
    context.attitude.q1 = 0.1F;  // NOLINT (readability-magic-numbers)
    context.dt.last_sampled_tick++;
    updated = updateMahonyFilter(&context, &inf_gyro);
    TEST_ASSERT_FALSE_MESSAGE(updated, "Update did not report failure on Inf gyroscope input");
    TEST_ASSERT_TRUE_MESSAGE(isContextReset(&context), "Inf quaternion norm failed to trigger an immediate reset");
}

/*********************************************************************************************************************************/
// HELPER FUNCTIONS
/*********************************************************************************************************************************/

/**
 * Feed N identical IMU samples into the filter.
 *
 * @details filter_context->dt.last_sampled_tick is set to the current last_sampled_tick value before
 * each call because updateMahonyFilter() derives elapsed time from
 * filter_context->dt.last_sampled_tick and filter_context->dt.last_valid_tick, not from sample->tick.
 *
 * @param[out] filter_context Mahony filter context to update
 * @param[in] sample Constant IMU sample to feed
 * @param[in] steps Number of update cycles to run.
 */
static void iterate_filter(MahonyContext* filter_context, const IMUsample* sample, uint32_t steps) {
    for (uint32_t i = 0U; i < steps; i++) {
        filter_context->dt.last_sampled_tick = current_tick;
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
 * A reset context has: attitude quaternion = identity [1,0,0,0] and all
 * error integrals = 0. These are exactly the fields touched by
 * resetMahonyFilter().
 *
 * @param filter_context Filter context to inspect
 * @retval 1 Context matches a reset state
 * @retval 0 Context has diverged from a reset state
 */
static bool isContextReset(const MahonyContext* filter_context) {
    const float default_integrals[kNBaxis] = {0.0F, 0.0F, 0.0F};
    const Quaternion unit_quaternion = {.q0 = 1.0F, .q1 = 0.0F, .q2 = 0.0F, .q3 = 0.0F};

    // NOLINTBEGIN (DeprecatedOrUnsafeBufferHandling)
    const uint8_t quat_resetted = (memcmp(&filter_context->attitude, &unit_quaternion, sizeof(Quaternion)) == 0);
    const uint8_t integrals_resetted =
        (memcmp(&filter_context->error_integrals, &default_integrals, (kNBaxis * sizeof(float))) == 0);
    // NOLINTEND

    return (bool)(quat_resetted && integrals_resetted);
}

/**
 * Check if two floats are equal bitwise
 *
 * @param first First float to check
 * @param second Second float to check
 * @return uint8_t 
 */
static bool floats_bit_identical(float first, float second) {
    uint32_t first_bits = 0;
    uint32_t second_bits = 0;

    // NOLINTBEGIN (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
    (void)memcpy(&first_bits, &first, sizeof(first));
    (void)memcpy(&second_bits, &second, sizeof(second));
    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)

    return (first_bits == second_bits);
}
