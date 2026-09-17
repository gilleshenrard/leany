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
    kSustainedSteps = 50U,       ///< Number of steps used to exercise sustained low-trust conditions
};

//private functions
static void iterate_filter(MahonyContext* filter_context, const IMUsample* sample, uint32_t steps);
static float quat_norm(const Quaternion* quat);
static bool isContextReset(const MahonyContext* filter_context);
static bool floats_bit_identical(float first, float second);
static void test_null_pointer_guards(void);
static void test_tick_handles_overflow(void);
static void test_bad_accel_samples_skipped_without_reset(void);
static void test_misaligned_accel_reduces_trust_but_keeps_updating(void);
static void test_manual_pure_gyro_zeroes_correction(void);
static void test_sustained_misalignment_never_freezes_updates(void);
static void test_trust_weight_full_at_perfect_conditions(void);
static void test_trust_weight_floored_beyond_alignment_limit(void);
static void test_trust_weight_scales_with_norm_deviation(void);
static void test_reset_clears_derived_trust_and_cause_fields(void);
static void test_yaw_angle_returns_0(void);
static void test_correct_attitude_angle_calculation(void);
static void test_controller_no_shift_at_rest(void);
static void test_gyro_integration_accumulates_correctly(void);
static void test_normalisation_prevents_drift_under_sustained_input(void);
static void test_integration_stable_at_high_angular_rate(void);
static void test_integral_clamped_on_windup(void);
static void test_out_of_range_axis_returns_0(void);
static void test_bad_quaternion_norm_triggers_reset(void);

//constants
static constexpr float kNormTolerance = 0.005F;          ///< Tolerance for quaternion norm comparisons
static constexpr float kTrustTolerance = 0.01F;          ///< Tolerance for trust weight / gain comparisons
static constexpr float kAngleTolerance_rad = 0.05F;      ///< Tolerance for angle comparisons in [rad] (~3 degrees)
static constexpr float kTickPeriod_sec = 0.01F;          ///< Simulated tick period in [s]: 10ms -> 100 Hz update rate
static constexpr float kPI_F = 3.14159265358979323846F;  ///< Pi, as a float value
static constexpr uint32_t kMaxTick = UINT32_MAX;         ///< Maximum value a system tick can take
static constexpr float kStrongGyro_radps = (4.0F * kPI_F);  ///< Strong rotation speed

// mirrors of mahony.c's private constants: any change there must be reflected here
static constexpr float kExpectedMaxIntegral = 0.3F;     ///< mirrors kMaxIntegralError in mahony.c
static constexpr float kExpectedKpTrustFloor = 0.2F;    ///< mirrors kMinKpTrustFraction in mahony.c
static constexpr float kExpectedMaxValidDT_sec = 4.0F;  ///< mirrors kMaxValidDTseconds in mahony.c

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
    RUN_TEST(test_misaligned_accel_reduces_trust_but_keeps_updating);
    RUN_TEST(test_manual_pure_gyro_zeroes_correction);
    RUN_TEST(test_sustained_misalignment_never_freezes_updates);
    RUN_TEST(test_trust_weight_full_at_perfect_conditions);
    RUN_TEST(test_trust_weight_floored_beyond_alignment_limit);
    RUN_TEST(test_trust_weight_scales_with_norm_deviation);
    RUN_TEST(test_reset_clears_derived_trust_and_cause_fields);
    RUN_TEST(test_yaw_angle_returns_0);
    RUN_TEST(test_correct_attitude_angle_calculation);
    RUN_TEST(test_controller_no_shift_at_rest);
    RUN_TEST(test_gyro_integration_accumulates_correctly);
    RUN_TEST(test_normalisation_prevents_drift_under_sustained_input);
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
 * resetMahonyFilter() only resets the quaternion, integrals, and derived
 * trust fields (weighed_kp, weighed_ki, trust_weight, last_reset_cause).
 * Configuration fields (gains, tick timebase, manual_pure_gyro) must be
 * set explicitly here.
 */
void setUp(void) {
    (void)memset(&context, 0, sizeof(context));  // NOLINT (DeprecatedOrUnsafeBufferHandling)
    resetMahonyFilter(&context);

    context.base_kp = kProportionalGain;
    context.base_ki = kIntegralGain;
    context.dt.tick_period_seconds = kTickPeriod_sec;
    context.dt.max_tick = kMaxTick;
    context.dt.last_sampled_tick = 0U;
    context.dt.last_valid_tick = 0U;
    context.manual_pure_gyro = false;

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
 * Test that tick wraparound does not trigger a spurious filter reset, and that
 * genuinely invalid dT values reset the filter with the correct recorded cause.
 *
 * @details
 * This is achieved by setting last_valid_tick near UINT32_MAX and last_sampled_tick
 * near zero, then running one update. Under these conditions computeDTseconds() computes
 * the correct elapsed time via bitmask subtraction rather than overflowing.
 * Example: last_valid_tick = UINT32_MAX - 5, last_sampled_tick = 3 →
 * delta = (3 - (UINT32_MAX - 5)) & UINT32_MAX = 9 ticks = 0.09s (valid).
 * Also verifies that dT=0 and dT beyond kMaxValidDTseconds each trigger a reset
 * with last_reset_cause == kDTinvalid.
 *
 * @internal
 * Exercises the bitmask subtraction in computeDTseconds() and the bounds check
 * in isDTvalid(). kExpectedMaxValidDT_sec mirrors mahony.c's private
 * kMaxValidDTseconds — any change there must be reflected here. Since a real
 * reset overwrites last_reset_cause to the actual cause (not kNone),
 * isContextReset() intentionally does not check that field — it is asserted
 * directly here instead.
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

    //test dT = 0 ticks -> filter reset, cause recorded as kDTinvalid
    context.dt.last_valid_tick = context.dt.last_sampled_tick;
    updateMahonyFilter(&context, &violent_pitch);
    TEST_ASSERT_TRUE_MESSAGE(isContextReset(&context), "dT 0s context failed");
    TEST_ASSERT_EQUAL_INT(kDTinvalid, context.last_reset_cause);

    //test dT beyond kMaxValidDTseconds -> filter reset, cause recorded as kDTinvalid
    const float excessive_dt_sec = (kExpectedMaxValidDT_sec + 0.5F);  // NOLINT(*-magic-numbers)
    const uint32_t tick_dt_excessive = (uint32_t)(excessive_dt_sec / kTickPeriod_sec);
    context.dt.last_valid_tick = 0;
    context.dt.last_sampled_tick = tick_dt_excessive;
    updateMahonyFilter(&context, &violent_pitch);
    TEST_ASSERT_TRUE_MESSAGE(isContextReset(&context), "dT beyond ceiling context failed");
    TEST_ASSERT_EQUAL_INT(kDTinvalid, context.last_reset_cause);
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
 * normValid(). This is the one deliberately-kept "hard skip, no gyro
 * integration" path in the current design — reserved for catastrophic norms,
 * as opposed to misalignment, which is handled by continuous trust-weighting
 * (see test_misaligned_accel_reduces_trust_but_keeps_updating).
 */
static void test_bad_accel_samples_skipped_without_reset(void) {
    const IMUsample bad_sample = {.accelerometer_g[kXaxis] = 5.0F};  // NOLINT(readability-magic-numbers)
    const uint8_t bad_streak_length = 10U;                           // NOLINT(readability-magic-numbers)

    // tilt the attitude so "untouched" is distinguishable from "coincidentally identity"
    context.attitude.q1 = 0.1F;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
    const Quaternion attitude_before = context.attitude;
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
 * Test that a norm-valid but misaligned accelerometer sample reduces the
 * trust-weighted gains, but never freezes the attitude update.
 *
 * @details
 * This is achieved by converging the filter, then feeding a unit-norm but
 * horizontally biased acceleration vector. Under these conditions the
 * alignment cosine falls below kMinAlignmentCosine, trust_weight is driven
 * toward its floor, and weighed_kp/weighed_ki shrink accordingly — but the
 * quaternion must still change every cycle, since gyro integration is
 * unconditional.
 * Example: accel=[0.6, 0, 0.8G] → norm=1.0 (passes normValid), cosine=0.8 
 * 0.9659 → trust_weight clamped low, but attitude still updates.
 *
 * @internal
 * Replaces the old hard-freeze test (test_alignment_check_freezes_update_on_
 * lateral_accel / test_alignment_check_disabled_allows_update), which
 * asserted a bit-identical, frozen quaternion — a behaviour that no longer
 * exists now that alignmentValid() has been removed and gyro integration
 * runs unconditionally through applyTrustToCoefficients()'s continuous
 * weighting instead.
 */
static void test_misaligned_accel_reduces_trust_but_keeps_updating(void) {
    iterate_filter(&context, &kPureGravity, kConvergenceSteps);

    const Quaternion attitude_before = context.attitude;

    /*
     * norm([0.6, 0, 0.8]) = sqrt(0.36 + 0 + 0.64) = 1.0 -> passes normValid.
     * dot([0.6, 0, 0.8], [0, 0, 1]) = 0.8 < 0.9659 -> below the alignment limit.
     */
    const IMUsample unit_norm_misaligned = {
        .accelerometer_g = {0.6F, 0.0F, 0.8F},  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
    };
    iterate_filter(&context, &unit_norm_misaligned, kAlignmentCheckSteps);

    // trust must have dropped well below full trust
    TEST_ASSERT_LESS_THAN_FLOAT(1.0F, context.trust_weight);
    // but weighed_kp must never drop below its floor
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT((context.base_kp * kExpectedKpTrustFloor), context.weighed_kp);

    // the quaternion must have moved — no freeze, unlike the old hard-gate behaviour
    const bool unchanged = (bool)(floats_bit_identical(attitude_before.q0, context.attitude.q0) &&
                                  floats_bit_identical(attitude_before.q1, context.attitude.q1) &&
                                  floats_bit_identical(attitude_before.q2, context.attitude.q2) &&
                                  floats_bit_identical(attitude_before.q3, context.attitude.q3));
    TEST_ASSERT_FALSE_MESSAGE(unchanged, "Misaligned-but-valid accel incorrectly froze the attitude");
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&context.attitude));
}

/**
 * Test that manual_pure_gyro forces the correction to zero, regardless of
 * how wrong the accelerometer reading is, while still integrating gyro data.
 *
 * @details
 * This is achieved by enabling manual_pure_gyro, then feeding a deliberately
 * wrong accelerometer reading (perpendicular to the true "up") alongside a
 * known constant gyro rate for exactly 1 second. Under these conditions the
 * resulting angle must match pure gyro integration exactly, proving the
 * garbage accelerometer reading had zero influence, and the telemetry fields
 * must reflect the forced-floor state.
 * Example: accel=[1,0,0] (wrong), gyro_x=Pi/2 rad/s, dt=0.01s, 100 steps ->
 * roll = Pi/2 rad/s × 1s = Pi/2 rad, identical to test_gyro_integration_
 * accumulates_correctly despite the garbage accelerometer input.
 *
 * @internal
 * Exercises the manual_pure_gyro branch in updateMahonyFilter(), which forces
 * errors[] to {0,0,0} and sets trust_weight/weighed_ki to 0 and weighed_kp to
 * base_kp * kMinKpTrustFraction, regardless of what applyTrustToCoefficients()
 * computed from the (here, deliberately bad) sensor data.
 */
static void test_manual_pure_gyro_zeroes_correction(void) {
    const float rate_90degrees_in_1sec = (kPI_F * 0.5F);

    context.manual_pure_gyro = true;

    // deliberately wrong accelerometer reading: perpendicular to true "up" at identity
    const IMUsample garbage_accel_with_gyro = {
        .accelerometer_g = {1.0F, 0.0F, 0.0F},  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
        .gyroscope_radps[kXaxis] = rate_90degrees_in_1sec,
    };
    iterate_filter(&context, &garbage_accel_with_gyro, kStepsIn1second);

    // telemetry must reflect the forced override, not the (bad) computed trust
    TEST_ASSERT_EQUAL_FLOAT(0.0F, context.trust_weight);  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
    TEST_ASSERT_EQUAL_FLOAT(0.0F, context.weighed_ki);    // NOLINT (cppcoreguidelines-avoid-magic-numbers)
    TEST_ASSERT_FLOAT_WITHIN(kTrustTolerance, (context.base_kp * kExpectedKpTrustFloor), context.weighed_kp);

    // the resulting angle must match pure gyro integration — the garbage accel had zero effect
    TEST_ASSERT_FLOAT_WITHIN(kAngleTolerance_rad, rate_90degrees_in_1sec, angleAlongAxis(&context, kXaxis));
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&context.attitude));
}

/**
 * Test that sustained, norm-valid misalignment never stalls the filter.
 *
 * @details
 * This is achieved by feeding a fixed, deliberately misaligned but unit-norm
 * accelerometer reading for many consecutive updates, with manual_pure_gyro
 * disabled. Under these conditions every single update must complete
 * successfully — last_valid_tick must track last_sampled_tick exactly, with
 * no accumulated backlog — and the attitude must have visibly moved under
 * the (reduced but nonzero) weighted correction.
 *
 * @internal
 * Regression guard for the live-lock found during bench testing: with the
 * old alignmentValid() hard gate, a persistently misaligned-but-valid sample
 * caused updateMahonyFilter() to return before ever calling
 * integrateGyroQuaternion(), so dt accumulated indefinitely until the
 * kMaxValidDTseconds timeout forced a reset. That early-return path no
 * longer exists for alignment; only normValid() retains it, deliberately,
 * for catastrophic norms (see test_bad_accel_samples_skipped_without_reset).
 */
static void test_sustained_misalignment_never_freezes_updates(void) {
    const IMUsample persistently_misaligned = {.accelerometer_g = {1.0F, 0.0F, 0.0F}};  // NOLINT(*-magic-numbers)

    iterate_filter(&context, &persistently_misaligned, kSustainedSteps);

    // every cycle must have completed: no backlog accumulated in dt
    TEST_ASSERT_EQUAL_UINT32(context.dt.last_sampled_tick, context.dt.last_valid_tick);

    // the attitude must have actually moved under the weighted correction, not frozen at identity
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0F, fabsf(angleAlongAxis(&context, kYaxis)));
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&context.attitude));
}

/**
 * Test that trust reaches (approximately) full strength under perfect
 * conditions: unit-norm acceleration, perfectly aligned with the estimate.
 *
 * @details
 * This is achieved by feeding kPureGravity once from a freshly reset (identity)
 * context. Under these conditions norm_absolute_deviation=0 and
 * alignment_cosine=1.0 exactly, so both interpolated trust factors saturate
 * at their maximum.
 * Example: accel=[0,0,1], estimate=[0,0,1] → cosine=1.0 → trust_weight≈1.0 →
 * weighed_kp≈base_kp, weighed_ki≈base_ki.
 *
 * @internal
 * Exercises the "ceiling" end of applyTrustToCoefficients()'s two
 * interpolation curves together, indirectly (the function is private).
 */
static void test_trust_weight_full_at_perfect_conditions(void) {
    context.dt.last_sampled_tick = 1U;
    updateMahonyFilter(&context, &kPureGravity);

    TEST_ASSERT_FLOAT_WITHIN(kTrustTolerance, 1.0F, context.trust_weight);
    TEST_ASSERT_FLOAT_WITHIN(kTrustTolerance, context.base_kp, context.weighed_kp);
    TEST_ASSERT_FLOAT_WITHIN(kTrustTolerance, context.base_ki, context.weighed_ki);
}

/**
 * Test that trust is floored to zero once alignment exceeds kMinAlignmentCosine,
 * without ever going negative.
 *
 * @details
 * This is achieved by feeding an accelerometer reading exactly perpendicular
 * to the current (identity) estimate. Under these conditions the alignment
 * cosine is 0, far beyond the 15° limit, so the interpolation clamps to its
 * floor rather than extrapolating past it.
 * Example: accel=[1,0,0], estimate=[0,0,1] → cosine=0 → trusted_alignment
 * clamps to 0 → trust_weight=0 → weighed_kp = base_kp * kMinKpTrustFraction,
 * weighed_ki = 0.
 *
 * @internal
 * Exercises the clamp_min_max() call inside linearInterpolation(), guarding
 * against the interpolation extrapolating to a negative trust value for
 * inputs beyond the configured range.
 */
static void test_trust_weight_floored_beyond_alignment_limit(void) {
    const IMUsample perpendicular_accel = {.accelerometer_g = {1.0F, 0.0F, 0.0F}};  // NOLINT(*-magic-numbers)

    context.dt.last_sampled_tick = 1U;
    updateMahonyFilter(&context, &perpendicular_accel);

    TEST_ASSERT_FLOAT_WITHIN(kTrustTolerance, 0.0F, context.trust_weight);
    TEST_ASSERT_FLOAT_WITHIN(kTrustTolerance, (context.base_kp * kExpectedKpTrustFloor), context.weighed_kp);
    TEST_ASSERT_FLOAT_WITHIN(kTrustTolerance, 0.0F, context.weighed_ki);
}

/**
 * Test that trust scales down proportionally to accelerometer norm deviation,
 * independent of alignment.
 *
 * @details
 * This is achieved by feeding an accelerometer reading that stays perfectly
 * aligned with the estimate's direction (so alignment trust stays at its
 * ceiling) but whose magnitude deviates from 1G by a known amount within
 * kMaxNormEpsilon. Under these conditions only the norm-based trust factor
 * should account for the resulting reduction.
 * Example: accel=[0,0,1.1] → norm=1.1, deviation=0.1, direction unchanged →
 * cosine=1.0 (full alignment trust) → trusted_norm = 1 - (0.1/0.15) ≈ 0.333
 * → trust_weight ≈ 0.333.
 *
 * @internal
 * Exercises the norm-based linearInterpolation() call in
 * applyTrustToCoefficients() in isolation from the alignment-based one, by
 * keeping direction constant and varying magnitude only. Uses the now-public
 * kMaxNormEpsilon directly rather than a mirrored copy.
 */
static void test_trust_weight_scales_with_norm_deviation(void) {
    const float norm_deviation = 0.1F;  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
    const float expected_trust = (1.0F - (norm_deviation / kMaxNormEpsilon));
    const IMUsample deviated_norm = {.accelerometer_g[kZaxis] = (1.0F + norm_deviation)};

    context.dt.last_sampled_tick = 1U;
    updateMahonyFilter(&context, &deviated_norm);

    TEST_ASSERT_FLOAT_WITHIN(kTrustTolerance, expected_trust, context.trust_weight);
}

/**
 * Test that resetMahonyFilter() clears every field it is responsible for,
 * including the derived trust fields and the reset cause.
 *
 * @details
 * This is achieved by dirtying every field resetMahonyFilter() is documented
 * to touch, then calling it directly and checking the resulting state via
 * isContextReset(), plus a direct check on last_reset_cause.
 *
 * @internal
 * isContextReset() deliberately does not check last_reset_cause (see its own
 * doc comment) since a reset triggered through updateMahonyFilter() legitimately
 * overwrites it to the actual cause. This test calls resetMahonyFilter()
 * directly, the one path where kNone is the correct expected value, and
 * checks it explicitly.
 */
static void test_reset_clears_derived_trust_and_cause_fields(void) {
    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    context.attitude.q1 = 0.3F;
    context.error_integrals[kXaxis] = 0.1F;
    context.weighed_kp = 99.0F;
    context.weighed_ki = 99.0F;
    context.trust_weight = 0.5F;
    context.last_reset_cause = kQuaternionNanInf;
    // NOLINTEND

    resetMahonyFilter(&context);

    TEST_ASSERT_TRUE_MESSAGE(isContextReset(&context), "resetMahonyFilter() left the context in a non-reset state");
    TEST_ASSERT_EQUAL_INT(kNone, context.last_reset_cause);
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
 * Exercises integrateGyroQuaternion() in isolation. With kp and ki zeroed,
 * applyProportionateErrors() adds zero correction and the integral branch is
 * skipped entirely, so the only active path is the quaternion derivative
 * and the Euler integration step.
 */
static void test_gyro_integration_accumulates_correctly(void) {
    const float rate_90degrees_in_1sec = (kPI_F * 0.5F);

    // Switch to zero correction gains: the filter becomes a pure integrator
    context.base_kp = 0.0F;
    context.base_ki = 0.0F;

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
 * verifies that the PI correction path in applyProportionateErrors() does not freeze the
 * update under high rates — the pitch must change by at least 0.1 rad over 0.5s.
 * This threshold was derived empirically with base_kp=25, base_ki=5: the accel
 * correction actively counters the rotation, but does not fully cancel it.
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
 * This is achieved by zeroing kp and setting a high ki, then feeding a small
 * (10°) tilt for 10 steps — small enough to stay within kMinAlignmentCosine's
 * 15° trust cone (so weighed_ki is scaled down, not floored to zero), but
 * large enough to produce a real, sustained cross-product error. Under these
 * conditions the integral saturates well within the iteration budget.
 * Example: accel=[sin10°,0,cos10°], body_estimates=[0,0,1] → error[Y]≈-0.174 →
 * cosine≈0.985 → trust_weight≈0.55 → weighed_ki≈55 → integral saturates by
 * step 4, clamped to -0.3 for the remainder.
 *
 * @internal
 * Exercises the clamp_absolute() call applied to each error_integrals[axis]
 * inside accumulateIntegralErrors(). A full 90° lateral sample (cosine=0)
 * would instead floor trust_weight to 0 via applyTrustToCoefficients(),
 * zeroing weighed_ki regardless of base_ki — this test must stay inside
 * the alignment cone to exercise the clamp at all. kExpectedMaxIntegral
 * mirrors mahony.c's private kMaxIntegralError, and kExpectedAlignmentLimit
 * documents why 10° was chosen over the old 90° sample.
 */
static void test_integral_clamped_on_windup(void) {
    // ki high enough that even scaled-down trust (~0.55 at 10°) still drives
    // rapid saturation well within kAlignmentCheckSteps
    static constexpr float kHighKi = 100.0F;       // NOLINT(cppcoreguidelines-avoid-magic-numbers)
    static constexpr float kSmallTiltDeg = 10.0F;  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
    static constexpr float kSmallTilt_rad = (kSmallTiltDeg * kPI_F / 180.0F);

    context.base_kp = 0.0F;
    context.base_ki = kHighKi;

    // Small tilt: within the alignment cone (cosine ≈ 0.985 > kExpectedAlignmentLimit),
    // still produces a real, nonzero error[Y]
    const IMUsample small_tilt = {
        .accelerometer_g = {sinf(kSmallTilt_rad), 0.0F, cosf(kSmallTilt_rad)},
    };
    iterate_filter(&context, &small_tilt, kAlignmentCheckSteps);

    // Y-axis integral must have saturated at -kExpectedMaxIntegral
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, -kExpectedMaxIntegral, context.error_integrals[kYaxis]);
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
 * unconditional filter reset, with the correct cause recorded.
 *
 * @details
 * This is achieved by feeding a gyroscope sample containing NaN, then
 * separately one containing Inf, each with an otherwise-valid gravity
 * accelerometer reading. Under these conditions the corrupted gyro value
 * propagates through the corrected rate of change into every quaternion
 * component, normaliseQuaternion() returns a non-finite norm, and the
 * filter must reset on this single occurrence rather than tolerating it.
 * Example: gyro_x=NaN -> corrected_gyro_radps[X]=NaN -> all four rate-of-
 * change terms NaN -> quaternion NaN -> norm NaN -> reset, cause=kQuaternionNanInf.
 *
 * @internal
 * Exercises the isnan()/isinf() branch in updateMahonyFilter(), reached via
 * a realistic corrupted-input path rather than direct quaternion
 * manipulation. With no debounce or counter left on this path, a single
 * bad sample must reset immediately. isContextReset() does not check
 * last_reset_cause (see its doc comment) since the real cause here is
 * kQuaternionNanInf, not kNone — asserted directly.
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
    TEST_ASSERT_EQUAL_INT(kQuaternionNanInf, context.last_reset_cause);

    // Inf sub-case, on a freshly re-tilted context
    context.attitude.q1 = 0.1F;  // NOLINT (readability-magic-numbers)
    context.dt.last_sampled_tick++;
    updated = updateMahonyFilter(&context, &inf_gyro);
    TEST_ASSERT_FALSE_MESSAGE(updated, "Update did not report failure on Inf gyroscope input");
    TEST_ASSERT_TRUE_MESSAGE(isContextReset(&context), "Inf quaternion norm failed to trigger an immediate reset");
    TEST_ASSERT_EQUAL_INT(kQuaternionNanInf, context.last_reset_cause);
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
 * A reset context has: attitude quaternion = identity [1,0,0,0], all error
 * integrals = 0, weighed_kp = 0, weighed_ki = 0, and trust_weight = 0.
 * last_reset_cause is deliberately NOT checked here: a reset triggered
 * through updateMahonyFilter() correctly overwrites it to the actual cause
 * (kDTinvalid / kQuaternionNanInf) immediately after calling
 * resetMahonyFilter(), so kNone would be the wrong expectation in that case.
 * Callers that need to verify last_reset_cause check it directly.
 *
 * @param filter_context Filter context to inspect
 * @retval true Context matches a reset state
 * @retval false Context has diverged from a reset state
 */
static bool isContextReset(const MahonyContext* filter_context) {
    const float default_integrals[kNBaxis] = {0.0F, 0.0F, 0.0F};
    const Quaternion unit_quaternion = {.q0 = 1.0F, .q1 = 0.0F, .q2 = 0.0F, .q3 = 0.0F};

    // NOLINTBEGIN (DeprecatedOrUnsafeBufferHandling)
    const bool quat_resetted = (memcmp(&filter_context->attitude, &unit_quaternion, sizeof(Quaternion)) == 0);
    const bool integrals_resetted =
        (memcmp(&filter_context->error_integrals, &default_integrals, (kNBaxis * sizeof(float))) == 0);
    // NOLINTEND

    const bool trust_resetted = (bool)(floats_bit_identical(filter_context->weighed_kp, 0.0F) &&
                                       floats_bit_identical(filter_context->weighed_ki, 0.0F) &&
                                       floats_bit_identical(filter_context->trust_weight, 0.0F));

    return (bool)(quat_resetted && integrals_resetted && trust_resetted);
}

/**
 * Check if two floats are equal bitwise
 *
 * @param first First float to check
 * @param second Second float to check
 * @return Whether both floats share the exact same bit pattern
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
