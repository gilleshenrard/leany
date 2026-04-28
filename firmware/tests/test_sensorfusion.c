/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file test_sensorfusion.c
 * @brief Unit tests for the Mahony AHRS filter using simulated IMU inputs.
 *
 * @details
 * All tests run on-host (x86). No HAL or sensor driver dependency is required.
 * Time is simulated by incrementing ctx.dt.current_tick directly before each
 * call to updateMahonyFilter(), since that function does not read
 * sample->latest_tick internally.
 *
 * ## Key implementation constraints respected here
 * - resetMahonyFilter() does NOT initialise kp, ki, kTickPeriod_sececonds,
 *   max_tick, or align_check_enabled. setUp() must set them explicitly.
 * - validateNorm() resets bad_acceleration_count to 0 on the first valid
 *   sample. Tests 4 feeds exactly (kMaxBadCounts - 1) bad samples to avoid
 *   triggering a full filter reset that would zero the counter.
 * - Test 2 uses kp=0, ki=0 to isolate pure first-order gyro integration,
 *   giving a known expected roll value. With nominal Kp=2.5, the accel
 *   correction actively fights the rotation and the expected angle is
 *   not analytically predictable.
 * - Test 4 uses a unit-norm but misaligned accel vector. A 5G lateral shock
 *   would fail validateNorm() before reaching alignmentValid(), making
 *   align_check_enabled irrelevant.
 *
 * @author Gilles Henrard
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "sensorfusion.h"
#include "unity.h"

static const float kPI_F = 3.14159265358979323846F;

enum {
    kConvergenceSteps = 2000U,  ///< Number of steps for the filter to reach a stable attitude from identity
    kStepsIn1second = 100U,     ///< Steps representing exactly 1 second at 100 Hz
};

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} RawValue;

// NOLINTBEGIN (misc-use-internal-linkage)
void test_static_convergence_to_identity(void);
void test_known_roll_rotation_90deg(void);
void test_quaternion_norm_stays_unity(void);
void test_lateral_acceleration_rejected_by_alignment_check(void);
void test_strong_rotation_does_not_diverge(void);
// NOLINTEND

static void run_filter(const RawValue* values, uint32_t steps);
static float quat_norm(const Quaternion* quat);

static const float kNormTolerance = 0.005F;      ///< Tolerance for quaternion norm comparisons
static const float kAngleTolerance_rad = 0.05F;  ///< Tolerance for angle comparisons in [rad] (~3 degrees)
static const float kTickPeriod_sec = 0.01F;      ///< Simulated tick period in [s]: 10ms → 100 Hz update rate

/**
 * Angular rate producing a 90-degree rotation in exactly 1 second, in [rad/s].
 * With kp=0 (pure integration), roll after kStepsIn1second ≈ PI/2.
 */
static const float kRate90DegreesIn1sec = (kPI_F * 0.5F);

/**
 * Gyro rate for the strong rotation test (~720 deg/s), in [rad/s].
 * Used to verify numerical stability of the quaternion integration step.
 */
static const float kStrongGyro_radps = (4.0F * kPI_F);

/**
 * All uint32_t bits set, used as max_tick.
 * getDT() masks with this value, making it a transparent modulo-wrap mask.
 */
#define MAX_TICK_VALUE (UINT32_MAX)

// /**
//  * Number of consecutive bad-norm samples to feed in test 4.
//  * Must be strictly less than kMaxBadCounts (5) to avoid triggering a
//  * filter reset that would zero bad_acceleration_count before we read it.
//  */
// static const uint8_t kBadAccelSampleCount = 3U;

static MahonyContext s_ctx;  ///< Filter context used during tests
static uint32_t s_tick;      ///< Simulated application tick

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Initialise the filter context to a clean, known state before each test.
 *
 * @note
 * resetMahonyFilter() only resets the quaternion, error integrals, and bad
 * counters. All other fields must be set explicitly here.
 */
void setUp(void) {
    memset(&s_ctx, 0, sizeof(s_ctx));
    resetMahonyFilter(&s_ctx);

    s_ctx.kp = kProportionalGain;
    s_ctx.ki = kIntegralGain;
    s_ctx.dt.tick_period_seconds = kTickPeriod_sec;
    s_ctx.dt.max_tick = MAX_TICK_VALUE;
    s_ctx.dt.current_tick = 0U;
    s_ctx.dt.previous_tick = 0U;
    s_ctx.align_check_enabled = 0U;

    s_tick = 1U;
}

/**
 * Free up the resources used during testing
 */
void tearDown(void) {}

/**
 * Test 1 — Static convergence to identity
 *
 * Input : accel = [0, 0, 1 G] (gravity along −Z body axis), gyro = [0, 0, 0].
 * Expect: roll → 0 rad, pitch → 0 rad, quaternion norm = 1.
 *
 * Rationale: with no angular rate and gravity pointing straight down, the PI
 * controller must drive orientation errors to zero over time.
 */
void test_static_convergence_to_identity(void) {
    const RawValue values = {.accel_z = 1.0F};
    run_filter(&values, kConvergenceSteps);

    TEST_ASSERT_FLOAT_WITHIN(kAngleTolerance_rad, 0.0F, angleAlongAxis(&s_ctx, kXaxis));
    TEST_ASSERT_FLOAT_WITHIN(kAngleTolerance_rad, 0.0F, angleAlongAxis(&s_ctx, kYaxis));
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&s_ctx.attitude));
}

/* ---------------------------------------------------------------------------
 * Test 2 — Known roll rotation (pure gyro integration, kp = 0)
 *
 * Input : accel = [0, 0, 1 G], gyro = [π/2, 0, 0] for exactly 1 second.
 * Expect: roll ≈ π/2 rad, quaternion norm = 1.
 *
 * Rationale: with kp = 0 the accel correction term is zeroed, leaving only
 * first-order Euler integration of the gyro. After 100 steps × 0.01 s × π/2
 * rad/s the accumulated angle is π/2. Quaternion normalisation each step
 * keeps the norm at 1 with negligible angle error (< kAngleTolerance_rad).
 *
 * Note: using nominal Kp=2.5 here would require knowing the exact convergence
 * behaviour of the PI controller during rotation, which is not analytically
 * tractable for a unit test.
 * ------------------------------------------------------------------------ */
void test_known_roll_rotation_90deg(void) {
    /* Converge with nominal gains so the filter starts from a stable baseline. */
    const RawValue values = {.accel_z = 1.0F};
    run_filter(&values, kConvergenceSteps);

    /* Switch to zero correction gains: the filter becomes a pure integrator. */
    s_ctx.kp = 0.0F;
    s_ctx.ki = 0.0F;

    /* Apply roll rate for exactly 1 second. */
    const RawValue roll_values = {
        .accel_z = 1.0F,
        .gyro_x = kRate90DegreesIn1sec,
    };
    run_filter(&roll_values, kStepsIn1second);

    TEST_ASSERT_FLOAT_WITHIN(kAngleTolerance_rad, kRate90DegreesIn1sec, angleAlongAxis(&s_ctx, kXaxis));
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&s_ctx.attitude));
}

/* ---------------------------------------------------------------------------
 * Test 3 — Quaternion norm stability under sustained arbitrary input
 *
 * Input : off-axis accel + multi-axis gyro, for 2000 steps.
 * Expect: quaternion norm stays within 1.0 ± kNormTolerance.
 *
 * Rationale: verifies that numerical drift in the first-order Euler integration
 * is fully corrected by the per-step quaternion normalisation.
 * ------------------------------------------------------------------------ */
void test_quaternion_norm_stays_unity(void) {
    const RawValue skewed = {
        .accel_x = 0.1F,
        .accel_y = 0.2F,
        .accel_z = 0.95F,
        .gyro_x = 0.1F,
        .gyro_y = -0.2F,
        .gyro_z = 0.3F,
    };
    run_filter(&skewed, kConvergenceSteps);

    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&s_ctx.attitude));
}

/* ---------------------------------------------------------------------------
 * Test 4 — Strong lateral acceleration rejected by alignment check
 *
 * Input : accel = [0.6, 0, 0.8 G] (norm ≈ 1.0, but 37° off the Z axis),
 *         gyro = [0, 0, 0], align_check_enabled = 1.
 * Expect: attitude is frozen (all updates skipped), norm = 1.
 *
 * Rationale: after convergence the estimated gravity is [0, 0, 1]. The dot
 * product with normalised [0.6, 0, 0.8] is 0.8, which is below
 * kMinAlignmentCosine (cos 15° ≈ 0.9659). alignmentValid() returns 0 and
 * updateMahonyFilter() returns early without touching the quaternion.
 *
 * Note: a 5 G lateral shock would fail validateNorm() before ever reaching
 * alignmentValid(), so align_check_enabled would play no role in that case.
 * ------------------------------------------------------------------------ */
void test_lateral_acceleration_rejected_by_alignment_check(void) {
    /* Converge to a stable identity orientation. */
    const RawValue values = {.accel_z = 1.0F};
    run_filter(&values, kConvergenceSteps);

    s_ctx.align_check_enabled = 1U;

    /* Snapshot the quaternion before injecting the misaligned samples. */
    const Quaternion attitude_before = s_ctx.attitude;

    /*
     * Feed 10 samples with a unit-norm but horizontally biased accel.
     * norm([0.6, 0, 0.8]) = sqrt(0.36 + 0 + 0.64) = 1.0 → passes validateNorm.
     * dot([0.6, 0, 0.8], [0, 0, 1]) = 0.8 < 0.9659 → fails alignmentValid.
     */
    const RawValue unit_norm_values = {
        .accel_x = 0.6F,
        .accel_z = 0.8F,
    };
    run_filter(&unit_norm_values, 10U);  // NOLINT (cppcoreguidelines-avoid-magic-numbers)

    /* The quaternion must be bit-identical (no update happened). */
    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q0, s_ctx.attitude.q0);
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q1, s_ctx.attitude.q1);
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q2, s_ctx.attitude.q2);
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q3, s_ctx.attitude.q3);
    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&s_ctx.attitude));
    // NOLINTEND
}

/* ---------------------------------------------------------------------------
 * Test 5 — High angular rate does not blow up the quaternion
 *
 * Input : accel = [0, 0, 1 G], gyro = [0, 4π, 0] (~720 deg/s pitch), 50 steps.
 * Expect: quaternion norm stays at 1.0, pitch angle changes by > 0.5 rad.
 *
 * Rationale: at 4π rad/s with dt = 0.01 s, each integration step adds
 * |dq|·dt ≈ 0.063 to each quaternion component before normalisation. This
 * verifies that normaliseQuaternion() successfully prevents norm blow-up
 * and that the filter tracks (not freezes) the motion.
 * ------------------------------------------------------------------------ */
void test_strong_rotation_does_not_diverge(void) {
    /* Converge to a stable reference attitude. */
    const RawValue values = {.accel_z = 1.0F};
    run_filter(&values, kConvergenceSteps);

    const float pitch_before = angleAlongAxis(&s_ctx, kYaxis);

    /* Apply violent pitch rotation for 0.5 s (50 steps at 100 Hz). */
    const RawValue violent_pitch = {
        .accel_z = 1.0F,
        .gyro_y = kStrongGyro_radps,
    };
    run_filter(&violent_pitch, 50U);  // NOLINT (cppcoreguidelines-avoid-magic-numbers)

    const float pitch_after = angleAlongAxis(&s_ctx, kYaxis);

    TEST_ASSERT_FLOAT_WITHIN(kNormTolerance, 1.0F, quat_norm(&s_ctx.attitude));
    /* The pitch must have moved by at least 0.5 rad — the filter is tracking. */
    TEST_ASSERT_GREATER_THAN_FLOAT(0.5F,  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
                                   fabsf(pitch_after - pitch_before));
}

/* ---------------------------------------------------------------------------
 * Runner
 * ------------------------------------------------------------------------ */
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_static_convergence_to_identity);
    RUN_TEST(test_known_roll_rotation_90deg);
    RUN_TEST(test_quaternion_norm_stays_unity);
    RUN_TEST(test_lateral_acceleration_rejected_by_alignment_check);
    RUN_TEST(test_strong_rotation_does_not_diverge);
    return UNITY_END();
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * @brief Feed N identical IMU samples into the filter.
 *
 * @details ctx.dt.current_tick is set to the current s_tick value before
 * each call because updateMahonyFilter() derives elapsed time from
 * ctx.dt.current_tick and ctx.dt.previous_tick, not from sample->latest_tick.
 *
 * @param values Raw values to use in the test
 * @param steps Number of update cycles to run.
 */
static void run_filter(const RawValue* values, uint32_t steps) {
    IMUsample sample;
    memset(&sample, 0, sizeof(sample));
    sample.accelerometer_g[kXaxis] = values->accel_x;
    sample.accelerometer_g[kYaxis] = values->accel_y;
    sample.accelerometer_g[kZaxis] = values->accel_z;
    sample.gyroscope_radps[kXaxis] = values->gyro_x;
    sample.gyroscope_radps[kYaxis] = values->gyro_y;
    sample.gyroscope_radps[kZaxis] = values->gyro_z;

    for (uint32_t i = 0U; i < steps; i++) {
        s_ctx.dt.current_tick = s_tick;
        updateMahonyFilter(&s_ctx, &sample);
        s_tick++;
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