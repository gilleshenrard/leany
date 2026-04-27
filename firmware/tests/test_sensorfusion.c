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
 * - resetMahonyFilter() does NOT initialise kp, ki, tick_period_seconds,
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


/* ---------------------------------------------------------------------------
 * Test configuration
 * ------------------------------------------------------------------------ */

/** Tolerance for quaternion norm comparisons. */
#define NORM_TOLERANCE (0.005f)

/** Tolerance for angle comparisons in [rad] (~3 degrees). */
#define ANGLE_TOLERANCE_RAD (0.05f)

/** Simulated tick period in [s]: 10ms → 100 Hz update rate. */
#define TICK_PERIOD_S (0.01f)

/**
 * All uint32_t bits set, used as max_tick.
 * getDT() masks with this value, making it a transparent modulo-wrap mask.
 */
#define MAX_TICK_VALUE (UINT32_MAX)

/** Number of steps for the filter to reach a stable attitude from identity. */
#define CONVERGENCE_STEPS (2000U)

/** Steps representing exactly 1 second at 100 Hz. */
#define ONE_SECOND_STEPS (100U)

/**
 * Angular rate producing a 90-degree rotation in exactly 1 second, in [rad/s].
 * With kp=0 (pure integration), roll after ONE_SECOND_STEPS ≈ PI/2.
 */
#define RATE_90DEG_IN_1S ((float)M_PI * 0.5f)

/**
 * Gyro rate for the strong rotation test (~720 deg/s), in [rad/s].
 * Used to verify numerical stability of the quaternion integration step.
 */
#define STRONG_GYRO_RADPS ((float)(4.0 * M_PI))

/**
 * Number of consecutive bad-norm samples to feed in test 4.
 * Must be strictly less than kMaxBadCounts (5) to avoid triggering a
 * filter reset that would zero bad_acceleration_count before we read it.
 */
#define BAD_ACCEL_SAMPLE_COUNT (3U)

/* ---------------------------------------------------------------------------
 * Fixtures
 * ------------------------------------------------------------------------ */

static MahonyContext s_ctx;

/** Monotonically increasing tick counter, incremented in run_filter(). */
static uint32_t s_tick;

/**
 * @brief Initialise the filter context to a clean, known state before each test.
 *
 * resetMahonyFilter() only resets the quaternion, error integrals, and bad
 * counters. All other fields must be set explicitly here.
 */
void setUp(void) {
    memset(&s_ctx, 0, sizeof(s_ctx));
    resetMahonyFilter(&s_ctx);

    s_ctx.kp = kProportionalGain;
    s_ctx.ki = kIntegralGain;
    s_ctx.dt.tick_period_seconds = TICK_PERIOD_S;
    s_ctx.dt.max_tick = MAX_TICK_VALUE;
    s_ctx.dt.current_tick = 0U;
    s_ctx.dt.previous_tick = 0U;
    s_ctx.align_check_enabled = 0U;

    s_tick = 1U;
}

void tearDown(void) {}

/* ---------------------------------------------------------------------------
 * Helper: feed identical IMU samples for N steps
 * ------------------------------------------------------------------------ */

/**
 * @brief Feed N identical IMU samples into the filter.
 *
 * @details ctx.dt.current_tick is set to the current s_tick value before
 * each call because updateMahonyFilter() derives elapsed time from
 * ctx.dt.current_tick and ctx.dt.previous_tick, not from sample->latest_tick.
 *
 * @param ax Accelerometer X in [G].
 * @param ay Accelerometer Y in [G].
 * @param az Accelerometer Z in [G].
 * @param gx Gyroscope X in [rad/s].
 * @param gy Gyroscope Y in [rad/s].
 * @param gz Gyroscope Z in [rad/s].
 * @param steps Number of update cycles to run.
 */
static void run_filter(float ax, float ay, float az, float gx, float gy, float gz, uint32_t steps) {
    IMUsample sample;
    memset(&sample, 0, sizeof(sample));
    sample.accelerometer_g[kXaxis] = ax;
    sample.accelerometer_g[kYaxis] = ay;
    sample.accelerometer_g[kZaxis] = az;
    sample.gyroscope_radps[kXaxis] = gx;
    sample.gyroscope_radps[kYaxis] = gy;
    sample.gyroscope_radps[kZaxis] = gz;

    for (uint32_t i = 0U; i < steps; i++) {
        s_ctx.dt.current_tick = s_tick;
        updateMahonyFilter(&s_ctx, &sample);
        s_tick++;
    }
}

/**
 * @brief Compute the Euclidean norm of the current attitude quaternion.
 *
 * @param ctx Mahony filter context.
 * @return Quaternion norm.
 */
static float quat_norm(const MahonyContext* ctx) {
    const Quaternion* q = &ctx->attitude;
    return sqrtf((q->q0 * q->q0) + (q->q1 * q->q1) + (q->q2 * q->q2) + (q->q3 * q->q3));
}

/* ---------------------------------------------------------------------------
 * Test 1 — Static convergence to identity
 *
 * Input : accel = [0, 0, 1 G] (gravity along −Z body axis), gyro = [0, 0, 0].
 * Expect: roll → 0 rad, pitch → 0 rad, quaternion norm = 1.
 *
 * Rationale: with no angular rate and gravity pointing straight down, the PI
 * controller must drive orientation errors to zero over time.
 * ------------------------------------------------------------------------ */
void test_static_convergence_to_identity(void) {
    run_filter(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, CONVERGENCE_STEPS);

    TEST_ASSERT_FLOAT_WITHIN(ANGLE_TOLERANCE_RAD, 0.0f, angleAlongAxis(&s_ctx, kXaxis));
    TEST_ASSERT_FLOAT_WITHIN(ANGLE_TOLERANCE_RAD, 0.0f, angleAlongAxis(&s_ctx, kYaxis));
    TEST_ASSERT_FLOAT_WITHIN(NORM_TOLERANCE, 1.0f, quat_norm(&s_ctx));
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
 * keeps the norm at 1 with negligible angle error (< ANGLE_TOLERANCE_RAD).
 *
 * Note: using nominal Kp=2.5 here would require knowing the exact convergence
 * behaviour of the PI controller during rotation, which is not analytically
 * tractable for a unit test.
 * ------------------------------------------------------------------------ */
void test_known_roll_rotation_90deg(void) {
    /* Converge with nominal gains so the filter starts from a stable baseline. */
    run_filter(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, CONVERGENCE_STEPS);

    /* Switch to zero correction gains: the filter becomes a pure integrator. */
    s_ctx.kp = 0.0f;
    s_ctx.ki = 0.0f;

    /* Apply roll rate for exactly 1 second. */
    run_filter(0.0f, 0.0f, 1.0f, RATE_90DEG_IN_1S, 0.0f, 0.0f, ONE_SECOND_STEPS);

    TEST_ASSERT_FLOAT_WITHIN(ANGLE_TOLERANCE_RAD, (float)(M_PI * 0.5), angleAlongAxis(&s_ctx, kXaxis));
    TEST_ASSERT_FLOAT_WITHIN(NORM_TOLERANCE, 1.0f, quat_norm(&s_ctx));
}

/* ---------------------------------------------------------------------------
 * Test 3 — Quaternion norm stability under sustained arbitrary input
 *
 * Input : off-axis accel + multi-axis gyro, for 2000 steps.
 * Expect: quaternion norm stays within 1.0 ± NORM_TOLERANCE.
 *
 * Rationale: verifies that numerical drift in the first-order Euler integration
 * is fully corrected by the per-step quaternion normalisation.
 * ------------------------------------------------------------------------ */
void test_quaternion_norm_stays_unity(void) {
    run_filter(0.1f, 0.2f, 0.95f, 0.1f, -0.2f, 0.3f, CONVERGENCE_STEPS);

    TEST_ASSERT_FLOAT_WITHIN(NORM_TOLERANCE, 1.0f, quat_norm(&s_ctx));
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
    run_filter(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, CONVERGENCE_STEPS);

    s_ctx.align_check_enabled = 1U;

    /* Snapshot the quaternion before injecting the misaligned samples. */
    const Quaternion attitude_before = s_ctx.attitude;

    /*
     * Feed 10 samples with a unit-norm but horizontally biased accel.
     * norm([0.6, 0, 0.8]) = sqrt(0.36 + 0 + 0.64) = 1.0 → passes validateNorm.
     * dot([0.6, 0, 0.8], [0, 0, 1]) = 0.8 < 0.9659 → fails alignmentValid.
     */
    run_filter(0.6f, 0.0f, 0.8f, 0.0f, 0.0f, 0.0f, 10U);

    /* The quaternion must be bit-identical (no update happened). */
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q0, s_ctx.attitude.q0);
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q1, s_ctx.attitude.q1);
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q2, s_ctx.attitude.q2);
    TEST_ASSERT_EQUAL_FLOAT(attitude_before.q3, s_ctx.attitude.q3);
    TEST_ASSERT_FLOAT_WITHIN(NORM_TOLERANCE, 1.0f, quat_norm(&s_ctx));
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
    run_filter(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, CONVERGENCE_STEPS);

    const float pitch_before = angleAlongAxis(&s_ctx, kYaxis);

    /* Apply violent pitch rotation for 0.5 s (50 steps at 100 Hz). */
    run_filter(0.0f, 0.0f, 1.0f, 0.0f, STRONG_GYRO_RADPS, 0.0f, 50U);

    const float pitch_after = angleAlongAxis(&s_ctx, kYaxis);

    TEST_ASSERT_FLOAT_WITHIN(NORM_TOLERANCE, 1.0f, quat_norm(&s_ctx));
    /* The pitch must have moved by at least 0.5 rad — the filter is tracking. */
    TEST_ASSERT_GREATER_THAN(0.5f, fabsf(pitch_after - pitch_before));
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