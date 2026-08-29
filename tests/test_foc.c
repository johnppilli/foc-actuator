#include "foc.h"
#include "minitest.h"
#include <stdlib.h>

static float frand(float lo, float hi)
{
    return lo + (hi - lo) * (float)rand() / (float)RAND_MAX;
}

static void test_wrap(void)
{
    ASSERT_NEAR(foc_wrap_2pi(0.0f), 0.0f, 1e-6f);
    ASSERT_NEAR(foc_wrap_2pi(-0.1f), FOC_TWO_PI - 0.1f, 1e-5f);
    ASSERT_NEAR(foc_wrap_2pi(7.0f), 7.0f - FOC_TWO_PI, 1e-5f);
    ASSERT_NEAR(foc_wrap_2pi(-13.0f), -13.0f + 3.0f * FOC_TWO_PI, 1e-4f);
    ASSERT_TRUE(foc_wrap_2pi(FOC_TWO_PI) < 1e-5f);
    ASSERT_NEAR(foc_wrap_pi(3.5f), 3.5f - FOC_TWO_PI, 1e-5f);
    ASSERT_NEAR(foc_wrap_pi(-3.5f), FOC_TWO_PI - 3.5f, 1e-5f);
    ASSERT_NEAR(foc_wrap_pi(1.0f), 1.0f, 1e-6f);
    ASSERT_TRUE(foc_wrap_pi(FOC_PI) < 0.0f && foc_wrap_pi(FOC_PI) > -FOC_PI - 1e-5f);
    ASSERT_NEAR(foc_clampf(5.0f, -1.0f, 1.0f), 1.0f, 0.0f);
    ASSERT_NEAR(foc_clampf(-5.0f, -1.0f, 1.0f), -1.0f, 0.0f);
    ASSERT_NEAR(foc_clampf(0.3f, -1.0f, 1.0f), 0.3f, 0.0f);
}

static void test_clarke_roundtrip(void)
{
    for (int i = 0; i < 200; i++) {
        float ia = frand(-5.0f, 5.0f), ib = frand(-5.0f, 5.0f);
        float ic = -ia - ib;
        foc_ab_t ab = foc_clarke(ia, ib);
        foc_abc_t abc = foc_inv_clarke(ab);
        ASSERT_NEAR(abc.a, ia, 1e-4f);
        ASSERT_NEAR(abc.b, ib, 1e-4f);
        ASSERT_NEAR(abc.c, ic, 1e-4f);
        ASSERT_NEAR(abc.a + abc.b + abc.c, 0.0f, 1e-4f);
        foc_ab_t ab3 = foc_clarke3(ia, ib, ic);
        ASSERT_NEAR(ab3.alpha, ab.alpha, 1e-4f);
        ASSERT_NEAR(ab3.beta, ab.beta, 1e-4f);
    }
}

static void test_park_roundtrip(void)
{
    for (int i = 0; i < 200; i++) {
        foc_ab_t ab = { frand(-5.0f, 5.0f), frand(-5.0f, 5.0f) };
        float th = frand(-10.0f, 10.0f), s, c;
        foc_sincos(th, &s, &c);
        foc_dq_t dq = foc_park(ab, s, c);
        foc_ab_t back = foc_inv_park(dq, s, c);
        ASSERT_NEAR(back.alpha, ab.alpha, 1e-4f);
        ASSERT_NEAR(back.beta, ab.beta, 1e-4f);
        /* rotation preserves magnitude */
        ASSERT_NEAR(dq.d * dq.d + dq.q * dq.q, ab.alpha * ab.alpha + ab.beta * ab.beta, 1e-3f);
    }
}

/* A balanced three-phase set rotating with the reference frame must look
 * constant in dq, with the amplitude preserved and the phase lead mapping to
 * the q axis. This pins down the sign conventions. */
static void test_rotating_vector_is_constant_in_dq(void)
{
    const float I = 2.0f, phi = 0.7f;
    for (int k = 0; k < 360; k++) {
        float th = (float)k * FOC_TWO_PI / 360.0f;
        float ia = I * cosf(th + phi);
        float ib = I * cosf(th + phi - FOC_TWO_PI / 3.0f);
        float s, c;
        foc_sincos(th, &s, &c);
        foc_dq_t dq = foc_park(foc_clarke(ia, ib), s, c);
        ASSERT_NEAR(dq.d, I * cosf(phi), 1e-3f);
        ASSERT_NEAR(dq.q, I * sinf(phi), 1e-3f);
    }
}

static void test_pi_proportional_and_integral(void)
{
    foc_pi_t pi;
    foc_pi_init(&pi, 2.0f, 0.0f, -100.0f, 100.0f);
    ASSERT_NEAR(foc_pi_update(&pi, 1.5f, 0.001f), 3.0f, 1e-6f);
    ASSERT_NEAR(pi.integral, 0.0f, 0.0f);

    foc_pi_init(&pi, 0.0f, 10.0f, -100.0f, 100.0f);
    ASSERT_NEAR(foc_pi_update(&pi, 1.0f, 0.1f), 1.0f, 1e-6f);
    ASSERT_NEAR(foc_pi_update(&pi, 1.0f, 0.1f), 2.0f, 1e-6f);
    ASSERT_NEAR(foc_pi_update(&pi, 1.0f, 0.1f), 3.0f, 1e-6f);
    /* integral is dt-scaled: halving dt halves the growth */
    ASSERT_NEAR(foc_pi_update(&pi, 1.0f, 0.05f), 3.5f, 1e-6f);
    foc_pi_reset(&pi);
    ASSERT_NEAR(pi.integral, 0.0f, 0.0f);
}

static void test_pi_output_clamp(void)
{
    foc_pi_t pi;
    foc_pi_init(&pi, 10.0f, 0.0f, -1.0f, 1.0f);
    ASSERT_NEAR(foc_pi_update(&pi, 5.0f, 0.001f), 1.0f, 0.0f);
    ASSERT_NEAR(foc_pi_update(&pi, -5.0f, 0.001f), -1.0f, 0.0f);
}

static void test_pi_anti_windup(void)
{
    /* Saturate hard for a full second, then flip the error. Without
     * anti-windup the integrator would sit at +100 and the output would stay
     * pinned high for a long time. */
    foc_pi_t pi;
    foc_pi_init(&pi, 1.0f, 100.0f, -1.0f, 1.0f);
    for (int i = 0; i < 1000; i++) (void)foc_pi_update(&pi, 1.0f, 0.001f);
    ASSERT_TRUE(pi.integral <= 1.0f);
    float out = foc_pi_update(&pi, -0.5f, 0.001f);
    ASSERT_TRUE(out < 0.0f);
    /* and the integrator is never outside the output range */
    foc_pi_init(&pi, 0.0f, 1000.0f, -2.0f, 2.0f);
    for (int i = 0; i < 1000; i++) (void)foc_pi_update(&pi, 3.0f, 0.01f);
    ASSERT_NEAR(pi.integral, 2.0f, 1e-6f);
    for (int i = 0; i < 1000; i++) (void)foc_pi_update(&pi, -3.0f, 0.01f);
    ASSERT_NEAR(pi.integral, -2.0f, 1e-6f);
}

static void test_pi_tuning(void)
{
    foc_pi_t pi;
    foc_pi_init(&pi, 0.0f, 0.0f, -1.0f, 1.0f);
    foc_pi_tune_current(&pi, 5.0f, 2e-3f, 500.0f);
    ASSERT_NEAR(pi.kp, 2e-3f * FOC_TWO_PI * 500.0f, 1e-6f);
    ASSERT_NEAR(pi.ki, 5.0f * FOC_TWO_PI * 500.0f, 1e-2f);
}

static void test_ctrl_step_basics(void)
{
    foc_ctrl_t c;
    foc_ctrl_init(&c, 5.0f, 2e-3f, 500.0f);
    foc_dq_t ref = { 0.0f, 0.0f };
    /* zero error, zero currents -> centred duties */
    foc_abc_t d = foc_ctrl_step(&c, 0.0f, 0.0f, 0.3f, ref, 12.0f, 50e-6f);
    ASSERT_NEAR(d.a, 0.5f, 1e-6f);
    ASSERT_NEAR(d.b, 0.5f, 1e-6f);
    ASSERT_NEAR(d.c, 0.5f, 1e-6f);

    /* huge q error: voltage must respect the circle limit, duties stay in [0,1] */
    foc_ctrl_reset(&c);
    ref.q = 1000.0f;
    for (int k = 0; k < 100; k++) {
        float th = (float)k * 0.0628f;
        d = foc_ctrl_step(&c, 0.0f, 0.0f, th, ref, 12.0f, 50e-6f);
        float vmag = sqrtf(c.v_dq.d * c.v_dq.d + c.v_dq.q * c.v_dq.q);
        ASSERT_TRUE(vmag <= 12.0f * FOC_ONE_OVER_SQRT3 * 0.95f + 1e-3f);
        ASSERT_TRUE(d.a >= 0.0f && d.a <= 1.0f);
        ASSERT_TRUE(d.b >= 0.0f && d.b <= 1.0f);
        ASSERT_TRUE(d.c >= 0.0f && d.c <= 1.0f);
    }

    /* measured currents are reported in dq for telemetry */
    foc_ctrl_reset(&c);
    ref.q = 0.0f;
    float s, co;
    foc_sincos(1.0f, &s, &co);
    foc_dq_t want = { 0.4f, -0.9f };
    foc_abc_t iabc = foc_inv_clarke(foc_inv_park(want, s, co));
    (void)foc_ctrl_step(&c, iabc.a, iabc.b, 1.0f, ref, 12.0f, 50e-6f);
    ASSERT_NEAR(c.i_dq.d, want.d, 1e-4f);
    ASSERT_NEAR(c.i_dq.q, want.q, 1e-4f);
}

int main(void)
{
    srand(1);
    RUN_TEST(test_wrap);
    RUN_TEST(test_clarke_roundtrip);
    RUN_TEST(test_park_roundtrip);
    RUN_TEST(test_rotating_vector_is_constant_in_dq);
    RUN_TEST(test_pi_proportional_and_integral);
    RUN_TEST(test_pi_output_clamp);
    RUN_TEST(test_pi_anti_windup);
    RUN_TEST(test_pi_tuning);
    RUN_TEST(test_ctrl_step_basics);
    MINITEST_MAIN_END();
}
