#include "svpwm.h"
#include "minitest.h"

static const float VBUS = 12.0f;

static void test_zero_vector(void)
{
    foc_abc_t d = svpwm(0.0f, 0.0f, VBUS);
    ASSERT_NEAR(d.a, 0.5f, 1e-6f);
    ASSERT_NEAR(d.b, 0.5f, 1e-6f);
    ASSERT_NEAR(d.c, 0.5f, 1e-6f);
}

/* The motor only sees phase-to-neutral voltages; whatever common mode we
 * inject must cancel. Reconstruct alpha/beta from the duties and compare. */
static void test_reconstruction_in_linear_region(void)
{
    const float mags[] = { 0.1f, 0.5f, 0.9f, 1.0f };
    for (unsigned mi = 0; mi < sizeof(mags) / sizeof(mags[0]); mi++) {
        float mag = mags[mi] * SVPWM_MAX_MAG(VBUS);
        for (int k = 0; k < 360; k++) {
            float th = (float)k * FOC_TWO_PI / 360.0f;
            float va = mag * cosf(th), vb = mag * sinf(th);
            foc_abc_t d = svpwm(va, vb, VBUS);
            ASSERT_TRUE(d.a >= 0.0f && d.a <= 1.0f);
            ASSERT_TRUE(d.b >= 0.0f && d.b <= 1.0f);
            ASSERT_TRUE(d.c >= 0.0f && d.c <= 1.0f);
            foc_ab_t back = foc_clarke3(
                svpwm_duty_to_phase_voltage(d, VBUS).a,
                svpwm_duty_to_phase_voltage(d, VBUS).b,
                svpwm_duty_to_phase_voltage(d, VBUS).c);
            ASSERT_NEAR(back.alpha, va, 1e-3f * VBUS);
            ASSERT_NEAR(back.beta, vb, 1e-3f * VBUS);
        }
    }
}

static void test_matches_classic_implementation(void)
{
    for (int k = 0; k < 720; k++) {
        float th = (float)k * FOC_TWO_PI / 720.0f;
        for (int mi = 1; mi <= 10; mi++) {
            float mag = 0.1f * (float)mi * SVPWM_MAX_MAG(VBUS);
            foc_abc_t a = svpwm(mag * cosf(th), mag * sinf(th), VBUS);
            foc_abc_t b = svpwm_classic(mag * cosf(th), mag * sinf(th), VBUS);
            ASSERT_NEAR(a.a, b.a, 2e-3f);
            ASSERT_NEAR(a.b, b.b, 2e-3f);
            ASSERT_NEAR(a.c, b.c, 2e-3f);
        }
    }
}

static void test_sector(void)
{
    for (int k = 0; k < 360; k++) {
        float th = ((float)k + 0.5f) * FOC_TWO_PI / 360.0f;  /* avoid exact boundaries */
        int want = k / 60 + 1;
        ASSERT_EQ_INT(svpwm_sector(cosf(th), sinf(th)), want);
    }
    ASSERT_EQ_INT(svpwm_sector(0.0f, 0.0f), 1);
}

/* SVPWM reaches a phase peak of v_bus/sqrt3; sine PWM only v_bus/2. At the
 * SVPWM limit the sine version clips and mis-reproduces the command. */
static void test_15_percent_more_voltage_than_sine(void)
{
    float mag = SVPWM_MAX_MAG(VBUS);
    ASSERT_NEAR(mag / (0.5f * VBUS), 1.1547f, 1e-3f);

    float worst_sv = 0.0f, worst_sine = 0.0f;
    for (int k = 0; k < 360; k++) {
        float th = (float)k * FOC_TWO_PI / 360.0f;
        float va = mag * cosf(th), vb = mag * sinf(th);
        foc_abc_t dsv = svpwm(va, vb, VBUS);
        foc_abc_t dsi = svpwm_sine(va, vb, VBUS);
        foc_abc_t psv = svpwm_duty_to_phase_voltage(dsv, VBUS);
        foc_abc_t psi = svpwm_duty_to_phase_voltage(dsi, VBUS);
        foc_ab_t bsv = foc_clarke3(psv.a, psv.b, psv.c);
        foc_ab_t bsi = foc_clarke3(psi.a, psi.b, psi.c);
        float esv = fabsf(bsv.alpha - va) + fabsf(bsv.beta - vb);
        float esi = fabsf(bsi.alpha - va) + fabsf(bsi.beta - vb);
        if (esv > worst_sv) worst_sv = esv;
        if (esi > worst_sine) worst_sine = esi;
    }
    ASSERT_TRUE(worst_sv < 1e-2f);
    ASSERT_TRUE(worst_sine > 0.5f);   /* sine PWM clips badly at this amplitude */

    /* at the limit, some phase must hit (nearly) full duty */
    foc_abc_t d = svpwm(mag, 0.0f, VBUS);
    ASSERT_NEAR(d.a, 0.5f + 0.5f * FOC_SQRT3 / 2.0f, 1e-3f);   /* 0.933 */
}

static void test_overmodulation_keeps_angle(void)
{
    for (int k = 0; k < 36; k++) {
        float th = (float)k * FOC_TWO_PI / 36.0f;
        float mag = 3.0f * SVPWM_MAX_MAG(VBUS);
        foc_abc_t d = svpwm(mag * cosf(th), mag * sinf(th), VBUS);
        ASSERT_TRUE(d.a >= 0.0f && d.a <= 1.0f);
        ASSERT_TRUE(d.b >= 0.0f && d.b <= 1.0f);
        ASSERT_TRUE(d.c >= 0.0f && d.c <= 1.0f);
        foc_abc_t p = svpwm_duty_to_phase_voltage(d, VBUS);
        foc_ab_t back = foc_clarke3(p.a, p.b, p.c);
        float back_mag = sqrtf(back.alpha * back.alpha + back.beta * back.beta);
        float back_th = foc_wrap_2pi(atan2f(back.beta, back.alpha));
        ASSERT_NEAR(back_mag, SVPWM_MAX_MAG(VBUS), 1e-2f);
        ASSERT_NEAR(foc_wrap_pi(back_th - th), 0.0f, 1e-3f);
    }
}

int main(void)
{
    RUN_TEST(test_zero_vector);
    RUN_TEST(test_reconstruction_in_linear_region);
    RUN_TEST(test_matches_classic_implementation);
    RUN_TEST(test_sector);
    RUN_TEST(test_15_percent_more_voltage_than_sine);
    RUN_TEST(test_overmodulation_keeps_angle);
    MINITEST_MAIN_END();
}
