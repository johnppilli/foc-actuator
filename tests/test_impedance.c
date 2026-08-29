#include "impedance.h"
#include "minitest.h"

static haptic_profile_t blank(void)
{
    haptic_profile_t p = haptic_preset(HAPTIC_PRESET_FREE);
    p.b_damp = 0.0f;
    p.tau_max = 100.0f;
    return p;
}

static void test_spring(void)
{
    haptic_profile_t p = blank();
    p.k_spring = 0.5f;
    p.theta0 = 0.3f;
    ASSERT_NEAR(haptic_torque(&p, 0.3f, 0.0f), 0.0f, 1e-6f);
    ASSERT_NEAR(haptic_torque(&p, 0.3f + 0.2f, 0.0f), -0.1f, 1e-6f);
    ASSERT_NEAR(haptic_torque(&p, 0.3f - 0.2f, 0.0f), 0.1f, 1e-6f);
    /* odd symmetry about theta0 */
    for (int i = 1; i < 20; i++) {
        float x = 0.1f * (float)i;
        ASSERT_NEAR(haptic_torque(&p, 0.3f + x, 0.0f), -haptic_torque(&p, 0.3f - x, 0.0f), 1e-6f);
    }
}

static void test_damping_opposes_velocity(void)
{
    haptic_profile_t p = blank();
    p.b_damp = 0.02f;
    ASSERT_NEAR(haptic_torque(&p, 1.0f, 5.0f), -0.1f, 1e-6f);
    ASSERT_NEAR(haptic_torque(&p, -3.0f, -5.0f), 0.1f, 1e-6f);
}

static void test_sawtooth_detents(void)
{
    haptic_profile_t p = blank();
    p.n_detents = 12;
    p.k_detent = 1.0f;
    p.detent_shape = DETENT_SAWTOOTH;
    float pitch = FOC_TWO_PI / 12.0f;
    for (int k = -13; k <= 13; k++) {
        float centre = (float)k * pitch;
        ASSERT_NEAR(haptic_torque(&p, centre, 0.0f), 0.0f, 1e-4f);
        /* restoring on both sides */
        ASSERT_TRUE(haptic_torque(&p, centre + 0.05f, 0.0f) < 0.0f);
        ASSERT_TRUE(haptic_torque(&p, centre - 0.05f, 0.0f) > 0.0f);
        ASSERT_NEAR(haptic_torque(&p, centre + 0.05f, 0.0f), -0.05f, 1e-4f);
    }
    /* periodic */
    for (int i = 0; i < 50; i++) {
        float th = 0.013f * (float)i;
        ASSERT_NEAR(haptic_torque(&p, th, 0.0f), haptic_torque(&p, th + 3.0f * pitch, 0.0f), 1e-4f);
    }
    /* click: sign flips across the cell boundary */
    float b = 0.5f * pitch;
    ASSERT_TRUE(haptic_torque(&p, b - 1e-3f, 0.0f) < -0.2f);
    ASSERT_TRUE(haptic_torque(&p, b + 1e-3f, 0.0f) > 0.2f);
    /* phase shifts the pattern */
    p.detent_phase = 0.1f;
    ASSERT_NEAR(haptic_torque(&p, 0.1f, 0.0f), 0.0f, 1e-4f);
}

static void test_sine_detents(void)
{
    haptic_profile_t p = blank();
    p.n_detents = 24;
    p.k_detent = 2.0f;
    p.detent_shape = DETENT_SINE;
    float pitch = FOC_TWO_PI / 24.0f;
    for (int k = 0; k < 24; k++) {
        float centre = (float)k * pitch;
        ASSERT_NEAR(haptic_torque(&p, centre, 0.0f), 0.0f, 1e-4f);
        /* stiffness at the centre equals k_detent (numerical derivative) */
        float h = 1e-3f;
        float slope = (haptic_torque(&p, centre + h, 0.0f) - haptic_torque(&p, centre - h, 0.0f)) / (2.0f * h);
        ASSERT_NEAR(slope, -2.0f, 0.02f);
    }
    /* continuous: no jumps anywhere */
    float prev = haptic_torque(&p, 0.0f, 0.0f);
    for (int i = 1; i < 2000; i++) {
        float th = (float)i * FOC_TWO_PI / 2000.0f;
        float now = haptic_torque(&p, th, 0.0f);
        ASSERT_TRUE(fabsf(now - prev) < 0.02f);
        prev = now;
    }
}

static void test_endstops(void)
{
    haptic_profile_t p = blank();
    p.theta_min = -1.0f;
    p.theta_max = 1.0f;
    p.k_endstop = 2.0f;
    p.b_endstop = 0.1f;
    ASSERT_NEAR(haptic_torque(&p, 0.0f, 3.0f), 0.0f, 1e-6f);
    ASSERT_NEAR(haptic_torque(&p, 0.99f, 3.0f), 0.0f, 1e-6f);
    ASSERT_NEAR(haptic_torque(&p, 1.5f, 0.0f), -1.0f, 1e-5f);
    ASSERT_NEAR(haptic_torque(&p, 1.5f, 2.0f), -1.2f, 1e-5f);
    ASSERT_NEAR(haptic_torque(&p, -1.5f, 0.0f), 1.0f, 1e-5f);
    /* disabled when max <= min */
    p.theta_max = p.theta_min;
    ASSERT_NEAR(haptic_torque(&p, 5.0f, 0.0f), 0.0f, 1e-6f);
}

static void test_clamp(void)
{
    haptic_profile_t p = blank();
    p.k_spring = 100.0f;
    p.tau_max = 0.25f;
    ASSERT_NEAR(haptic_torque(&p, 10.0f, 0.0f), -0.25f, 0.0f);
    ASSERT_NEAR(haptic_torque(&p, -10.0f, 0.0f), 0.25f, 0.0f);
}

static void test_presets_are_sane(void)
{
    for (int i = 0; i < HAPTIC_PRESET_COUNT; i++) {
        haptic_profile_t p = haptic_preset((haptic_preset_t)i);
        ASSERT_TRUE(p.tau_max > 0.0f);
        ASSERT_TRUE(haptic_preset_name((haptic_preset_t)i)[0] != '?');
        for (int k = -50; k <= 50; k++) {
            float th = 0.1f * (float)k;
            float tau = haptic_torque(&p, th, 2.0f);
            ASSERT_TRUE(isfinite(tau));
            ASSERT_TRUE(fabsf(tau) <= p.tau_max);
        }
    }
    haptic_profile_t s = haptic_preset(HAPTIC_PRESET_SPRING);
    ASSERT_TRUE(haptic_torque(&s, 0.5f, 0.0f) < 0.0f);
    haptic_profile_t d = haptic_preset(HAPTIC_PRESET_DETENTS_12);
    ASSERT_TRUE(d.n_detents == 12);
    haptic_profile_t e = haptic_preset(HAPTIC_PRESET_ENDSTOPS);
    ASSERT_TRUE(haptic_torque(&e, 2.0f, 0.0f) < 0.0f);
    ASSERT_NEAR(haptic_torque(&e, 0.0f, 0.0f), 0.0f, 1e-6f);
}

int main(void)
{
    RUN_TEST(test_spring);
    RUN_TEST(test_damping_opposes_velocity);
    RUN_TEST(test_sawtooth_detents);
    RUN_TEST(test_sine_detents);
    RUN_TEST(test_endstops);
    RUN_TEST(test_clamp);
    RUN_TEST(test_presets_are_sane);
    MINITEST_MAIN_END();
}
