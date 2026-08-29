#include "encoder.h"
#include "foc.h"
#include "minitest.h"

static uint16_t raw_at(float theta_m, uint16_t cpr)
{
    float a = foc_wrap_2pi(theta_m);
    int32_t r = (int32_t)(a * (float)cpr / FOC_TWO_PI + 0.5f);
    return (uint16_t)(r % cpr);
}

static void test_raw_to_rad(void)
{
    ASSERT_NEAR(encoder_raw_to_rad(0, 4096), 0.0f, 0.0f);
    ASSERT_NEAR(encoder_raw_to_rad(1024, 4096), FOC_PI / 2.0f, 1e-5f);
    ASSERT_NEAR(encoder_raw_to_rad(2048, 4096), FOC_PI, 1e-5f);
    ASSERT_NEAR(encoder_raw_to_rad(4095, 4096), FOC_TWO_PI - FOC_TWO_PI / 4096.0f, 1e-5f);
}

static void test_delta_counts(void)
{
    ASSERT_EQ_INT(encoder_delta_counts(5, 4090, 4096), 11);
    ASSERT_EQ_INT(encoder_delta_counts(4090, 5, 4096), -11);
    ASSERT_EQ_INT(encoder_delta_counts(100, 50, 4096), 50);
    ASSERT_EQ_INT(encoder_delta_counts(50, 100, 4096), -50);
    ASSERT_EQ_INT(encoder_delta_counts(0, 0, 4096), 0);
    ASSERT_EQ_INT(encoder_delta_counts(2047, 0, 4096), 2047);
    ASSERT_EQ_INT(encoder_delta_counts(2049, 0, 4096), -2047);
}

static void test_lpf_alpha(void)
{
    ASSERT_NEAR(encoder_lpf_alpha(0.0f, 1e-3f), 1.0f, 0.0f);
    float a = encoder_lpf_alpha(50.0f, 200e-6f);
    ASSERT_TRUE(a > 0.0f && a < 1.0f);
    ASSERT_NEAR(a, 200e-6f / (200e-6f + 1.0f / (FOC_TWO_PI * 50.0f)), 1e-6f);
}

static void test_constant_rotation_velocity_and_turns(void)
{
    const float dt = 200e-6f, w = 20.0f;
    encoder_t e;
    encoder_init(&e, 4096, 7, 50.0f, dt);
    float t = 0.0f;
    float worst = 0.0f;
    for (int i = 0; i < 5000; i++) {   /* 1 s, ~3.2 turns */
        t += dt;
        encoder_update(&e, raw_at(w * t, 4096), dt);
        if (t > 0.1f) {
            float err = fabsf(encoder_velocity(&e) - w);
            if (err > worst) worst = err;
        }
    }
    ASSERT_TRUE(worst < 0.05f * w);
    ASSERT_NEAR(encoder_theta_cont(&e), w * t, 0.01f);
    ASSERT_EQ_INT(e.turns, (long)floorf(w * t / FOC_TWO_PI));
    ASSERT_NEAR(encoder_theta_mech(&e), foc_wrap_2pi(w * t), 0.01f);
}

static void test_reverse_rotation_through_zero_no_spike(void)
{
    const float dt = 200e-6f, w = -15.0f;
    encoder_t e;
    encoder_init(&e, 4096, 7, 50.0f, dt);
    float t = 0.0f, worst = 0.0f;
    encoder_update(&e, raw_at(0.3f, 4096), dt);  /* start just above zero, moving down */
    for (int i = 0; i < 2000; i++) {
        t += dt;
        encoder_update(&e, raw_at(0.3f + w * t, 4096), dt);
        if (fabsf(encoder_velocity(&e)) > worst) worst = fabsf(encoder_velocity(&e));
    }
    ASSERT_TRUE(worst < 1.2f * fabsf(w));
    ASSERT_NEAR(encoder_velocity(&e), w, 0.05f * fabsf(w));
    ASSERT_NEAR(encoder_theta_cont(&e), 0.3f + w * t, 0.01f);
    ASSERT_TRUE(e.turns < 0);
}

static void test_zero_turns(void)
{
    encoder_t e;
    encoder_init(&e, 4096, 7, 50.0f, 1e-3f);
    encoder_update(&e, 100, 1e-3f);
    encoder_update(&e, 4000, 1e-3f);   /* went backwards through zero */
    ASSERT_TRUE(e.turns == -1);
    encoder_zero_turns(&e);
    ASSERT_EQ_INT(e.turns, 0);
    ASSERT_NEAR(encoder_theta_cont(&e), encoder_theta_mech(&e), 0.0f);
}

static void test_electrical_angle(void)
{
    encoder_t e;
    encoder_init(&e, 4096, 7, 50.0f, 1e-3f);
    encoder_set_calibration(&e, 1.0f, 1);
    encoder_update(&e, 0, 1e-3f);
    ASSERT_NEAR(encoder_theta_elec(&e), foc_wrap_2pi(-1.0f), 1e-5f);
    encoder_update(&e, 2048, 1e-3f);
    ASSERT_NEAR(encoder_theta_elec(&e), foc_wrap_2pi(7.0f * FOC_PI - 1.0f), 1e-4f);

    encoder_set_calibration(&e, 1.0f, -1);
    ASSERT_NEAR(encoder_theta_elec(&e), foc_wrap_2pi(-7.0f * FOC_PI - 1.0f), 1e-4f);

    /* a mechanical turn is pole_pairs electrical turns */
    encoder_set_calibration(&e, 0.0f, 1);
    float prev = 0.0f;
    int elec_wraps = 0;
    for (int r = 0; r < 4096; r += 8) {
        encoder_update(&e, (uint16_t)r, 1e-3f);
        float th = encoder_theta_elec(&e);
        if (th < prev) elec_wraps++;
        prev = th;
    }
    ASSERT_EQ_INT(elec_wraps, 6);   /* 7 electrical turns => 6 wraps inside [0, 2pi) */
}

static void test_extrapolation(void)
{
    const float dt = 200e-6f, w = 10.0f;
    encoder_t e;
    encoder_init(&e, 4096, 7, 0.0f, dt);   /* no filtering: exact velocity */
    float t = 0.0f;
    for (int i = 0; i < 1000; i++) {
        t += dt;
        encoder_update(&e, raw_at(w * t, 4096), dt);
    }
    ASSERT_NEAR(encoder_velocity_elec(&e), 7.0f * w, 7.0f * w * 0.1f);
    float th0 = encoder_theta_elec(&e);
    float th1 = encoder_theta_elec_extrap(&e, 100e-6f);
    ASSERT_NEAR(foc_wrap_pi(th1 - th0), 7.0f * encoder_velocity(&e) * 100e-6f, 1e-4f);
}

int main(void)
{
    RUN_TEST(test_raw_to_rad);
    RUN_TEST(test_delta_counts);
    RUN_TEST(test_lpf_alpha);
    RUN_TEST(test_constant_rotation_velocity_and_turns);
    RUN_TEST(test_reverse_rotation_through_zero_no_spike);
    RUN_TEST(test_zero_turns);
    RUN_TEST(test_electrical_angle);
    RUN_TEST(test_extrapolation);
    MINITEST_MAIN_END();
}
