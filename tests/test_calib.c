#include "calib.h"
#include "encoder.h"
#include "minitest.h"

/* Ideal rotor: mechanical angle follows the commanded electrical angle
 * through a first-order lag; the encoder sees it through an arbitrary
 * mounting (offset + direction). */
typedef struct {
    float pp;
    float theta_m;      /* true mechanical angle */
    float tau;          /* lag */
    float enc_off;
    int   enc_dir;
    bool  stuck;
} ideal_rotor_t;

static float ideal_rotor_encoder(const ideal_rotor_t *r)
{
    return foc_wrap_2pi((float)r->enc_dir * r->theta_m + r->enc_off);
}

static void ideal_rotor_step(ideal_rotor_t *r, float theta_e_cmd, float v_d, float dt)
{
    if (r->stuck || v_d <= 0.0f) return;
    float target = theta_e_cmd / r->pp;
    /* follow the nearest equivalent of the target (rotor locks to any pole) */
    float err = target - r->theta_m;
    float period = FOC_TWO_PI / r->pp;
    err -= period * floorf(err / period + 0.5f);
    r->theta_m += err * (dt / r->tau);
}

static void run_calib(calib_t *c, ideal_rotor_t *r, float dt, float max_s)
{
    calib_start(c);
    float t = 0.0f;
    while (calib_running(c) && t < max_s) {
        calib_update(c, ideal_rotor_encoder(r), dt);
        ideal_rotor_step(r, c->theta_e_cmd, c->v_d_cmd, dt);
        t += dt;
    }
}

static void check_recovers(int pp, float off, int dir, uint8_t hint)
{
    ideal_rotor_t r = { (float)pp, 0.4f, 0.02f, off, dir, false };
    calib_t c;
    calib_init(&c, 2.0f, hint);
    run_calib(&c, &r, 50e-6f, 10.0f);
    ASSERT_EQ_INT(c.state, CALIB_DONE);
    ASSERT_EQ_INT(c.direction, dir);
    ASSERT_EQ_INT(c.pole_pairs, pp);
    float expect = foc_wrap_2pi((float)dir * (float)pp * off);
    ASSERT_NEAR(foc_wrap_pi(c.offset_elec - expect), 0.0f, 0.02f);
    ASSERT_TRUE(fabsf(c.residual) < 0.02f);
    ASSERT_NEAR(c.v_d_cmd, 0.0f, 0.0f);

    /* and the encoder, once calibrated, reports theta_e = 0 where we locked */
    encoder_t e;
    encoder_init(&e, 4096, (uint8_t)pp, 50.0f, 50e-6f);
    encoder_set_calibration(&e, c.offset_elec, c.direction);
    /* lock rotor at theta_e = 1.0 and read the encoder */
    ideal_rotor_t r2 = r;
    for (int i = 0; i < 20000; i++) ideal_rotor_step(&r2, 1.0f, 2.0f, 50e-6f);
    float enc = ideal_rotor_encoder(&r2);
    uint16_t raw = (uint16_t)((int)(enc * 4096.0f / FOC_TWO_PI + 0.5f) % 4096);
    encoder_update(&e, raw, 50e-6f);
    ASSERT_NEAR(foc_wrap_pi(encoder_theta_elec(&e) - 1.0f), 0.0f, 0.03f);
}

static void test_recovers_offset_direction_pole_pairs(void)
{
    check_recovers(7, 0.0f, 1, 0);
    check_recovers(7, 1.234f, 1, 0);
    check_recovers(7, 1.234f, -1, 0);
    check_recovers(11, -2.5f, -1, 0);
    check_recovers(11, 3.0f, 1, 11);   /* with a correct hint */
    check_recovers(1, 0.7f, 1, 0);     /* single pole pair: full mech turn */
    check_recovers(21, 0.1f, -1, 0);
}

static void test_fails_when_rotor_does_not_move(void)
{
    ideal_rotor_t r = { 7.0f, 0.4f, 0.02f, 0.0f, 1, true };
    calib_t c;
    calib_init(&c, 2.0f, 7);
    run_calib(&c, &r, 50e-6f, 10.0f);
    ASSERT_EQ_INT(c.state, CALIB_FAILED);
    ASSERT_TRUE(c.fail_reason != 0);
    ASSERT_NEAR(c.v_d_cmd, 0.0f, 0.0f);
}

static void test_fails_on_pole_pair_mismatch(void)
{
    ideal_rotor_t r = { 7.0f, 0.4f, 0.02f, 0.0f, 1, false };
    calib_t c;
    calib_init(&c, 2.0f, 11);
    run_calib(&c, &r, 50e-6f, 10.0f);
    ASSERT_EQ_INT(c.state, CALIB_FAILED);
    ASSERT_TRUE(c.fail_reason != 0);
}

static void test_commands_are_gentle(void)
{
    /* voltage ramps rather than steps, and angle advances smoothly */
    ideal_rotor_t r = { 7.0f, 0.0f, 0.02f, 0.0f, 1, false };
    calib_t c;
    calib_init(&c, 3.0f, 7);
    calib_start(&c);
    float prev_v = 0.0f, prev_th = 0.0f, max_dv = 0.0f, max_dth = 0.0f;
    float dt = 50e-6f;
    while (calib_running(&c)) {
        calib_update(&c, ideal_rotor_encoder(&r), dt);
        ideal_rotor_step(&r, c.theta_e_cmd, c.v_d_cmd, dt);
        float dv = fabsf(c.v_d_cmd - prev_v);
        float dth = fabsf(foc_wrap_pi(c.theta_e_cmd - prev_th));
        if (dv > max_dv) max_dv = dv;
        if (dth > max_dth) max_dth = dth;
        prev_v = c.v_d_cmd;
        prev_th = c.theta_e_cmd;
        ASSERT_TRUE(c.v_d_cmd >= 0.0f && c.v_d_cmd <= 3.0f + 1e-6f);
    }
    ASSERT_TRUE(max_dv < 0.01f);
    ASSERT_TRUE(max_dth < 0.01f);
    ASSERT_EQ_INT(c.state, CALIB_DONE);
}

static void test_duty_output(void)
{
    calib_t c;
    calib_init(&c, 2.0f, 7);
    c.theta_e_cmd = 0.0f;
    c.v_d_cmd = 2.0f;
    foc_abc_t d = calib_duty(&c, 12.0f);
    /* v_d on the d axis at theta_e = 0 is a pure alpha voltage: phase a high */
    ASSERT_TRUE(d.a > 0.5f && d.b < 0.5f && d.c < 0.5f);
    ASSERT_NEAR(d.b, d.c, 1e-6f);
    ASSERT_TRUE(calib_state_name(CALIB_DONE)[0] == 'd');
}

int main(void)
{
    RUN_TEST(test_recovers_offset_direction_pole_pairs);
    RUN_TEST(test_fails_when_rotor_does_not_move);
    RUN_TEST(test_fails_on_pole_pair_mismatch);
    RUN_TEST(test_commands_are_gentle);
    RUN_TEST(test_duty_output);
    MINITEST_MAIN_END();
}
