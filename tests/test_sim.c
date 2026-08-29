/* Closed-loop tests: the FOC library driving the simulated motor. */
#include "calib.h"
#include "encoder.h"
#include "foc.h"
#include "impedance.h"
#include "motor_sim.h"
#include "svpwm.h"
#include "minitest.h"

#define DT   50e-6f
#define VBUS 12.0f
#define BW_HZ 500.0f

typedef struct {
    foc_ctrl_t  ctrl;
    motor_sim_t m;
    float t;
} loop_t;

static void loop_init(loop_t *l)
{
    motor_sim_init_default(&l->m);
    foc_ctrl_init(&l->ctrl, l->m.R, l->m.L, BW_HZ);
    l->t = 0.0f;
}

/* One tick using the true electrical angle plus an optional angle error. */
static void loop_tick(loop_t *l, foc_dq_t i_ref, float tau_load, float angle_err)
{
    float theta_e = foc_wrap_2pi(motor_sim_theta_e(&l->m) + angle_err);
    foc_abc_t d = foc_ctrl_step(&l->ctrl, l->m.i_abc.a, l->m.i_abc.b, theta_e, i_ref, VBUS, DT);
    motor_sim_step_duty(&l->m, d, VBUS, tau_load, DT);
    l->t += DT;
}

static void test_open_loop_alignment(void)
{
    motor_sim_t m;
    motor_sim_init_default(&m);
    m.theta_m = 0.3f;   /* start somewhere */
    float s, c;
    foc_sincos(0.0f, &s, &c);
    foc_dq_t v = { 2.0f, 0.0f };
    foc_abc_t duty = svpwm(foc_inv_park(v, s, c).alpha, foc_inv_park(v, s, c).beta, VBUS);
    for (int i = 0; i < 20000; i++) motor_sim_step_duty(&m, duty, VBUS, 0.0f, DT);   /* 1 s */
    /* rotor pulled to theta_e = 0 (within the friction dead band) */
    ASSERT_NEAR(foc_wrap_pi(motor_sim_theta_e(&m)), 0.0f, 0.15f);
    ASSERT_NEAR(m.i_d, 2.0f / m.R, 0.01f);
    ASSERT_NEAR(m.i_q, 0.0f, 0.01f);
    ASSERT_NEAR(m.omega_m, 0.0f, 0.5f);
}

static void test_current_step_locked_rotor(void)
{
    loop_t l;
    loop_init(&l);
    l.m.J = 1.0f;   /* effectively locked */
    foc_dq_t ref = { 0.0f, 0.5f };
    float t10 = -1.0f, t90 = -1.0f, peak = 0.0f, id_max = 0.0f;
    for (int i = 0; i < 400; i++) {   /* 20 ms */
        loop_tick(&l, ref, 0.0f, 0.0f);
        float iq = l.m.i_q;
        if (t10 < 0.0f && iq >= 0.05f) t10 = l.t;
        if (t90 < 0.0f && iq >= 0.45f) t90 = l.t;
        if (iq > peak) peak = iq;
        if (fabsf(l.m.i_d) > id_max) id_max = fabsf(l.m.i_d);
        if (l.t > 5e-3f) ASSERT_NEAR(iq, 0.5f, 0.005f);
    }
    ASSERT_TRUE(t10 > 0.0f && t90 > 0.0f);
    ASSERT_TRUE(t90 - t10 < 1.5e-3f);      /* 500 Hz bandwidth: ~0.7 ms */
    ASSERT_TRUE(peak < 0.5f * 1.05f);       /* < 5% overshoot */
    ASSERT_TRUE(id_max < 0.02f);
    /* the controller reports what it measured */
    ASSERT_NEAR(l.ctrl.i_dq.q, 0.5f, 0.01f);
    ASSERT_NEAR(l.ctrl.v_dq.q, 0.5f * l.m.R, 0.05f);   /* V = I R at standstill */
}

static void test_current_step_free_rotor_accelerates(void)
{
    loop_t l;
    loop_init(&l);
    foc_dq_t ref = { 0.0f, 0.5f };
    for (int i = 0; i < 1000; i++) {   /* 50 ms */
        loop_tick(&l, ref, 0.0f, 0.0f);
        if (l.t > 3e-3f) {
            ASSERT_NEAR(l.m.i_q, 0.5f, 0.03f);
            ASSERT_NEAR(l.m.i_d, 0.0f, 0.03f);
        }
    }
    ASSERT_TRUE(l.m.omega_m > 50.0f);
    /* back-EMF is showing up in v_q */
    ASSERT_TRUE(l.ctrl.v_dq.q > 0.5f * l.m.R + 1.0f);
}

static void test_torque_holds_matched_load(void)
{
    loop_t l;
    loop_init(&l);
    float tau_load = 0.03f;
    foc_dq_t ref = { 0.0f, tau_load / motor_sim_kt(&l.m) };
    for (int i = 0; i < 4000; i++) loop_tick(&l, ref, tau_load, 0.0f);   /* 200 ms */
    ASSERT_NEAR(l.m.omega_m, 0.0f, 1.0f);
    ASSERT_NEAR(l.m.theta_m, 0.0f, 0.3f);
    ASSERT_NEAR(l.m.tau_e, tau_load, 0.002f);
}

static void test_angle_error_kills_torque(void)
{
    /* Same command as the free-rotor test, but the controller thinks the
     * rotor is 90 electrical degrees from where it is: no torque. This is
     * why the calibration step exists. */
    loop_t l;
    loop_init(&l);
    foc_dq_t ref = { 0.0f, 0.5f };
    for (int i = 0; i < 1000; i++) loop_tick(&l, ref, 0.0f, FOC_PI / 2.0f);
    ASSERT_TRUE(fabsf(l.m.omega_m) < 5.0f);
}

static void run_calib_on_sim(motor_sim_t *m, calib_t *c)
{
    calib_start(c);
    encoder_t enc;
    encoder_init(&enc, m->enc_cpr, 0, 50.0f, DT);
    while (calib_running(c)) {
        encoder_update(&enc, motor_sim_encoder_raw(m), DT);
        calib_update(c, encoder_theta_mech(&enc), DT);
        motor_sim_step_duty(m, calib_duty(c, VBUS), VBUS, 0.0f, DT);
    }
}

static void check_calib_on_sim(float off, int dir, uint8_t hint)
{
    motor_sim_t m;
    motor_sim_init_default(&m);
    m.enc_offset_mech = off;
    m.enc_direction = (int8_t)dir;
    m.theta_m = 0.2f;
    calib_t c;
    calib_init(&c, 4.0f, hint);
    run_calib_on_sim(&m, &c);
    ASSERT_EQ_INT(c.state, CALIB_DONE);
    ASSERT_EQ_INT(c.direction, dir);
    ASSERT_EQ_INT(c.pole_pairs, m.pp);
    /* friction lets the rotor lock a little off the commanded angle:
     * asin(tau_f / (kt * i_d)) ~ 0.05 rad here */
    ASSERT_NEAR(foc_wrap_pi(c.offset_elec - motor_sim_expected_offset_elec(&m)), 0.0f, 0.1f);

    /* with the result loaded, the encoder tracks the true electrical angle
     * while the motor is driven around */
    encoder_t enc;
    encoder_init(&enc, m.enc_cpr, c.pole_pairs, 50.0f, DT);
    encoder_set_calibration(&enc, c.offset_elec, c.direction);
    foc_ctrl_t ctrl;
    foc_ctrl_init(&ctrl, m.R, m.L, BW_HZ);
    foc_dq_t ref = { 0.0f, 0.3f };
    float worst = 0.0f;
    for (int i = 0; i < 4000; i++) {
        encoder_update(&enc, motor_sim_encoder_raw(&m), DT);
        float th = encoder_theta_elec(&enc);
        float err = fabsf(foc_wrap_pi(th - motor_sim_theta_e(&m)));
        if (i > 100 && err > worst) worst = err;
        foc_abc_t d = foc_ctrl_step(&ctrl, m.i_abc.a, m.i_abc.b, th, ref, VBUS, DT);
        motor_sim_step_duty(&m, d, VBUS, 0.0f, DT);
    }
    ASSERT_TRUE(worst < 0.15f);
    ASSERT_TRUE(fabsf(m.omega_m) > 20.0f);   /* and it actually produces torque */
}

static void test_calibration_on_simulated_motor(void)
{
    check_calib_on_sim(0.0f, 1, 0);
    check_calib_on_sim(1.234f, 1, 11);
    check_calib_on_sim(1.234f, -1, 0);
    check_calib_on_sim(-2.9f, -1, 11);
}

/* Full haptic stack: encoder -> impedance law -> current loop -> motor, with
 * an external "finger" torque. The spring should deflect by tau/k. */
static void test_haptic_spring_deflection(void)
{
    motor_sim_t m;
    motor_sim_init_default(&m);
    m.enc_offset_mech = 0.7f;
    encoder_t enc;
    encoder_init(&enc, m.enc_cpr, m.pp, 50.0f, 4.0f * DT);
    encoder_set_calibration(&enc, motor_sim_expected_offset_elec(&m), m.enc_direction);
    foc_ctrl_t ctrl;
    foc_ctrl_init(&ctrl, m.R, m.L, BW_HZ);
    haptic_profile_t p = haptic_preset(HAPTIC_PRESET_SPRING);
    p.tau_max = 0.2f;
    float kt = motor_sim_kt(&m);
    float tau_ext = 0.03f;   /* Nm, pushes positive */

    encoder_update(&enc, motor_sim_encoder_raw(&m), 4.0f * DT);
    encoder_zero_turns(&enc);
    float theta_start = encoder_theta_cont(&enc);
    for (int i = 0; i < 40000; i++) {   /* 2 s */
        if (i % 4 == 0) encoder_update(&enc, motor_sim_encoder_raw(&m), 4.0f * DT);
        float theta = encoder_theta_cont(&enc) - theta_start;
        float tau = haptic_torque(&p, theta, encoder_velocity(&enc));
        foc_dq_t ref = { 0.0f, tau / kt };
        foc_abc_t d = foc_ctrl_step(&ctrl, m.i_abc.a, m.i_abc.b, encoder_theta_elec(&enc), ref, VBUS, DT);
        motor_sim_step_duty(&m, d, VBUS, -tau_ext, DT);   /* negative load = pushes positive */
    }
    float deflection = (float)m.enc_direction * (encoder_theta_cont(&enc) - theta_start);
    ASSERT_NEAR(deflection, tau_ext / p.k_spring, 0.1f);
    ASSERT_NEAR(m.omega_m, 0.0f, 0.5f);
}

int main(void)
{
    RUN_TEST(test_open_loop_alignment);
    RUN_TEST(test_current_step_locked_rotor);
    RUN_TEST(test_current_step_free_rotor_accelerates);
    RUN_TEST(test_torque_holds_matched_load);
    RUN_TEST(test_angle_error_kills_torque);
    RUN_TEST(test_calibration_on_simulated_motor);
    RUN_TEST(test_haptic_spring_deflection);
    MINITEST_MAIN_END();
}
