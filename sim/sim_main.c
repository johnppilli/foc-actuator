/*
 * sim_main.c - closed-loop scenarios for the desktop simulator.
 *
 *   ./build/sim <scenario> > out.csv
 *
 * Scenarios:
 *   step            iq step, rotor locked         (current-loop tuning)
 *   step-free       iq step, rotor free            (acceleration, back-EMF)
 *   load            constant iq, load torque step  (torque control holds)
 *   calib           encoder offset calibration     (see stderr for the result)
 *   haptic-spring   finger sweeps a virtual spring
 *   haptic-detents  finger sweeps through 12 detents
 *   haptic-endstops finger pushes into virtual walls
 *   openloop        the original open-loop rotating-voltage drive, for contrast
 *   haptic-curves   torque vs angle for every preset (no simulation)
 *
 * CSV: first line "# scenario=<name>", then a header row, then data.
 */
#include "calib.h"
#include "encoder.h"
#include "foc.h"
#include "impedance.h"
#include "motor_sim.h"
#include "svpwm.h"
#include <stdio.h>
#include <string.h>

#define DT    50e-6f
#define VBUS  12.0f
#define BW_HZ 500.0f
#define ENC_DIV 4          /* encoder read every 4th tick: 5 kHz */

static void header(const char *scenario, const char *cols)
{
    printf("# scenario=%s\n%s\n", scenario, cols);
}

static void scenario_step(bool locked)
{
    motor_sim_t m;
    motor_sim_init_default(&m);
    if (locked) m.J = 1.0f;
    foc_ctrl_t c;
    foc_ctrl_init(&c, m.R, m.L, BW_HZ);
    header(locked ? "step" : "step-free", "t,iq_ref,iq,id,vd,vq,omega,theta_e,duty_a,duty_b,duty_c");
    float t = 0.0f;
    int n = locked ? 200 : 2000;   /* 10 ms / 100 ms */
    for (int i = 0; i < n; i++) {
        foc_dq_t ref = { 0.0f, t >= 1e-3f ? 0.5f : 0.0f };
        foc_abc_t d = foc_ctrl_step(&c, m.i_abc.a, m.i_abc.b, motor_sim_theta_e(&m), ref, VBUS, DT);
        motor_sim_step_duty(&m, d, VBUS, 0.0f, DT);
        printf("%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n", (double)t, (double)ref.q, (double)m.i_q, (double)m.i_d,
               (double)c.v_dq.d, (double)c.v_dq.q, (double)m.omega_m, (double)motor_sim_theta_e(&m),
               (double)d.a, (double)d.b, (double)d.c);
        t += DT;
    }
}

static void scenario_load(void)
{
    motor_sim_t m;
    motor_sim_init_default(&m);
    foc_ctrl_t c;
    foc_ctrl_init(&c, m.R, m.L, BW_HZ);
    header("load", "t,iq_ref,iq,tau_e,tau_load,omega,theta_m");
    float t = 0.0f;
    float kt = motor_sim_kt(&m);
    foc_dq_t ref = { 0.0f, 0.4f };
    for (int i = 0; i < 6000; i++) {   /* 300 ms */
        float tau_load = (t >= 0.1f) ? kt * ref.q : 0.0f;   /* matched load arrives at 100 ms */
        foc_abc_t d = foc_ctrl_step(&c, m.i_abc.a, m.i_abc.b, motor_sim_theta_e(&m), ref, VBUS, DT);
        motor_sim_step_duty(&m, d, VBUS, tau_load, DT);
        printf("%g,%g,%g,%g,%g,%g,%g\n", (double)t, (double)ref.q, (double)m.i_q, (double)m.tau_e,
               (double)tau_load, (double)m.omega_m, (double)m.theta_m);
        t += DT;
    }
}

static void scenario_calib(void)
{
    motor_sim_t m;
    motor_sim_init_default(&m);
    m.enc_offset_mech = 1.234f;
    m.enc_direction = -1;
    m.theta_m = 0.2f;
    encoder_t enc;
    encoder_init(&enc, m.enc_cpr, 0, 50.0f, DT);
    calib_t c;
    calib_init(&c, 4.0f, 0);
    calib_start(&c);
    header("calib", "t,state,theta_e_cmd,v_d,theta_m_enc,mech_accum,id,iq");
    float t = 0.0f;
    int k = 0;
    while (calib_running(&c)) {
        encoder_update(&enc, motor_sim_encoder_raw(&m), DT);
        calib_update(&c, encoder_theta_mech(&enc), DT);
        motor_sim_step_duty(&m, calib_duty(&c, VBUS), VBUS, 0.0f, DT);
        if ((k++ % 20) == 0) {   /* 1 kHz is plenty for a plot */
            printf("%g,%d,%g,%g,%g,%g,%g,%g\n", (double)t, (int)c.state, (double)c.theta_e_cmd,
                   (double)c.v_d_cmd, (double)encoder_theta_mech(&enc), (double)c.mech_accum,
                   (double)m.i_d, (double)m.i_q);
        }
        t += DT;
    }
    fprintf(stderr, "calib: %s\n", calib_state_name(c.state));
    if (c.state == CALIB_DONE) {
        fprintf(stderr, "  pole pairs   : %d (true %d)\n", c.pole_pairs, m.pp);
        fprintf(stderr, "  direction    : %d (true %d)\n", c.direction, m.enc_direction);
        fprintf(stderr, "  offset_elec  : %.4f rad (expected %.4f, error %.4f)\n",
                (double)c.offset_elec, (double)motor_sim_expected_offset_elec(&m),
                (double)foc_wrap_pi(c.offset_elec - motor_sim_expected_offset_elec(&m)));
        fprintf(stderr, "  residual     : %.4f rad\n", (double)c.residual);
    } else {
        fprintf(stderr, "  reason: %s\n", c.fail_reason ? c.fail_reason : "?");
    }
}

/* A finger modelled as a stiff spring dragging the knob along a trajectory. */
static void scenario_haptic(const char *name, haptic_preset_t preset, int mode)
{
    motor_sim_t m;
    motor_sim_init_default(&m);
    m.enc_offset_mech = 0.7f;
    encoder_t enc;
    encoder_init(&enc, m.enc_cpr, m.pp, 50.0f, ENC_DIV * DT);
    encoder_set_calibration(&enc, motor_sim_expected_offset_elec(&m), m.enc_direction);
    foc_ctrl_t c;
    foc_ctrl_init(&c, m.R, m.L, BW_HZ);
    haptic_profile_t p = haptic_preset(preset);
    float kt = motor_sim_kt(&m);
    const float k_finger = 0.3f;

    encoder_update(&enc, motor_sim_encoder_raw(&m), ENC_DIV * DT);
    encoder_zero_turns(&enc);
    float theta_start = encoder_theta_cont(&enc);

    header(name, "t,theta,omega,theta_finger,tau_cmd,tau_ext,iq_ref,iq");
    float t = 0.0f;
    const float T = 4.0f;
    for (int i = 0; i < (int)(T / DT); i++) {
        if (i % ENC_DIV == 0) encoder_update(&enc, motor_sim_encoder_raw(&m), ENC_DIV * DT);
        float theta = encoder_theta_cont(&enc) - theta_start;
        float omega = encoder_velocity(&enc);
        float theta_finger;
        if (mode == 0)      theta_finger = 1.5f * sinf(FOC_TWO_PI * 0.5f * t);          /* back and forth */
        else if (mode == 1) theta_finger = FOC_TWO_PI * (t / T);                          /* one slow turn */
        else                theta_finger = 2.2f * sinf(FOC_TWO_PI * 0.4f * t);          /* into the walls */
        /* the knob sees the finger through the encoder direction */
        float tau_ext = k_finger * (theta_finger - theta);
        float tau_cmd = haptic_torque(&p, theta, omega);
        foc_dq_t ref = { 0.0f, tau_cmd / kt };
        foc_abc_t d = foc_ctrl_step(&c, m.i_abc.a, m.i_abc.b, encoder_theta_elec(&enc), ref, VBUS, DT);
        /* external torque acts on the true shaft; encoder direction maps it */
        motor_sim_step_duty(&m, d, VBUS, -(float)m.enc_direction * tau_ext, DT);
        if (i % 20 == 0) {
            printf("%g,%g,%g,%g,%g,%g,%g,%g\n", (double)t, (double)theta, (double)omega, (double)theta_finger,
                   (double)tau_cmd, (double)tau_ext, (double)ref.q, (double)c.i_dq.q);
        }
        t += DT;
    }
}

static void scenario_openloop(void)
{
    /* What firmware/main.c does today: a rotating voltage vector at fixed
     * amplitude, no feedback. amplitude 0.3 of half the bus, 0.2 rad per ms. */
    motor_sim_t m;
    motor_sim_init_default(&m);
    header("openloop", "t,theta_e_cmd,theta_e_true,slip,id,iq,omega");
    float t = 0.0f, theta = 0.0f;
    const float v_peak = 0.3f * 0.5f * VBUS;
    for (int i = 0; i < 20000; i++) {   /* 1 s */
        if (i % 20 == 0) theta = foc_wrap_2pi(theta + 0.2f);   /* 1 ms steps like HAL_Delay(1) */
        float s, c;
        foc_sincos(theta, &s, &c);
        foc_dq_t v = { v_peak, 0.0f };
        foc_ab_t ab = foc_inv_park(v, s, c);
        foc_abc_t d = svpwm_sine(ab.alpha, ab.beta, VBUS);
        motor_sim_step_duty(&m, d, VBUS, 0.0f, DT);
        if (i % 10 == 0) {
            printf("%g,%g,%g,%g,%g,%g,%g\n", (double)t, (double)theta, (double)motor_sim_theta_e(&m),
                   (double)foc_wrap_pi(theta - motor_sim_theta_e(&m)), (double)m.i_d, (double)m.i_q,
                   (double)m.omega_m);
        }
        t += DT;
    }
}

static void scenario_haptic_curves(void)
{
    printf("# scenario=haptic-curves\ntheta");
    for (int i = 0; i < HAPTIC_PRESET_COUNT; i++) printf(",%s", haptic_preset_name((haptic_preset_t)i));
    printf("\n");
    haptic_profile_t p[HAPTIC_PRESET_COUNT];
    for (int i = 0; i < HAPTIC_PRESET_COUNT; i++) p[i] = haptic_preset((haptic_preset_t)i);
    for (int k = -1000; k <= 1000; k++) {
        float th = FOC_PI * (float)k / 1000.0f;
        printf("%g", (double)th);
        for (int i = 0; i < HAPTIC_PRESET_COUNT; i++) printf(",%g", (double)haptic_torque(&p[i], th, 0.0f));
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    const char *s = argc > 1 ? argv[1] : "step";
    if (!strcmp(s, "step"))            scenario_step(true);
    else if (!strcmp(s, "step-free"))  scenario_step(false);
    else if (!strcmp(s, "load"))       scenario_load();
    else if (!strcmp(s, "calib"))      scenario_calib();
    else if (!strcmp(s, "haptic-spring"))   scenario_haptic("haptic-spring", HAPTIC_PRESET_SPRING, 0);
    else if (!strcmp(s, "haptic-detents"))  scenario_haptic("haptic-detents", HAPTIC_PRESET_DETENTS_12, 1);
    else if (!strcmp(s, "haptic-endstops")) scenario_haptic("haptic-endstops", HAPTIC_PRESET_ENDSTOPS, 2);
    else if (!strcmp(s, "openloop"))   scenario_openloop();
    else if (!strcmp(s, "haptic-curves")) scenario_haptic_curves();
    else {
        fprintf(stderr, "unknown scenario '%s'\n", s);
        return 2;
    }
    return 0;
}
