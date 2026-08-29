#include "motor_sim.h"
#include "svpwm.h"
#include <math.h>

void motor_sim_init_default(motor_sim_t *m)
{
    m->R = 5.6f;
    m->L = 1.8e-3f;
    m->pp = 11;
    m->lambda = 0.0048f;          /* kt = 1.5*11*0.0048 = 0.079 Nm/A */
    m->J = 2.0e-5f;
    m->B = 2.0e-5f;
    m->tau_coulomb = 0.003f;
    m->enc_cpr = 4096;
    m->enc_offset_mech = 0.0f;
    m->enc_direction = 1;
    m->substeps = 10;
    m->i_d = m->i_q = 0.0f;
    m->omega_m = 0.0f;
    m->theta_m = 0.0f;
    m->tau_e = 0.0f;
    m->i_abc.a = m->i_abc.b = m->i_abc.c = 0.0f;
}

float motor_sim_kt(const motor_sim_t *m)
{
    return 1.5f * (float)m->pp * m->lambda;
}

float motor_sim_theta_e(const motor_sim_t *m)
{
    return foc_wrap_2pi((float)m->pp * m->theta_m);
}

void motor_sim_step_abc(motor_sim_t *m, foc_abc_t v_phase, float tau_load, float dt)
{
    int n = m->substeps > 0 ? m->substeps : 1;
    float h = dt / (float)n;
    float kt = motor_sim_kt(m);
    foc_ab_t v_ab = foc_clarke3(v_phase.a, v_phase.b, v_phase.c);

    for (int k = 0; k < n; k++) {
        float theta_e = (float)m->pp * m->theta_m;
        float s, c;
        foc_sincos(theta_e, &s, &c);
        foc_dq_t v = foc_park(v_ab, s, c);
        float w_e = (float)m->pp * m->omega_m;

        float did = (v.d - m->R * m->i_d + w_e * m->L * m->i_q) / m->L;
        float diq = (v.q - m->R * m->i_q - w_e * m->L * m->i_d - w_e * m->lambda) / m->L;

        m->tau_e = kt * m->i_q;
        /* coulomb friction with a small viscous core so it does not chatter */
        float w_eps = 0.05f;
        float tau_f = m->tau_coulomb * foc_clampf(m->omega_m / w_eps, -1.0f, 1.0f);
        float dw = (m->tau_e - m->B * m->omega_m - tau_f - tau_load) / m->J;

        m->i_d += did * h;
        m->i_q += diq * h;
        m->omega_m += dw * h;
        m->theta_m += m->omega_m * h;
    }

    /* phase currents for the controller to measure */
    float theta_e = (float)m->pp * m->theta_m;
    float s, c;
    foc_sincos(theta_e, &s, &c);
    foc_dq_t i_dq = { m->i_d, m->i_q };
    foc_ab_t i_ab = foc_inv_park(i_dq, s, c);
    m->i_abc = foc_inv_clarke(i_ab);
}

void motor_sim_step_duty(motor_sim_t *m, foc_abc_t duty, float v_bus, float tau_load, float dt)
{
    motor_sim_step_abc(m, svpwm_duty_to_phase_voltage(duty, v_bus), tau_load, dt);
}

uint16_t motor_sim_encoder_raw(const motor_sim_t *m)
{
    float a = foc_wrap_2pi((float)m->enc_direction * m->theta_m + m->enc_offset_mech);
    float counts = a * (float)m->enc_cpr / FOC_TWO_PI;
    int32_t r = (int32_t)(counts + 0.5f);
    if (r >= (int32_t)m->enc_cpr) r -= (int32_t)m->enc_cpr;
    if (r < 0) r += (int32_t)m->enc_cpr;
    return (uint16_t)r;
}

float motor_sim_expected_offset_elec(const motor_sim_t *m)
{
    /* encoder reads theta_enc = dir*theta_m + off; true theta_e = pp*theta_m.
     * theta_e = dir*pp*theta_enc - offset_e  =>  offset_e = dir*pp*off */
    return foc_wrap_2pi((float)m->enc_direction * (float)m->pp * m->enc_offset_mech);
}
