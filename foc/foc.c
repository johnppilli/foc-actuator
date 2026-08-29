#include "foc.h"
#include "svpwm.h"
#include <math.h>

/* ---- angle helpers ------------------------------------------------------ */

float foc_wrap_2pi(float theta)
{
    theta = fmodf(theta, FOC_TWO_PI);
    if (theta < 0.0f) theta += FOC_TWO_PI;
    /* fmodf can return exactly 2*pi after the add for tiny negatives */
    if (theta >= FOC_TWO_PI) theta -= FOC_TWO_PI;
    return theta;
}

float foc_wrap_pi(float theta)
{
    theta = foc_wrap_2pi(theta + FOC_PI) - FOC_PI;
    return theta;
}

float foc_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void foc_sincos(float theta, float *s, float *c)
{
    *s = sinf(theta);
    *c = cosf(theta);
}

/* ---- transforms --------------------------------------------------------- */

foc_ab_t foc_clarke(float ia, float ib)
{
    foc_ab_t out;
    out.alpha = ia;
    out.beta  = (ia + 2.0f * ib) * FOC_ONE_OVER_SQRT3;   /* == (ib - ic)/sqrt3 with ic = -ia-ib */
    return out;
}

foc_ab_t foc_clarke3(float ia, float ib, float ic)
{
    foc_ab_t out;
    out.alpha = (2.0f * ia - ib - ic) * (1.0f / 3.0f);
    out.beta  = (ib - ic) * FOC_ONE_OVER_SQRT3;
    return out;
}

foc_abc_t foc_inv_clarke(foc_ab_t ab)
{
    foc_abc_t out;
    out.a = ab.alpha;
    out.b = (-ab.alpha + FOC_SQRT3 * ab.beta) * 0.5f;
    out.c = (-ab.alpha - FOC_SQRT3 * ab.beta) * 0.5f;
    return out;
}

foc_dq_t foc_park(foc_ab_t ab, float sin_th, float cos_th)
{
    foc_dq_t out;
    out.d =  ab.alpha * cos_th + ab.beta * sin_th;
    out.q = -ab.alpha * sin_th + ab.beta * cos_th;
    return out;
}

foc_ab_t foc_inv_park(foc_dq_t dq, float sin_th, float cos_th)
{
    foc_ab_t out;
    out.alpha = dq.d * cos_th - dq.q * sin_th;
    out.beta  = dq.d * sin_th + dq.q * cos_th;
    return out;
}

/* ---- PI controller ------------------------------------------------------ */

void foc_pi_init(foc_pi_t *pi, float kp, float ki, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->integral = 0.0f;
}

void foc_pi_reset(foc_pi_t *pi)
{
    pi->integral = 0.0f;
}

float foc_pi_update(foc_pi_t *pi, float error, float dt)
{
    float p = pi->kp * error;
    float i_new = foc_clampf(pi->integral + pi->ki * error * dt, pi->out_min, pi->out_max);
    float out = p + i_new;

    if (out > pi->out_max) {
        out = pi->out_max;
        if (error > 0.0f) i_new = pi->integral;   /* saturated and pushing further: hold */
    } else if (out < pi->out_min) {
        out = pi->out_min;
        if (error < 0.0f) i_new = pi->integral;
    }
    pi->integral = i_new;
    return out;
}

void foc_pi_tune_current(foc_pi_t *pi, float R_ohm, float L_henry, float bandwidth_hz)
{
    float wc = FOC_TWO_PI * bandwidth_hz;
    pi->kp = L_henry * wc;
    pi->ki = R_ohm * wc;
}

/* ---- dq current controller --------------------------------------------- */

void foc_ctrl_init(foc_ctrl_t *c, float R_ohm, float L_henry, float bandwidth_hz)
{
    foc_pi_init(&c->pi_d, 0.0f, 0.0f, -1.0f, 1.0f);
    foc_pi_init(&c->pi_q, 0.0f, 0.0f, -1.0f, 1.0f);
    foc_pi_tune_current(&c->pi_d, R_ohm, L_henry, bandwidth_hz);
    foc_pi_tune_current(&c->pi_q, R_ohm, L_henry, bandwidth_hz);
    c->modulation_limit = 0.95f;
    foc_ctrl_reset(c);
}

void foc_ctrl_reset(foc_ctrl_t *c)
{
    foc_pi_reset(&c->pi_d);
    foc_pi_reset(&c->pi_q);
    c->i_ab.alpha = c->i_ab.beta = 0.0f;
    c->i_dq.d = c->i_dq.q = 0.0f;
    c->v_dq.d = c->v_dq.q = 0.0f;
    c->v_ab.alpha = c->v_ab.beta = 0.0f;
    c->duty.a = c->duty.b = c->duty.c = 0.5f;
}

foc_abc_t foc_ctrl_step(foc_ctrl_t *c, float ia, float ib, float theta_e,
                        foc_dq_t i_ref, float v_bus, float dt)
{
    float s, co;
    foc_sincos(theta_e, &s, &co);

    c->i_ab = foc_clarke(ia, ib);
    c->i_dq = foc_park(c->i_ab, s, co);

    /* Voltage budget: the SVPWM linear region is a circle of radius v_bus/sqrt3. */
    float v_max = v_bus * FOC_ONE_OVER_SQRT3 * c->modulation_limit;

    c->pi_d.out_min = -v_max;
    c->pi_d.out_max =  v_max;
    c->v_dq.d = foc_pi_update(&c->pi_d, i_ref.d - c->i_dq.d, dt);

    /* Whatever d used, q gets the rest of the circle. */
    float q_max = sqrtf(v_max * v_max - c->v_dq.d * c->v_dq.d);
    c->pi_q.out_min = -q_max;
    c->pi_q.out_max =  q_max;
    c->v_dq.q = foc_pi_update(&c->pi_q, i_ref.q - c->i_dq.q, dt);

    c->v_ab = foc_inv_park(c->v_dq, s, co);
    c->duty = svpwm(c->v_ab.alpha, c->v_ab.beta, v_bus);
    return c->duty;
}
