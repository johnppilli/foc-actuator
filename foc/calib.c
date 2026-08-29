#include "calib.h"
#include "svpwm.h"
#include <math.h>

void calib_init(calib_t *c, float v_align, uint8_t pole_pairs_hint)
{
    c->v_align = v_align;
    c->ramp_s = 0.3f;
    c->settle_s = 0.5f;
    c->measure_s = 0.2f;
    c->rotate_s = 2.0f;
    c->pole_pairs_hint = pole_pairs_hint;
    c->offset_elec = 0.0f;
    c->direction = 1;
    c->pole_pairs = pole_pairs_hint;
    c->mech_per_elec_rev = 0.0f;
    c->residual = 0.0f;
    c->fail_reason = 0;
    c->state = CALIB_IDLE;
    c->theta_e_cmd = 0.0f;
    c->v_d_cmd = 0.0f;
    c->t = 0.0f;
    c->sum_sin = c->sum_cos = 0.0f;
    c->n_avg = 0;
    c->theta_m_zero = 0.0f;
    c->theta_m_prev = 0.0f;
    c->mech_accum = 0.0f;
    c->have_prev = false;
}

static void enter(calib_t *c, calib_state_t s)
{
    c->state = s;
    c->t = 0.0f;
    if (s == CALIB_MEASURE_ZERO || s == CALIB_MEASURE_END) {
        c->sum_sin = c->sum_cos = 0.0f;
        c->n_avg = 0;
    }
}

static void fail(calib_t *c, const char *why)
{
    c->fail_reason = why;
    c->v_d_cmd = 0.0f;
    enter(c, CALIB_FAILED);
}

void calib_start(calib_t *c)
{
    c->fail_reason = 0;
    c->mech_accum = 0.0f;
    c->have_prev = false;
    c->theta_e_cmd = 0.0f;
    c->v_d_cmd = 0.0f;
    enter(c, CALIB_RAMP_UP);
}

bool calib_running(const calib_t *c)
{
    return c->state != CALIB_IDLE && c->state != CALIB_DONE && c->state != CALIB_FAILED;
}

static float averaged_angle(const calib_t *c)
{
    return foc_wrap_2pi(atan2f(c->sum_sin, c->sum_cos));
}

static void finish(calib_t *c, float theta_m_end)
{
    (void)theta_m_end;
    float moved = c->mech_accum;
    if (fabsf(moved) < 0.05f) {
        fail(c, "rotor did not move (v_align too low, motor disconnected, or encoder stuck)");
        return;
    }
    int8_t dir = (moved > 0.0f) ? 1 : -1;
    float pp_f = FOC_TWO_PI / fabsf(moved);
    int pp_est = (int)(pp_f + 0.5f);
    if (pp_est < 1) pp_est = 1;
    if (pp_est > 64) {
        fail(c, "implausible pole-pair estimate");
        return;
    }
    if (c->pole_pairs_hint != 0 && pp_est != (int)c->pole_pairs_hint) {
        fail(c, "pole-pair estimate does not match hint");
        return;
    }
    c->pole_pairs = (uint8_t)pp_est;
    c->direction = dir;
    c->mech_per_elec_rev = moved;
    c->residual = fabsf(moved) - FOC_TWO_PI / (float)pp_est;
    c->offset_elec = foc_wrap_2pi((float)dir * (float)pp_est * c->theta_m_zero);
    enter(c, CALIB_RAMP_DOWN);
}

void calib_update(calib_t *c, float theta_m, float dt)
{
    if (!calib_running(c)) {
        c->v_d_cmd = 0.0f;
        return;
    }

    /* Track mechanical motion continuously once we have a reference. */
    if (c->have_prev) {
        c->mech_accum += foc_wrap_pi(theta_m - c->theta_m_prev);
    }
    c->theta_m_prev = theta_m;

    c->t += dt;

    switch (c->state) {
    case CALIB_RAMP_UP:
        c->theta_e_cmd = 0.0f;
        c->v_d_cmd = c->v_align * foc_clampf(c->t / c->ramp_s, 0.0f, 1.0f);
        if (c->t >= c->ramp_s) enter(c, CALIB_SETTLE_ZERO);
        break;

    case CALIB_SETTLE_ZERO:
        c->v_d_cmd = c->v_align;
        if (c->t >= c->settle_s) enter(c, CALIB_MEASURE_ZERO);
        break;

    case CALIB_MEASURE_ZERO:
        c->sum_sin += sinf(theta_m);
        c->sum_cos += cosf(theta_m);
        c->n_avg++;
        if (c->t >= c->measure_s) {
            c->theta_m_zero = averaged_angle(c);
            /* start accumulating motion from here */
            c->mech_accum = 0.0f;
            c->have_prev = true;
            c->theta_m_prev = theta_m;
            enter(c, CALIB_ROTATE);
        }
        break;

    case CALIB_ROTATE:
        c->theta_e_cmd = FOC_TWO_PI * foc_clampf(c->t / c->rotate_s, 0.0f, 1.0f);
        if (c->t >= c->rotate_s) {
            c->theta_e_cmd = FOC_TWO_PI;   /* same as 0 electrically */
            enter(c, CALIB_SETTLE_END);
        }
        break;

    case CALIB_SETTLE_END:
        if (c->t >= c->settle_s) enter(c, CALIB_MEASURE_END);
        break;

    case CALIB_MEASURE_END:
        c->sum_sin += sinf(theta_m);
        c->sum_cos += cosf(theta_m);
        c->n_avg++;
        if (c->t >= c->measure_s) finish(c, averaged_angle(c));
        break;

    case CALIB_RAMP_DOWN:
        c->v_d_cmd = c->v_align * (1.0f - foc_clampf(c->t / c->ramp_s, 0.0f, 1.0f));
        if (c->t >= c->ramp_s) {
            c->v_d_cmd = 0.0f;
            enter(c, CALIB_DONE);
        }
        break;

    default:
        break;
    }
}

foc_abc_t calib_duty(const calib_t *c, float v_bus)
{
    float s, co;
    foc_sincos(c->theta_e_cmd, &s, &co);
    foc_dq_t v = { c->v_d_cmd, 0.0f };
    foc_ab_t ab = foc_inv_park(v, s, co);
    return svpwm(ab.alpha, ab.beta, v_bus);
}

const char *calib_state_name(calib_state_t s)
{
    switch (s) {
    case CALIB_IDLE:         return "idle";
    case CALIB_RAMP_UP:      return "ramp_up";
    case CALIB_SETTLE_ZERO:  return "settle_zero";
    case CALIB_MEASURE_ZERO: return "measure_zero";
    case CALIB_ROTATE:       return "rotate";
    case CALIB_SETTLE_END:   return "settle_end";
    case CALIB_MEASURE_END:  return "measure_end";
    case CALIB_RAMP_DOWN:    return "ramp_down";
    case CALIB_DONE:         return "done";
    case CALIB_FAILED:       return "failed";
    }
    return "?";
}
