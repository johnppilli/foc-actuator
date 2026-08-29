#include "encoder.h"
#include "foc.h"
#include <math.h>

float encoder_lpf_alpha(float cutoff_hz, float dt)
{
    if (cutoff_hz <= 0.0f) return 1.0f;           /* no filtering */
    float tau = 1.0f / (FOC_TWO_PI * cutoff_hz);
    return dt / (dt + tau);
}

void encoder_init(encoder_t *e, uint16_t cpr, uint8_t pole_pairs, float vel_cutoff_hz, float dt)
{
    e->cpr = cpr;
    e->pole_pairs = pole_pairs;
    e->direction = 1;
    e->offset_elec = 0.0f;
    e->vel_alpha = encoder_lpf_alpha(vel_cutoff_hz, dt);
    e->have_prev = false;
    e->prev_raw = 0;
    e->turns = 0;
    e->theta_m = 0.0f;
    e->theta_cont = 0.0f;
    e->velocity = 0.0f;
}

void encoder_set_calibration(encoder_t *e, float offset_elec, int8_t direction)
{
    e->offset_elec = foc_wrap_2pi(offset_elec);
    e->direction = (direction < 0) ? -1 : 1;
}

float encoder_raw_to_rad(uint16_t raw, uint16_t cpr)
{
    return (float)raw * (FOC_TWO_PI / (float)cpr);
}

int32_t encoder_delta_counts(uint16_t now, uint16_t prev, uint16_t cpr)
{
    int32_t d = (int32_t)now - (int32_t)prev;
    int32_t half = (int32_t)cpr / 2;
    if (d >= half)  d -= (int32_t)cpr;
    if (d < -half)  d += (int32_t)cpr;
    return d;
}

void encoder_update(encoder_t *e, uint16_t raw, float dt)
{
    if (raw >= e->cpr) raw = (uint16_t)(raw % e->cpr);
    e->theta_m = encoder_raw_to_rad(raw, e->cpr);

    if (!e->have_prev) {
        e->have_prev = true;
        e->prev_raw = raw;
        e->theta_cont = e->theta_m;
        e->velocity = 0.0f;
        return;
    }

    int32_t d = encoder_delta_counts(raw, e->prev_raw, e->cpr);
    e->prev_raw = raw;

    /* continuous angle accumulates the wrap-corrected step; turns follow it */
    e->theta_cont += (float)d * (FOC_TWO_PI / (float)e->cpr);
    e->turns = (int32_t)floorf(e->theta_cont / FOC_TWO_PI);

    if (dt > 0.0f) {
        float v_raw = (float)d * (FOC_TWO_PI / (float)e->cpr) / dt;
        e->velocity += e->vel_alpha * (v_raw - e->velocity);
    }
}

void encoder_zero_turns(encoder_t *e)
{
    e->theta_cont = e->theta_m;
    e->turns = 0;
}

float encoder_theta_mech(const encoder_t *e)  { return e->theta_m; }
float encoder_theta_cont(const encoder_t *e)  { return e->theta_cont; }
float encoder_velocity(const encoder_t *e)    { return e->velocity; }

float encoder_theta_elec(const encoder_t *e)
{
    return foc_wrap_2pi((float)e->direction * (float)e->pole_pairs * e->theta_m - e->offset_elec);
}

float encoder_theta_elec_extrap(const encoder_t *e, float age_s)
{
    return foc_wrap_2pi(encoder_theta_elec(e) + encoder_velocity_elec(e) * age_s);
}

float encoder_velocity_elec(const encoder_t *e)
{
    return (float)e->direction * (float)e->pole_pairs * e->velocity;
}
