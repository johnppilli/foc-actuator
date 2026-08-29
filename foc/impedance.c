#include "impedance.h"
#include <math.h>

float haptic_spring_torque(const haptic_profile_t *p, float theta)
{
    if (p->k_spring == 0.0f) return 0.0f;
    return -p->k_spring * (theta - p->theta0);
}

float haptic_detent_torque(const haptic_profile_t *p, float theta)
{
    if (p->n_detents == 0 || p->k_detent == 0.0f) return 0.0f;
    float n = (float)p->n_detents;
    float x = theta - p->detent_phase;
    if (p->detent_shape == DETENT_SINE) {
        return -(p->k_detent / n) * sinf(n * x);
    }
    /* sawtooth: distance to the nearest detent, in [-pitch/2, pitch/2) */
    float pitch = FOC_TWO_PI / n;
    float e = x - pitch * floorf(x / pitch + 0.5f);
    return -p->k_detent * e;
}

float haptic_endstop_torque(const haptic_profile_t *p, float theta, float omega)
{
    if (!(p->theta_max > p->theta_min)) return 0.0f;
    if (theta > p->theta_max) {
        return -p->k_endstop * (theta - p->theta_max) - p->b_endstop * omega;
    }
    if (theta < p->theta_min) {
        return -p->k_endstop * (theta - p->theta_min) - p->b_endstop * omega;
    }
    return 0.0f;
}

float haptic_torque(const haptic_profile_t *p, float theta, float omega)
{
    float tau = haptic_spring_torque(p, theta)
              + haptic_detent_torque(p, theta)
              + haptic_endstop_torque(p, theta, omega)
              - p->b_damp * omega;
    return foc_clampf(tau, -p->tau_max, p->tau_max);
}

haptic_profile_t haptic_preset(haptic_preset_t which)
{
    haptic_profile_t p;
    p.k_spring = 0.0f;
    p.theta0 = 0.0f;
    p.b_damp = 0.0005f;
    p.n_detents = 0;
    p.k_detent = 0.0f;
    p.detent_phase = 0.0f;
    p.detent_shape = DETENT_SAWTOOTH;
    p.theta_min = 0.0f;
    p.theta_max = 0.0f;
    p.k_endstop = 0.0f;
    p.b_endstop = 0.0f;
    p.tau_max = 0.10f;   /* ~1.2 A on a 0.08 Nm/A gimbal motor; raise once you trust the current loop */

    switch (which) {
    case HAPTIC_PRESET_FREE:
        break;
    case HAPTIC_PRESET_SPRING:
        p.k_spring = 0.06f;
        p.b_damp = 0.002f;
        break;
    case HAPTIC_PRESET_DAMPED:
        p.b_damp = 0.015f;
        break;
    case HAPTIC_PRESET_DETENTS_12:
        p.n_detents = 12;
        p.k_detent = 0.4f;
        p.b_damp = 0.001f;
        break;
    case HAPTIC_PRESET_DETENTS_SMOOTH:
        p.n_detents = 24;
        p.k_detent = 0.5f;
        p.detent_shape = DETENT_SINE;
        p.b_damp = 0.001f;
        break;
    case HAPTIC_PRESET_ENDSTOPS:
        p.theta_min = -FOC_PI / 2.0f;
        p.theta_max =  FOC_PI / 2.0f;
        p.k_endstop = 1.5f;
        p.b_endstop = 0.02f;
        p.b_damp = 0.001f;
        break;
    default:
        break;
    }
    return p;
}

const char *haptic_preset_name(haptic_preset_t which)
{
    switch (which) {
    case HAPTIC_PRESET_FREE:           return "free";
    case HAPTIC_PRESET_SPRING:         return "spring";
    case HAPTIC_PRESET_DAMPED:         return "damped";
    case HAPTIC_PRESET_DETENTS_12:     return "detents12";
    case HAPTIC_PRESET_DETENTS_SMOOTH: return "detents_smooth";
    case HAPTIC_PRESET_ENDSTOPS:       return "endstops";
    default:                           return "?";
    }
}
