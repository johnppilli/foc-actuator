/*
 * impedance.h - software-defined feel: torque as a function of position and
 * velocity.
 *
 *   tau = spring + detents + end stops - damping, clamped to +-tau_max
 *
 * Feed the result to the current loop as iq_ref = tau / kt.
 *
 * Detent shapes (both parameterised by the stiffness at the detent centre):
 *   SAWTOOTH: linear spring to the nearest detent; torque jumps sign at the
 *             cell boundary. Feels like a mechanical click.
 *   SINE:     tau = -(k/N) sin(N (theta - phase)). Smooth, no discontinuity.
 */
#ifndef IMPEDANCE_H
#define IMPEDANCE_H

#include "foc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { DETENT_SAWTOOTH = 0, DETENT_SINE = 1 } detent_shape_t;

typedef struct {
    /* spring toward theta0 */
    float k_spring;          /* Nm/rad, 0 = off */
    float theta0;            /* rad */
    /* viscous damping, always active */
    float b_damp;            /* Nm/(rad/s) */
    /* detents */
    uint16_t n_detents;      /* per mechanical revolution, 0 = off */
    float k_detent;          /* Nm/rad at the detent centre */
    float detent_phase;      /* rad, position of one detent */
    detent_shape_t detent_shape;
    /* end stops (active when theta_max > theta_min) */
    float theta_min, theta_max;
    float k_endstop;         /* Nm/rad inside the wall */
    float b_endstop;         /* extra damping inside the wall */
    /* output clamp */
    float tau_max;           /* Nm */
} haptic_profile_t;

typedef enum {
    HAPTIC_PRESET_FREE = 0,      /* almost nothing, just a touch of damping */
    HAPTIC_PRESET_SPRING,        /* returns to centre */
    HAPTIC_PRESET_DAMPED,        /* heavy, viscous */
    HAPTIC_PRESET_DETENTS_12,    /* 12 mechanical clicks per turn */
    HAPTIC_PRESET_DETENTS_SMOOTH,/* 24 soft bumps per turn */
    HAPTIC_PRESET_ENDSTOPS,      /* free between +-90 degrees, walls beyond */
    HAPTIC_PRESET_COUNT
} haptic_preset_t;

/* Torque for a mechanical angle (continuous, rad) and velocity (rad/s). */
float haptic_torque(const haptic_profile_t *p, float theta, float omega);
/* Individual terms, exposed for plotting and tests. */
float haptic_spring_torque(const haptic_profile_t *p, float theta);
float haptic_detent_torque(const haptic_profile_t *p, float theta);
float haptic_endstop_torque(const haptic_profile_t *p, float theta, float omega);

haptic_profile_t haptic_preset(haptic_preset_t which);
const char *haptic_preset_name(haptic_preset_t which);

#ifdef __cplusplus
}
#endif
#endif /* IMPEDANCE_H */
