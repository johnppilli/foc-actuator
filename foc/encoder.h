/*
 * encoder.h - absolute magnetic encoder processing (AS5600 and similar).
 *
 * Turns a raw count (0..cpr-1) into:
 *   - wrapped mechanical angle in [0, 2*pi)
 *   - continuous multi-turn mechanical angle
 *   - low-pass-filtered mechanical velocity
 *   - electrical angle:  theta_e = wrap(direction * pole_pairs * theta_m - offset_e)
 *
 * direction and offset_e come from the calibration routine in calib.h.
 *
 * Velocity quality note: a 12-bit encoder has 1.5 mrad steps. Differentiating
 * one step at 5 kHz gives a 7.7 rad/s quantisation spike, so the velocity is
 * low-pass filtered (default 50 Hz) and callers should keep damping gains
 * modest. A PLL-style observer would be the next upgrade.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_AS5600_CPR 4096u

typedef struct {
    /* configuration */
    uint16_t cpr;
    uint8_t  pole_pairs;
    int8_t   direction;      /* +1 or -1, from calibration */
    float    offset_elec;    /* electrical radians, from calibration */
    float    vel_alpha;      /* first-order LPF coefficient for velocity */
    /* state */
    bool     have_prev;
    uint16_t prev_raw;
    int32_t  turns;          /* completed mechanical turns since init (signed) */
    float    theta_m;        /* wrapped mechanical angle [0, 2*pi) */
    float    theta_cont;     /* continuous multi-turn mechanical angle */
    float    velocity;       /* filtered mechanical velocity, rad/s */
} encoder_t;

/* dt is the interval between encoder_update() calls; vel_cutoff_hz sets the
 * velocity low-pass corner. */
void  encoder_init(encoder_t *e, uint16_t cpr, uint8_t pole_pairs, float vel_cutoff_hz, float dt);
void  encoder_set_calibration(encoder_t *e, float offset_elec, int8_t direction);
/* Feed a new raw reading. dt is the time since the previous reading. */
void  encoder_update(encoder_t *e, uint16_t raw, float dt);
/* Reset multi-turn tracking so the current position is turn 0. */
void  encoder_zero_turns(encoder_t *e);

float encoder_theta_mech(const encoder_t *e);      /* [0, 2*pi) */
float encoder_theta_cont(const encoder_t *e);      /* continuous, multi-turn */
float encoder_theta_elec(const encoder_t *e);      /* [0, 2*pi) */
/* Electrical angle extrapolated forward by age_s using the velocity estimate.
 * Useful when the control loop runs faster than the encoder is read. */
float encoder_theta_elec_extrap(const encoder_t *e, float age_s);
float encoder_velocity(const encoder_t *e);        /* mechanical rad/s */
float encoder_velocity_elec(const encoder_t *e);   /* electrical rad/s (sign follows theta_e) */

/* Pure helpers. */
float   encoder_raw_to_rad(uint16_t raw, uint16_t cpr);
/* Shortest signed step from prev to now on a cpr-count circle. */
int32_t encoder_delta_counts(uint16_t now, uint16_t prev, uint16_t cpr);
/* LPF coefficient for a first-order filter with the given corner and period. */
float   encoder_lpf_alpha(float cutoff_hz, float dt);

#ifdef __cplusplus
}
#endif
#endif /* ENCODER_H */
