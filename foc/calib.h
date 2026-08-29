/*
 * calib.h - encoder offset / direction / pole-pair calibration.
 *
 * Runs open loop: it holds a fixed voltage on the d axis so the rotor locks
 * to the commanded electrical angle, then slowly rotates that angle through
 * one full electrical revolution while watching the encoder.
 *
 *   theta_e = wrap(direction * pole_pairs * theta_m - offset_elec)
 *
 *   - offset_elec: where the encoder reads when the rotor sits at theta_e = 0
 *   - direction:   sign of d(theta_e)/d(theta_m)
 *   - pole pairs:  2*pi / |mechanical angle moved per electrical revolution|
 *
 * Call calib_update() every control tick with the current mechanical angle;
 * apply the returned theta_e_cmd / v_d_cmd to the motor (calib_duty() does
 * the inverse Park + SVPWM for you). When state == CALIB_DONE, copy the
 * results into the encoder with encoder_set_calibration().
 */
#ifndef CALIB_H
#define CALIB_H

#include "foc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CALIB_IDLE = 0,
    CALIB_RAMP_UP,       /* ramp v_d from 0 to v_align at theta_e = 0 */
    CALIB_SETTLE_ZERO,   /* let the rotor lock */
    CALIB_MEASURE_ZERO,  /* average theta_m */
    CALIB_ROTATE,        /* theta_e 0 -> 2*pi, accumulate mechanical motion */
    CALIB_SETTLE_END,
    CALIB_MEASURE_END,
    CALIB_RAMP_DOWN,     /* v_d back to 0 */
    CALIB_DONE,
    CALIB_FAILED
} calib_state_t;

typedef struct {
    /* configuration */
    float   v_align;         /* volts on the d axis while locked */
    float   ramp_s;          /* v_d ramp time */
    float   settle_s;        /* wait after each move */
    float   measure_s;       /* averaging window */
    float   rotate_s;        /* time for one electrical revolution */
    uint8_t pole_pairs_hint; /* 0 = trust the estimate; else must match it */

    /* results (valid when state == CALIB_DONE) */
    float   offset_elec;
    int8_t  direction;
    uint8_t pole_pairs;      /* estimated, or the hint when it agreed */
    float   mech_per_elec_rev; /* signed mechanical rad moved per electrical rev */
    float   residual;        /* |mech_per_elec_rev| - 2*pi/pole_pairs, rad */
    const char *fail_reason; /* set when state == CALIB_FAILED */

    /* commands for the caller (updated each calib_update) */
    calib_state_t state;
    float   theta_e_cmd;
    float   v_d_cmd;

    /* internal */
    float    t;
    float    sum_sin, sum_cos;
    uint32_t n_avg;
    float    theta_m_zero;
    float    theta_m_prev;
    float    mech_accum;
    bool     have_prev;
} calib_t;

void  calib_init(calib_t *c, float v_align, uint8_t pole_pairs_hint);
void  calib_start(calib_t *c);
void  calib_update(calib_t *c, float theta_m, float dt);
bool  calib_running(const calib_t *c);
/* Duties to apply for the current command (theta_e_cmd, v_d_cmd). */
foc_abc_t calib_duty(const calib_t *c, float v_bus);
const char *calib_state_name(calib_state_t s);

#ifdef __cplusplus
}
#endif
#endif /* CALIB_H */
