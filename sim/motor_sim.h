/*
 * motor_sim.h - permanent-magnet synchronous motor model for desktop testing.
 *
 * Electrical (rotor frame, Ld = Lq = L, amplitude-invariant transforms):
 *   L di_d/dt = v_d - R i_d + w_e L i_q
 *   L di_q/dt = v_q - R i_q - w_e L i_d - w_e lambda
 *   tau_e     = 1.5 * pp * lambda * i_q            (kt = 1.5 pp lambda)
 * Mechanical:
 *   J dw_m/dt = tau_e - B w_m - tau_coulomb(w_m) - tau_load
 *   d theta_m/dt = w_m,   theta_e = pp * theta_m
 *
 * The inverter is modelled as ideal average voltages (duty * v_bus); no
 * dead time, no ripple. Euler integration with sub-steps.
 *
 * The encoder model reports what an absolute encoder mounted with an
 * arbitrary offset and orientation would read, so calibration can be tested.
 */
#ifndef MOTOR_SIM_H
#define MOTOR_SIM_H

#include "foc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* motor parameters */
    float   R;            /* phase resistance, ohm */
    float   L;            /* phase inductance, H */
    float   lambda;       /* PM flux linkage, V*s/rad (electrical) */
    uint8_t pp;           /* pole pairs */
    float   J;            /* rotor + load inertia, kg m^2 */
    float   B;            /* viscous friction, N m s/rad */
    float   tau_coulomb;  /* coulomb friction, N m */
    /* encoder mounting */
    uint16_t enc_cpr;
    float    enc_offset_mech; /* rad added to the true mechanical angle */
    int8_t   enc_direction;   /* +1 or -1 */
    /* integration */
    int     substeps;
    /* state */
    float   i_d, i_q;
    float   omega_m;
    float   theta_m;      /* continuous mechanical angle, rad */
    /* last outputs */
    float   tau_e;
    foc_abc_t i_abc;
} motor_sim_t;

/* Small gimbal motor with a knob on it; edit for your motor. */
void      motor_sim_init_default(motor_sim_t *m);
/* Advance by dt with phase-to-neutral voltages applied. */
void      motor_sim_step_abc(motor_sim_t *m, foc_abc_t v_phase, float tau_load, float dt);
/* Advance by dt with PWM duties applied (converted to phase voltages). */
void      motor_sim_step_duty(motor_sim_t *m, foc_abc_t duty, float v_bus, float tau_load, float dt);
float     motor_sim_theta_e(const motor_sim_t *m);   /* true electrical angle [0, 2*pi) */
float     motor_sim_kt(const motor_sim_t *m);
uint16_t  motor_sim_encoder_raw(const motor_sim_t *m);
/* The electrical offset a correct calibration should find for this mounting. */
float     motor_sim_expected_offset_elec(const motor_sim_t *m);

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_SIM_H */
