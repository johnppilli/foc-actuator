/*
 * foc.h - hardware-independent FOC math.
 *
 * Conventions used throughout this library:
 *   - Angles are in radians. Electrical angle theta_e is in [0, 2*pi).
 *   - Clarke transform is amplitude-invariant: a balanced three-phase set with
 *     peak amplitude I gives an alpha/beta vector of magnitude I.
 *   - Park: d = alpha*cos + beta*sin, q = -alpha*sin + beta*cos.
 *     With the rotor flux on the d axis, iq is the torque-producing current.
 *   - Currents in amps, voltages in volts, time in seconds.
 *
 * Everything here is plain C99 with float math only (no double, no malloc,
 * no printf) so it builds unchanged for the Cortex-M4F and for the desktop
 * test/simulation harness.
 */
#ifndef FOC_H
#define FOC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_PI            3.14159265f
#define FOC_TWO_PI        6.28318531f
#define FOC_SQRT3         1.73205081f
#define FOC_ONE_OVER_SQRT3 0.57735027f

/* Two-phase stationary frame (alpha/beta). */
typedef struct { float alpha, beta; } foc_ab_t;
/* Rotor frame (direct/quadrature). */
typedef struct { float d, q; } foc_dq_t;
/* Three-phase quantities: currents, voltages, or PWM duty cycles. */
typedef struct { float a, b, c; } foc_abc_t;

/* ---- angle helpers ------------------------------------------------------ */

/* Wrap to [0, 2*pi). */
float foc_wrap_2pi(float theta);
/* Wrap to [-pi, pi). */
float foc_wrap_pi(float theta);
/* Clamp x to [lo, hi]. */
float foc_clampf(float x, float lo, float hi);
/* sin and cos of theta in one call. The firmware can later swap this for a
 * lookup table or the G4's CORDIC peripheral without touching callers. */
void foc_sincos(float theta, float *s, float *c);

/* ---- transforms --------------------------------------------------------- */

/* Clarke from two phase currents, assuming ia + ib + ic == 0. */
foc_ab_t  foc_clarke(float ia, float ib);
/* Clarke from all three phase currents (uses the extra measurement; more
 * noise-tolerant than foc_clarke when all three shunts are readable). */
foc_ab_t  foc_clarke3(float ia, float ib, float ic);
foc_abc_t foc_inv_clarke(foc_ab_t ab);
/* Park / inverse Park take sin/cos of theta_e so callers compute them once. */
foc_dq_t  foc_park(foc_ab_t ab, float sin_th, float cos_th);
foc_ab_t  foc_inv_park(foc_dq_t dq, float sin_th, float cos_th);

/* ---- PI controller ------------------------------------------------------ */

typedef struct {
    float kp;
    float ki;         /* per second: output += ki * error * dt */
    float out_min;
    float out_max;
    float integral;   /* integrator state in OUTPUT units (ki already applied) */
} foc_pi_t;

void  foc_pi_init(foc_pi_t *pi, float kp, float ki, float out_min, float out_max);
void  foc_pi_reset(foc_pi_t *pi);
/* One update. Output is clamped to [out_min, out_max]; the integrator is
 * clamped to the same range and frozen while the output is saturated in the
 * direction of the error (conditional integration), so it never winds up. */
float foc_pi_update(foc_pi_t *pi, float error, float dt);
/* Pole-placement tuning for a current loop on an R-L plant:
 *   kp = L * wc,  ki = R * wc,  wc = 2*pi*bandwidth_hz.
 * The PI zero cancels the electrical pole and the closed loop is a first-order
 * response with the requested bandwidth. Start around 300-1000 Hz for a
 * 20 kHz loop. */
void  foc_pi_tune_current(foc_pi_t *pi, float R_ohm, float L_henry, float bandwidth_hz);

/* ---- dq current controller --------------------------------------------- */

typedef struct {
    foc_pi_t pi_d;
    foc_pi_t pi_q;
    /* Fraction of the SVPWM linear limit (v_bus/sqrt(3)) the controller may
     * use. < 1 leaves headroom so the PI never asks for duties outside [0,1]. */
    float modulation_limit;
    /* Last-step values, kept for telemetry and the simulator. */
    foc_ab_t i_ab;
    foc_dq_t i_dq;
    foc_dq_t v_dq;
    foc_ab_t v_ab;
    foc_abc_t duty;
} foc_ctrl_t;

/* Initialise both PI loops from motor R/L and a target bandwidth. */
void foc_ctrl_init(foc_ctrl_t *c, float R_ohm, float L_henry, float bandwidth_hz);
void foc_ctrl_reset(foc_ctrl_t *c);
/* One control tick. Inputs: two measured phase currents, electrical angle,
 * dq current references, bus voltage, loop period. Returns the three PWM
 * duty cycles in [0,1]. The d loop has priority on the voltage budget: the q
 * loop's limits are shrunk each tick so |v_dq| stays inside the circle. */
foc_abc_t foc_ctrl_step(foc_ctrl_t *c, float ia, float ib, float theta_e,
                        foc_dq_t i_ref, float v_bus, float dt);

#ifdef __cplusplus
}
#endif
#endif /* FOC_H */
