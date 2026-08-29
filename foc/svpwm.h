/*
 * svpwm.h - space-vector PWM: alpha/beta voltage command -> three duty cycles.
 *
 * Duty cycle d for a phase means the high-side switch is on for fraction d
 * of the PWM period, so the pole voltage is d * v_bus. With the motor's
 * neutral floating, the phase-to-neutral voltage is the pole voltage minus
 * the average of the three pole voltages.
 *
 * The linear range of SVPWM is a circle of radius v_bus/sqrt(3) in the
 * alpha/beta plane (phase peak 0.577 v_bus). Plain sine PWM only reaches
 * v_bus/2 (0.5 v_bus), so SVPWM gives 15.5% more usable voltage.
 */
#ifndef SVPWM_H
#define SVPWM_H

#include "foc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum vector magnitude that stays inside the linear region. */
#define SVPWM_MAX_MAG(v_bus) ((v_bus) * FOC_ONE_OVER_SQRT3)

/* Min/max (common-mode) injection SVPWM. Commands beyond the linear region
 * are scaled down to the circle, preserving the vector angle. */
foc_abc_t svpwm(float v_alpha, float v_beta, float v_bus);

/* Classic sector/T1/T2 implementation. Identical output to svpwm() inside the
 * linear region; kept as an independent reference for testing. */
foc_abc_t svpwm_classic(float v_alpha, float v_beta, float v_bus);

/* Plain sine PWM (no common-mode injection). Clamps duties to [0,1]. */
foc_abc_t svpwm_sine(float v_alpha, float v_beta, float v_bus);

/* Sector 1..6 of the alpha/beta vector, 60 degrees each, sector 1 = [0,60). */
int svpwm_sector(float v_alpha, float v_beta);

/* Phase-to-neutral voltages a Y-connected motor sees for the given duties. */
foc_abc_t svpwm_duty_to_phase_voltage(foc_abc_t duty, float v_bus);

#ifdef __cplusplus
}
#endif
#endif /* SVPWM_H */
