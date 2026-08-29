/*
 * foc_loop.c - the 20 kHz control loop and its mode state machine.
 *
 * foc_loop_isr() runs from the ADC end-of-injected-sequence interrupt, i.e.
 * once per PWM period right after the phase currents were sampled. It:
 *   1. converts the ADC results to amps and trips on overcurrent
 *   2. kicks / consumes the non-blocking encoder read
 *   3. runs the active mode (open loop, calibration, torque, haptic)
 *   4. writes the PWM duties
 *   5. queues a telemetry frame every BOARD_TELEM_DECIMATION ticks
 *
 * Commands from the serial console only set volatile requests; the loop
 * applies them at the top of the next tick, so mode changes are clean.
 *
 * Bring-up order on real hardware:
 *   1. "m open"   : should spin exactly like the original open-loop firmware,
 *                   but now driven from the ISR. Telemetry shows currents.
 *   2. "m calib"  : rotor snaps to alignment, rotates once slowly, stops.
 *                   Check i_d is positive during the hold; if negative flip
 *                   BOARD_CURRENT_SIGN. Result printed with "?".
 *   3. "m torque" + "q 0.2" : shaft should resist / turn with ~0.2 A.
 *   4. "m haptic" + "h 3"   : detents.
 */
#include "board.h"
#include "calib.h"
#include "encoder.h"
#include "foc.h"
#include "impedance.h"
#include "svpwm.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

telem_ring_t g_telem_ring;

/* requests from the main context */
static volatile foc_mode_t s_req_mode = MODE_IDLE;
static volatile bool       s_req_pending = false;
static volatile bool       s_req_clear_fault = false;
static volatile float      s_iq_ref = 0.0f;
static volatile float      s_id_ref = 0.0f;
static volatile int        s_haptic_preset = HAPTIC_PRESET_SPRING;
static volatile float      s_ol_v = 1.8f;      /* 0.3 * (12 V / 2): the original demo amplitude */
static volatile float      s_ol_w = 200.0f;    /* 0.2 rad per ms, as in the original loop */

/* loop state */
static foc_mode_t   s_mode = MODE_IDLE;
static foc_fault_t  s_fault = FAULT_NONE;
static foc_ctrl_t   s_ctrl;
static encoder_t    s_enc;
static calib_t      s_calib;
static haptic_profile_t s_haptic;
static int          s_haptic_loaded = -1;
static bool         s_calibrated = false;
static uint32_t     s_tick = 0;
static uint32_t     s_enc_age = 0;         /* loops since the last encoder sample */
static float        s_ol_theta = 0.0f;
static float        s_haptic_theta0 = 0.0f;
static float        s_i_abc[3];
static float        s_theta_e = 0.0f;
static uint32_t     s_zero_samples = 0;
static float        s_zero_acc[3];

static void set_fault(foc_fault_t f)
{
    s_fault = f;
    s_mode = MODE_FAULT;
    board_pwm_output(false);
    board_pwm_set_duty(0.5f, 0.5f, 0.5f);
    foc_ctrl_reset(&s_ctrl);
}

void foc_loop_init(void)
{
    telem_ring_init(&g_telem_ring);
    foc_ctrl_init(&s_ctrl, MOTOR_R_OHM, MOTOR_L_H, MOTOR_CURRENT_BW_HZ);
    encoder_init(&s_enc, BOARD_ENC_CPR, MOTOR_POLE_PAIRS, BOARD_ENC_VEL_CUTOFF_HZ,
                 BOARD_ENC_DECIMATION * BOARD_LOOP_DT);
    calib_init(&s_calib, MOTOR_CALIB_VD_V, MOTOR_POLE_PAIRS);
    s_haptic = haptic_preset(HAPTIC_PRESET_SPRING);
    s_haptic_loaded = HAPTIC_PRESET_SPRING;
    for (int k = 0; k < 3; k++) s_zero_acc[k] = 0.0f;
    board_pwm_init();
}

void foc_loop_start(void)
{
    board_pwm_start();
    board_current_start();
    /* The ISR is now running; the first 2000 ticks (100 ms) average the ADC
     * offsets with all gates off, see the zeroing branch in foc_loop_isr. */
}

/* ---- requests ----------------------------------------------------------- */

void foc_loop_request_mode(foc_mode_t m) { s_req_mode = m; s_req_pending = true; }
void foc_loop_set_iq_ref(float a)        { s_iq_ref = a; }
void foc_loop_set_id_ref(float a)        { s_id_ref = a; }
void foc_loop_set_haptic_preset(int p)
{
    if (p < 0 || p >= HAPTIC_PRESET_COUNT) return;
    s_haptic_preset = p;
}
void foc_loop_set_openloop(float v, float w) { s_ol_v = v; s_ol_w = w; }
void foc_loop_clear_fault(void)          { s_req_clear_fault = true; }
foc_mode_t  foc_loop_mode(void)          { return s_mode; }
foc_fault_t foc_loop_fault(void)         { return s_fault; }

static const char *mode_name(foc_mode_t m)
{
    switch (m) {
    case MODE_IDLE: return "idle";
    case MODE_OPENLOOP: return "open";
    case MODE_CALIB: return "calib";
    case MODE_TORQUE: return "torque";
    case MODE_HAPTIC: return "haptic";
    case MODE_FAULT: return "fault";
    }
    return "?";
}

static const char *fault_name(foc_fault_t f)
{
    switch (f) {
    case FAULT_NONE: return "none";
    case FAULT_OVERCURRENT: return "overcurrent";
    case FAULT_ENCODER: return "encoder";
    case FAULT_CALIB: return "calib";
    case FAULT_NOT_CALIBRATED: return "not-calibrated";
    }
    return "?";
}

/* newlib-nano's printf has no %f unless -u _printf_float is linked; format
 * to three decimals by hand instead. */
static const char *f3(char *out, size_t n, float x)
{
    long m = (long)(x * 1000.0f + (x >= 0.0f ? 0.5f : -0.5f));
    snprintf(out, n, "%s%ld.%03ld", m < 0 ? "-" : "", labs(m) / 1000, labs(m) % 1000);
    return out;
}

void foc_loop_status_text(char *buf, size_t n)
{
    float off[3];
    char a[16], b[16], c[16], d[16], e[16];
    board_current_offsets(off);
    int w = snprintf(buf, n,
             "\nmode=%s fault=%s tick=%lu calibrated=%d pp=%u dir=%d offset=%s calib=%s%s%s\n",
             mode_name(s_mode), fault_name(s_fault), (unsigned long)s_tick, (int)s_calibrated,
             (unsigned)s_enc.pole_pairs, (int)s_enc.direction, f3(a, sizeof a, s_enc.offset_elec),
             calib_state_name(s_calib.state),
             s_calib.fail_reason ? " reason=" : "", s_calib.fail_reason ? s_calib.fail_reason : "");
    if (w < 0 || (size_t)w >= n) return;
    snprintf(buf + w, n - (size_t)w,
             "i=[%s %s %s] zero=[%ld %ld %ld] enc_err=%lu theta_m=%s w=%s\n",
             f3(a, sizeof a, s_i_abc[0]), f3(b, sizeof b, s_i_abc[1]), f3(c, sizeof c, s_i_abc[2]),
             (long)off[0], (long)off[1], (long)off[2],
             (unsigned long)board_encoder_errors(),
             f3(d, sizeof d, encoder_theta_mech(&s_enc)), f3(e, sizeof e, encoder_velocity(&s_enc)));
}

/* ---- the tick ------------------------------------------------------------ */

static void apply_requests(void)
{
    if (s_req_clear_fault) {
        s_req_clear_fault = false;
        if (s_mode == MODE_FAULT) {
            s_fault = FAULT_NONE;
            s_mode = MODE_IDLE;
        }
    }
    if (!s_req_pending) return;
    s_req_pending = false;
    foc_mode_t m = s_req_mode;
    if (s_mode == MODE_FAULT) return;              /* clear the fault first */
    if (m == s_mode) return;

    foc_ctrl_reset(&s_ctrl);
    switch (m) {
    case MODE_IDLE:
        board_pwm_output(false);
        break;
    case MODE_OPENLOOP:
        s_ol_theta = 0.0f;
        board_pwm_output(true);
        break;
    case MODE_CALIB:
        calib_init(&s_calib, MOTOR_CALIB_VD_V, MOTOR_POLE_PAIRS);
        calib_start(&s_calib);
        board_pwm_output(true);
        break;
    case MODE_TORQUE:
    case MODE_HAPTIC:
        if (!s_calibrated) { set_fault(FAULT_NOT_CALIBRATED); return; }
        s_haptic_theta0 = encoder_theta_cont(&s_enc);
        board_pwm_output(true);
        break;
    default:
        return;
    }
    s_mode = m;
}

void foc_loop_isr(void)
{
    s_tick++;

    /* 1. currents. First 100 ms: average the zero-current ADC offsets. */
    if (s_zero_samples < 2000u) {
        uint16_t raw[3];
        board_current_read_raw(raw);
        for (int k = 0; k < 3; k++) s_zero_acc[k] += (float)raw[k];
        if (++s_zero_samples == 2000u) {
            uint16_t avg[3];
            for (int k = 0; k < 3; k++) avg[k] = (uint16_t)(s_zero_acc[k] / 2000.0f + 0.5f);
            board_current_zero(avg);
        }
        board_pwm_set_duty(0.5f, 0.5f, 0.5f);
        return;
    }
    board_current_read(s_i_abc);
    for (int k = 0; k < 3; k++) {
        if (fabsf(s_i_abc[k]) > BOARD_I_MAX_A && s_mode != MODE_FAULT) {
            set_fault(FAULT_OVERCURRENT);
        }
    }

    /* 2. encoder */
    if ((s_tick % BOARD_ENC_DECIMATION) == 0u) board_encoder_request();
    uint16_t raw;
    if (board_encoder_take(&raw)) {
        encoder_update(&s_enc, raw, (float)(s_enc_age + 1u) * BOARD_LOOP_DT);
        s_enc_age = 0;
    } else {
        s_enc_age++;
    }
    bool enc_needed = (s_mode == MODE_CALIB || s_mode == MODE_TORQUE || s_mode == MODE_HAPTIC);
    if (enc_needed && s_enc_age > BOARD_ENC_TIMEOUT_LOOPS) set_fault(FAULT_ENCODER);

    apply_requests();

    /* 3. mode */
    foc_abc_t duty = { 0.5f, 0.5f, 0.5f };
    foc_dq_t i_ref = { s_id_ref, s_iq_ref };
    const float dt = BOARD_LOOP_DT;
    const float age = (float)s_enc_age * dt;

    switch (s_mode) {
    case MODE_OPENLOOP: {
        s_ol_theta = foc_wrap_2pi(s_ol_theta + s_ol_w * dt);
        float s, c;
        foc_sincos(s_ol_theta, &s, &c);
        foc_dq_t v = { s_ol_v, 0.0f };
        foc_ab_t ab = foc_inv_park(v, s, c);
        duty = svpwm(ab.alpha, ab.beta, BOARD_VBUS_V);
        s_theta_e = s_ol_theta;
        /* report what the current loop would see, for the sign check */
        s_ctrl.i_dq = foc_park(foc_clarke(s_i_abc[0], s_i_abc[1]), s, c);
        break;
    }
    case MODE_CALIB: {
        calib_update(&s_calib, encoder_theta_mech(&s_enc), dt);
        duty = calib_duty(&s_calib, BOARD_VBUS_V);
        s_theta_e = s_calib.theta_e_cmd;
        float s, c;
        foc_sincos(s_theta_e, &s, &c);
        s_ctrl.i_dq = foc_park(foc_clarke(s_i_abc[0], s_i_abc[1]), s, c);
        if (s_calib.state == CALIB_DONE) {
            encoder_init(&s_enc, BOARD_ENC_CPR, s_calib.pole_pairs, BOARD_ENC_VEL_CUTOFF_HZ,
                         BOARD_ENC_DECIMATION * dt);
            encoder_set_calibration(&s_enc, s_calib.offset_elec, s_calib.direction);
            s_calibrated = true;
            s_mode = MODE_IDLE;
            board_pwm_output(false);
        } else if (s_calib.state == CALIB_FAILED) {
            set_fault(FAULT_CALIB);
        }
        break;
    }
    case MODE_TORQUE:
        s_theta_e = encoder_theta_elec_extrap(&s_enc, age);
        duty = foc_ctrl_step(&s_ctrl, s_i_abc[0], s_i_abc[1], s_theta_e, i_ref, BOARD_VBUS_V, dt);
        break;
    case MODE_HAPTIC: {
        if (s_haptic_loaded != s_haptic_preset) {
            s_haptic = haptic_preset((haptic_preset_t)s_haptic_preset);
            s_haptic_loaded = s_haptic_preset;
            s_haptic_theta0 = encoder_theta_cont(&s_enc);
        }
        float theta = encoder_theta_cont(&s_enc) - s_haptic_theta0;
        float tau = haptic_torque(&s_haptic, theta, encoder_velocity(&s_enc));
        /* positive iq turns the rotor in the +theta_e direction, which is
         * encoder direction * +theta_m: map the torque accordingly */
        i_ref.d = 0.0f;
        i_ref.q = (float)s_enc.direction * tau / MOTOR_KT_NM_PER_A;
        s_iq_ref = i_ref.q;
        s_theta_e = encoder_theta_elec_extrap(&s_enc, age);
        duty = foc_ctrl_step(&s_ctrl, s_i_abc[0], s_i_abc[1], s_theta_e, i_ref, BOARD_VBUS_V, dt);
        break;
    }
    case MODE_IDLE:
    case MODE_FAULT:
    default:
        break;
    }

    /* 4. output */
    board_pwm_set_duty(duty.a, duty.b, duty.c);

    /* 5. telemetry */
    if ((s_tick % BOARD_TELEM_DECIMATION) == 0u) {
        telem_frame_t f;
        f.tick = s_tick;
        f.theta_e = s_theta_e;
        f.theta_m = encoder_theta_mech(&s_enc);
        f.omega_m = encoder_velocity(&s_enc);
        f.i_d = s_ctrl.i_dq.d;
        f.i_q = s_ctrl.i_dq.q;
        f.i_q_ref = s_iq_ref;
        f.v_d = s_ctrl.v_dq.d;
        f.v_q = s_ctrl.v_dq.q;
        f.v_bus = BOARD_VBUS_V;
        f.mode = (uint8_t)s_mode;
        f.fault = (uint8_t)s_fault;
        telem_ring_push_frame(&g_telem_ring, &f);
    }
}
