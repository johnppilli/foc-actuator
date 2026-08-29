/*
 * board.h - B-G431B-ESC1 board support + control-loop interface.
 *
 * Everything hardware-specific for the firmware lives behind this header:
 * pin/gain constants, current sensing, PWM, encoder transport, serial, and
 * the control loop entry points. The FOC math itself is in ../../foc and is
 * shared with the desktop tests and simulator.
 *
 * Peripherals added here (OPAMP1-3, ADC1/2, USART2) are configured in code,
 * not in the .ioc, so they do not appear in CubeMX. If you regenerate from
 * the .ioc, re-enable HAL_ADC/OPAMP/UART_MODULE_ENABLED in
 * stm32g4xx_hal_conf.h (or add the peripherals in CubeMX and delete the
 * matching init code here).
 */
#ifndef BOARD_H
#define BOARD_H

#include "main.h"
#include "telemetry.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- board constants: verify against UM2516 before trusting amps -------- */
#define BOARD_VBUS_V            12.0f   /* bench supply; TODO read PA0 divider instead */
#define BOARD_SHUNT_OHM         0.003f
#define BOARD_AMP_GAIN          9.14f   /* external gain network around the internal op-amps */
#define BOARD_ADC_VREF_V        3.3f
#define BOARD_AMPS_PER_COUNT    (BOARD_ADC_VREF_V / 4095.0f / (BOARD_SHUNT_OHM * BOARD_AMP_GAIN))
/* Low-side shunts: positive phase current (into the motor) flows up through
 * the shunt from ground, so the amplifier output drops below its midpoint.
 * Bring-up check: in calib mode, i_d in telemetry must be POSITIVE. If it is
 * negative, flip this sign. */
#define BOARD_CURRENT_SIGN      (-1.0f)
#define BOARD_I_MAX_A           3.0f    /* software overcurrent trip */

/* ---- PWM / loop timing (must match MX_TIM1_Init) ----------------------- */
#define BOARD_PWM_ARR           4250u   /* 170 MHz / (2 * 4250) = 20 kHz centre-aligned */
#define BOARD_PWM_HZ            20000.0f
#define BOARD_LOOP_DT           (1.0f / BOARD_PWM_HZ)
#define BOARD_ADC_TRIGGER_LEAD  20u     /* timer ticks before the counter peak */

/* ---- encoder ------------------------------------------------------------ */
#define BOARD_ENC_CPR           4096u
#define BOARD_ENC_I2C_ADDR      0x36u
#define BOARD_ENC_REG_RAW_ANGLE 0x0Cu
#define BOARD_ENC_DECIMATION    4u      /* read every 4th loop: 5 kHz */
#define BOARD_ENC_TIMEOUT_LOOPS 400u    /* 20 ms without a reading = fault */
#define BOARD_ENC_VEL_CUTOFF_HZ 50.0f

/* ---- telemetry / serial ----------------------------------------------- */
#define BOARD_TELEM_DECIMATION  20u     /* frame every 20 loops: 1 kHz */
#define BOARD_UART_BAUD         921600u

/* ---- motor (edit for yours) -------------------------------------------- */
#define MOTOR_R_OHM             5.6f
#define MOTOR_L_H               1.8e-3f
#define MOTOR_POLE_PAIRS        11u     /* 0 = let calibration estimate it */
#define MOTOR_KT_NM_PER_A       0.079f
#define MOTOR_CURRENT_BW_HZ     500.0f
#define MOTOR_CALIB_VD_V        3.0f    /* alignment voltage: ~0.5 A on a 5.6 ohm motor */

/* handles owned by the CubeMX-generated main.c */
extern TIM_HandleTypeDef htim1;
extern I2C_HandleTypeDef hi2c1;

/* ---- current sensing: board_current.c ---------------------------------- */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
void board_current_init(void);                 /* op-amps + ADC injected channels */
void board_current_start(void);                /* start conversions; JEOS interrupt drives the loop */
void board_current_read_raw(uint16_t raw[3]);  /* last injected results, counts */
void board_current_read(float i_abc[3]);       /* amps, offset-corrected */
void board_current_zero(const uint16_t raw[3]);/* set the zero-current offsets */
void board_current_offsets(float out[3]);

/* ---- PWM: board_pwm.c -------------------------------------------------- */
void board_pwm_init(void);                     /* CH4 ADC trigger, safe idle state */
void board_pwm_start(void);
void board_pwm_set_duty(float a, float b, float c);
void board_pwm_output(bool enable);            /* MOE: false = all gates off */

/* ---- encoder transport: board_encoder.c -------------------------------- */
void     board_encoder_init(void);
void     board_encoder_request(void);          /* kick a non-blocking read (from the loop) */
bool     board_encoder_take(uint16_t *raw);    /* new value since the last take? */
uint32_t board_encoder_errors(void);
void     board_encoder_poll(void);             /* main loop: bus-stuck recovery */

/* ---- serial: board_serial.c ------------------------------------------- */
extern UART_HandleTypeDef huart2;
void board_serial_init(void);
void board_serial_poll(void);                  /* main loop: commands in, telemetry out */
void board_serial_print(const char *s);        /* text, main context only */

/* ---- control loop: foc_loop.c ----------------------------------------- */
typedef enum {
    MODE_IDLE = 0,   /* gates off, motor free */
    MODE_OPENLOOP,   /* rotating voltage vector, no feedback (the original demo) */
    MODE_CALIB,      /* encoder offset / direction / pole-pair calibration */
    MODE_TORQUE,     /* closed-loop current control, iq_ref from serial */
    MODE_HAPTIC,     /* impedance control with a haptic preset */
    MODE_FAULT
} foc_mode_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_OVERCURRENT,
    FAULT_ENCODER,
    FAULT_CALIB,
    FAULT_NOT_CALIBRATED
} foc_fault_t;

extern telem_ring_t g_telem_ring;

void foc_loop_init(void);
void foc_loop_start(void);                     /* arms PWM + ADC; the loop then runs from the ADC ISR */
void foc_loop_isr(void);                       /* one control tick; called from the ADC JEOS callback */
void foc_loop_request_mode(foc_mode_t m);
void foc_loop_set_iq_ref(float a);
void foc_loop_set_id_ref(float a);
void foc_loop_set_haptic_preset(int preset);
void foc_loop_set_openloop(float v_amp, float elec_rad_per_s);
void foc_loop_clear_fault(void);
void foc_loop_status_text(char *buf, size_t n);
foc_mode_t  foc_loop_mode(void);
foc_fault_t foc_loop_fault(void);

#endif /* BOARD_H */
