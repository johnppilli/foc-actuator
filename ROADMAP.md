# Roadmap

Sensored field-oriented control on an STM32, written from scratch, to torque-control a brushless gimbal motor. Demo target: a programmable haptic knob.

Status: in progress. Target: mid-August 2026.

Legend: `[x]` done and verified on hardware; `(sim)` written, unit-tested and validated on the simulated motor, waiting on hardware.

## Phase 0: fundamentals
- [x] Motor basics: BLDC, the 90 degree torque rule, pole pairs, mechanical vs electrical angle
- [ ] Electronics: PWM, three-phase inverter, half-bridges
- [ ] Control: open vs closed loop, PI controllers
- [ ] FOC: Clarke/Park transforms, the d-q frame, Id held at 0 with Iq as torque

## Phase 1: hardware and toolchain
- [x] Finalize and order parts (B-G431B-ESC1, gimbal motor, magnetic encoder, PSU)
- [x] Set up the STM32 toolchain, flash a blinky and a serial hello
- [x] Checkpoint: open-loop spin

## Phase 2: sensing
- [x] Read the magnetic encoder over SPI or I2C
- [ ] Encoder offset calibration; mechanical to electrical angle (times pole pairs) (sim: `foc/calib.c`, `foc/encoder.c`)
- [ ] Current sensing through the shunts, sampled in sync with the PWM (code: `firmware/Core/Src/board_current.c`)
- [ ] Checkpoint: angle and currents read reliably

## Phase 3: FOC core
- [x] Clarke and Park transforms (`foc/foc.c`, round-trip and rotating-vector tests)
- [ ] Current PI loops for Id and Iq, holding Id at 0 (sim: `foc_ctrl_step`, step response in `tests/test_sim.c`)
- [ ] SVPWM output to the three phases (sim: `foc/svpwm.c`, cross-checked against a second implementation)
- [ ] Run the control loop at about 20 kHz from a timer interrupt (code: ADC-triggered ISR in `foc_loop.c`)
- [ ] Milestone: torque control. Command a force, motor delivers it.

## Phase 4: impedance and haptics
- [ ] Software-defined stiffness and damping: soft, stiff, springy, detented (sim: `foc/impedance.c`, presets plotted in `docs/haptic-presets.png`)
- [ ] Checkpoint: haptic knob demo

## Phase 5: wrap up
- [ ] Demo video, writeup, photos

## Stretch
- [ ] Position/velocity outer loop
- [ ] Mount it on a one-link arm
- [ ] Design my own inverter/power board
- [ ] Read bus voltage from the PA0 divider instead of a constant
- [ ] Hardware overcurrent: wire the board's comparator to TIM1 BKIN
- [ ] Velocity observer (PLL) instead of filtered finite differences
