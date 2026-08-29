# Roadmap

Sensored field-oriented control on an STM32, written from scratch, to torque-control a brushless gimbal motor. Demo target: a programmable haptic knob.

Status: in progress. Target: mid-August 2026.

## Phase 0: fundamentals
- [x] Motor basics: BLDC, the 90 degree torque rule, pole pairs, mechanical vs electrical angle
- [ ] Electronics: PWM, three-phase inverter, half-bridges
- [ ] Control: open vs closed loop, PI controllers
- [ ] FOC: Clarke/Park transforms, the d-q frame, Id held at 0 with Iq as torque

## Phase 1: hardware and toolchain
- [ ] Finalize and order parts (B-G431B-ESC1, gimbal motor, magnetic encoder, PSU)
- [ ] Set up the STM32 toolchain, flash a blinky and a serial hello
- [ ] Checkpoint: open-loop spin

## Phase 2: sensing
- [ ] Read the magnetic encoder over SPI or I2C
- [ ] Encoder offset calibration; mechanical to electrical angle (times pole pairs)
- [ ] Current sensing through the shunts, sampled in sync with the PWM
- [ ] Checkpoint: angle and currents read reliably

## Phase 3: FOC core
- [ ] Clarke and Park transforms
- [ ] Current PI loops for Id and Iq, holding Id at 0
- [ ] SVPWM output to the three phases
- [ ] Run the control loop at about 20 kHz from a timer interrupt
- [ ] Milestone: torque control. Command a force, motor delivers it.

## Phase 4: impedance and haptics
- [ ] Software-defined stiffness and damping: soft, stiff, springy, detented
- [ ] Checkpoint: haptic knob demo

## Phase 5: wrap up
- [ ] Demo video, writeup, photos

## Stretch
- [ ] Position/velocity outer loop
- [ ] Mount it on a one-link arm
- [ ] Design my own inverter/power board
