# Roadmap — FOC Actuator (Torque-Controlled BLDC, Robot Joint)

**Status:** 🟡 In progress — target completion **mid-August 2026**
**Goal:** Implement sensored Field-Oriented Control (FOC) firmware *from scratch* on an STM32 to
torque-control a brushless gimbal motor — the unit cell of a humanoid robot joint — demoed as a
programmable haptic knob.

---

## Phase 0 — Learn the fundamentals (in progress)
- [x] Motor basics — BLDC, the 90° torque rule, pole pairs (mechanical vs electrical angle)
- [ ] Electronics — PWM, 3-phase inverter / half-bridge
- [ ] Control — open vs closed loop, PI controllers
- [ ] Core FOC idea — Clarke/Park transforms, the d-q frame, Id=0 / Iq=torque

## Phase 1 — Hardware & toolchain
- [ ] Finalize + order parts (B-G431B-ESC1 driver, gimbal motor, magnetic encoder, PSU)
- [ ] Set up STM32 toolchain, flash a blinky / serial "hello"
- [ ] **Checkpoint: open-loop spin** — motor turns smoothly, no feedback yet (first proof of life)

## Phase 2 — Sensing
- [ ] Read magnetic encoder (shaft angle) over SPI/I2C
- [ ] Encoder offset calibration; convert mechanical → electrical angle (× pole pairs)
- [ ] Current sensing via shunts, timed to the PWM
- [ ] **Checkpoint: angle + currents reading reliably**

## Phase 3 — FOC core  ← the real mountain
- [ ] Clarke + Park transforms
- [ ] Current PI loops (Id, Iq); hold Id = 0
- [ ] SVPWM output to the 3 phases
- [ ] Run the control loop at ~20 kHz off a timer interrupt
- [ ] **MILESTONE: torque control** — command a force, motor delivers it. (Done-enough here.)

## Phase 4 — Impedance / haptics
- [ ] Software-defined stiffness/damping → soft / stiff / springy / clicky knob
- [ ] **Checkpoint: programmable haptic demo** you can feel push back

## Phase 5 — Polish & ship
- [ ] Demo video, writeup, photos

## Stretch
- [ ] Position / velocity outer loop
- [ ] Bolt to a 1-link limb
- [ ] (Fall follow-on) design my own inverter / power board from scratch
