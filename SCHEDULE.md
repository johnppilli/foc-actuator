# Sprint Schedule — FOC Actuator (July 2026)

**Goal:** reach the **torque-control milestone** (the resume-ready finish line), aiming
**end of July**, with **~2 weeks (by Jul 21)** as the stretch goal.
**Pace assumed:** ~3–4 focused hrs/day.

---

## Current status (as of Jul 7)
- [x] Toolchain + flash chain proven (blinky flashed the board)
- [x] Real CubeMX/HAL project on CubeIDE 1.18, clock @ 170 MHz
- [x] **3-phase PWM driver (TIM1) fully configured + compiles clean**
- [x] Open-loop spin → first motion ✅ (07-09) ← *you are here*
- [ ] Encoder reading + calibration
- [ ] FOC core → torque control (**the milestone**)

---

## Week-by-week

### Week 1 — Jul 7–13:  Spin + Sense
- [x] Write open-loop spin code (start PWM, drive 3 CCRs as sines 120° apart, ramp the angle)
- [x] Wire motor to J7 + DC supply **current-limited to ~0.5 A** (shoot-through safety net)
- [x] **CHECKPOINT: motor physically spins** ✅ (07-09, open-loop, ~0.22A @ 12V)
- [ ] Read the AS5600 magnetic encoder (shaft angle over I2C) — just print it

### Week 2 — Jul 14–20:  FOC core  ← the mountain (high variance)
- [ ] Encoder offset calibration; mechanical → electrical angle (× pole pairs)
- [ ] Clarke + Park transforms
- [ ] Current sensing (shunts, timed to PWM)
- [ ] Current PI loops (hold Id = 0, set Iq)
- [ ] SVPWM output
- [ ] **MILESTONE: torque control** — command a force, motor delivers it

### Week 3 — Jul 21–27:  Polish the milestone
- [ ] Tune PI gains (kill buzzing / oscillation / sluggishness)
- [ ] Make torque control solid + repeatable
- [ ] Record a 30-sec demo video; write the resume bullet
- [ ] (Stretch) start impedance/haptic knob demo

### By Jul 31 — **DONE (resume-ready)**
- [ ] Torque control working + demo video + resume bullet ✅

---

## Reality notes (read these when the calendar stresses you out)
- **"Done" = torque control + demo video + resume bullet.** The impedance/haptic knob is
  *optional polish* — you can move on to the next project once torque control works.
- **The FOC core (Week 2) is the wildcard.** It might click in a week, or a subtle bug
  (encoder offset, current-sense timing, PI tuning) eats an extra week of "why does it just
  buzz." That single week decides *2-weeks vs end-of-month*. Aim stretch, plan end-of-month.
- **Measure by checkpoints, not the calendar.** Next checkpoint = first spin.
- **Don't rush/skip the FOC core** — it IS the resume signal ("motor control from the math
  up"). Move fast, but don't shortcut the part that makes this worth doing.
