# FOC Actuator — Torque-Controlled Brushless Motor (Robot Joint)

A single brushless motor I control by **force (torque)**, running **FOC firmware I wrote
myself**. It's the unit cell of a humanoid robot joint — demoed as a programmable haptic
knob you can feel push back with your fingers.

**Field:** humanoid robotics → actuators (the bottleneck layer).
**Goal:** done / resume-ready by **mid-August 2026**.

---

## The one-line idea

**FOC = the trick that lets code control exactly how *hard* a motor pushes.**

A servo only knows "go to this angle." This knows *force* — push soft, push hard, be
springy, push back when touched. Force is what robots actually need (to soften a footstep,
hold something gently, not hurt people).

## The 3 parts (+ 1 helper)

- **Motor** = the muscle (brushless gimbal motor)
- **Encoder** = the eyes (reads the shaft angle)
- **Controller** = the brain (microcontroller running *my* FOC code)
- **Driver board** = the arm (delivers the actual power to the motor)

Loop runs ~20,000×/sec: encoder reads angle → FOC decides how much push → driver delivers
it → motor pushes → repeat.

## Two rules that keep it resume-grade

1. **Write the FOC control code myself.** Buy the hardware (normal), but do NOT just call
   the SimpleFOC library. Read SimpleFOC as a *reference* only. The signal is "I built
   motor control from the math up."
2. **Don't fab the power board from scratch this summer.** Use a known-good dev board so a
   broken motor means a *code* bug, not a hardware mystery. Building my own inverter board
   is the **fall** follow-on project. The "from scratch" that earns credit is the
   *algorithm*, and that's 100% mine.

## Parts (to finalize before ordering)

- Driver board: **B-G431B-ESC1** (~$25) — has STM32 + driver + current sensing on one board
- **Gimbal motor** (~30–40mm BLDC)
- **Magnetic encoder** (AS5600 simple / AS5047 better)
- Power supply
- (optional) knob cap + small screen for the haptic demo

## Build arc (each stage = a working checkpoint)

1. **Spin open-loop** — motor turns smoothly, no feedback yet. First proof of life.
2. **Add encoder** — read shaft angle; calibrate the offset.
3. **FOC core** — Clarke/Park transforms + current PI loops. The heart.
4. **Torque control works** ← THE milestone. Command a force, motor delivers it.
5. **Impedance control** — soft / stiff / springy / clicky in software. The haptic demo.
6. **(Stretch)** position/velocity loop, or bolt it to a 1-link limb.

## Timeline → mid-August (~8 weeks)

- Wk 1–2: order parts, toolchain, open-loop spin
- Wk 3–4: encoder + calibration
- Wk 5–6: FOC core + current loop → **torque control** (be done here even if nothing else)
- Wk 7: impedance control / haptic demo
- Wk 8: polish, demo video, writeup, resume bullet

Also: slot in the **one remaining macro-keypad debug session** early so that project gets
a clean ending too.

## Reality check

Ordering/wiring parts is the easy 5%. **Writing the FOC is the real mountain** — the math,
current-sense timing, encoder calibration, PI tuning, and debugging a motor that just
buzzes for unknown reasons. That difficulty *is* the value. (Same as "I just wrote some
Verilog" undersells the RISC-V CPU.)

## Resume bullet (draft)

> **Field-Oriented Control BLDC Actuator** — Implemented sensored FOC from scratch on
> STM32 (Clarke/Park transforms, current PI loops, SVPWM) with magnetic encoder feedback;
> achieved closed-loop torque and impedance control. Demonstrated as a programmable haptic
> actuator — the core building block of a humanoid joint.
