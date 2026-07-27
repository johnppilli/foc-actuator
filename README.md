# FOC Actuator — Torque-Controlled Brushless Motor (Robot Joint)

A single brushless motor I control by **force (torque)**, running **FOC firmware I wrote
myself**. It's the unit cell of a humanoid robot joint — demoed as a programmable haptic
knob you can feel push back with your fingers.

**Field:** humanoid robotics → actuators (the bottleneck layer).
**Goal:** reach the torque-control milestone by **mid-August 2026**.

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

