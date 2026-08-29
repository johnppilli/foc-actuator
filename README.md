# FOC Actuator

Field-oriented control firmware for a brushless gimbal motor, written from scratch on an STM32G431. The goal is torque control: command a force and have the motor deliver it. That is the building block of a robot joint, and the demo target is a haptic knob that pushes back when you turn it.

## Hardware

- B-G431B-ESC1 driver board (STM32G431, gate driver, and current shunts on one board)
- Small brushless gimbal motor
- AS5600 magnetic encoder over I2C
- Bench power supply

- `foc.c`: Clarke and Park transforms, their inverses, and a PI controller, with a small check in `main()` that runs them on fixed inputs. Build with `gcc foc.c -lm -o foc`.
- `firmware/`: STM32CubeIDE project for the G431. It currently does open-loop commutation on TIM1 (three complementary PWM pairs) and reads the AS5600 raw angle on I2C1.
- `ROADMAP.md`: the plan and where it stands.

## Status

Open-loop spin and encoder reads work on hardware. Next is aligning the encoder to the electrical angle, then closing the current loops for real torque control, then impedance control for the haptic demo.
