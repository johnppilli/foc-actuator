# Learning Checklist — what to understand before/while building

> Don't master it all before starting. Get **Tier 1** solid (concept, not mastery),
> then learn Tier 2 at the stage you hit it. Build and learn in parallel.

---

## Tier 1 — understand BEFORE you start

### Motor basics
- [ ] **BLDC = electromagnets pulling permanent magnets.** 3 phases (A/B/C) = 3 wires.
      Mixing current into A/B/C steers a magnetic "arrow"; the rotor magnets chase it.
      *Analogy: 3 people pulling ropes on a ring — vary each pull, steer the net pull.*
      🔎 "how does a brushless motor work animation"
- [ ] **Torque = the 90° rule.** Stator arrow pulls hardest when 90° AHEAD of the rotor
      magnets. Straight at them (0°) = no twist; 90° ahead = max twist. FOC keeps the push
      at that perfect 90°, which is why it needs the rotor angle.
      *Analogy: push a merry-go-round at the edge (90°) not toward the center pole (0°).*
      🔎 "field oriented control intuition" / "why 90 degrees motor torque"
- [ ] ⚠️ **Pole pairs + electrical vs mechanical angle (the #1 gotcha).** Rotor has several
      magnet pairs (gimbal motors ~7). One physical spin = 7 electrical cycles.
      `electrical angle = mechanical angle × pole pairs`. Encoder reads mechanical; FOC needs
      electrical. Get it wrong = motor buzzes. Almost everyone trips here first.
      *Analogy: clock face painted with the same 7-color pattern repeated around it.*
      🔎 "pole pairs electrical vs mechanical angle motor"

### Electronics
- [ ] **PWM.** MCU only does on/off; switch fast + vary % on (duty cycle) to fake an
      adjustable voltage. 50% on ≈ half voltage.
      *Analogy: flick a light switch fast — on half the time = half brightness.*
      🔎 "what is PWM explained"
- [ ] **3-phase inverter / half-bridge (concept only — the board does this).** 2 switches
      per phase, 6 total. PWM flips them to shove current through the motor.
      🔎 "three phase inverter motor driver explained"

### Control
- [ ] **Open vs closed loop.** Closed = measure result, error = goal − measured, correct
      continuously. FOC is closed loop.
- [ ] **PI controller.** P = correct in proportion to error now. I = sum leftover error over
      time and kill it. Target current → PI adjusts voltage until actual = target.
      *Analogy: cruise control. P = more gas the further below target; I = keep adding until
      you hit it exactly.*
      🔎 "PID controller explained simply"

### Core FOC idea (the heart)
- [ ] **Hop into the rotor's spinning frame** → the 3 swinging phase currents become steady
      DC numbers, easy to control. **Clarke** (3→2 values), **Park** (rotate into spinning
      frame using rotor angle).
- [ ] **Id = 0 (no torque), Iq = torque (your push dial).** FOC = "hold Id at 0, set Iq."
      *Analogy: a horse on a carousel looks like it's bobbing/circling from the ground, but
      RIDE the carousel and it sits still. Park = hopping on so the spinning thing holds still.*
      🔎 "Clarke and Park transform explained" / "dq frame field oriented control intuition"

### Math comfort
- [ ] sin/cos around a circle; rotating a 2D arrow by an angle (the transforms ARE 2D rotations).
      🔎 "2D rotation matrix intuition"

**The two that matter most: the 90° torque rule + the d-q carousel idea. Respect the
pole-pairs gotcha early.**

---

## Tier 2 — learn AT the stage you hit it
- [ ] Encoders: how they report angle (SPI/I2C), resolution, **offset calibration**
- [ ] Your MCU: timers (make PWM), ADC (read currents), interrupts (run loop at fixed fast rate)
- [ ] Current sensing: shunts + why timing the measurement to the PWM matters
- [ ] The actual Clarke & Park equations
- [ ] 3-phase PWM: start with sinusoidal PWM, upgrade to SVPWM later
- [ ] PI tuning in practice (buzzing / oscillation / sluggish → adjust gains)

## Tier 3 — deepen / reference later
- [ ] Back-EMF; sensored vs sensorless
- [ ] Impedance-control math (for the haptic demo)
- [ ] SVPWM optimization, field weakening, anti-windup, fixed-point vs float

## Resources
- SimpleFOC docs → **Theory** section (read to understand; don't use the lib as your engine)
- Ben Katz blog (Mini Cheetah actuator) — real from-scratch motor control
- YouTube: "FOC explained", "Park Clarke transform intuition"
