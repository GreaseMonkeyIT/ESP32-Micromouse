## Hardware Setup

This Micromouse build uses a simple low-cost sensing and control stack intended
for rapid iteration on maze-solving logic before moving to a faster competition
platform.

### Core hardware

- **ESP32** as the main controller
- **2 N20 DC gear motors** for differential drive
- **Wheel encoders** for distance-based forward motion
- **MPU6050** for yaw tracking during turns
- **3 ultrasonic sensors** for left / front / right wall detection
- **Battery pack** sized for both motor and logic rails
- **Caster wheel** for mechanical stability

### Sensor / control roles

- **Encoders** determine when one cell-length move is complete.
- **MPU6050** is used only during turn maneuvers, where delta-angle targeting
  reduces drift accumulation across repeated turns.
- **Ultrasonic sensors** provide binary wall presence used by the flood-fill
  layer to update the maze map.

### Integration notes

1. Mount the N20 motors so both wheel axes are square to the chassis.
2. Align the encoders carefully; forward distance accuracy depends on clean pulse counts.
3. Place the three ultrasonic sensors with clear left / front / right coverage.
4. Connect the MPU6050 over I2C and perform offset calibration once during `setup()`.
5. Route wiring away from wheels and keep sensor wiring mechanically secure.
6. Validate motor polarity early so "forward", "left", and "right" primitives match the software assumptions.

### Current project state

- The hardware-side code has been refactored beyond the original simulator-era version.
- Navigation now uses absolute wall bitmasks with reciprocal wall writes.
- Turn control uses per-turn delta-angle targets instead of accumulated heading.
- Sensor averaging has been shortened to reduce loop blocking time.

This file is meant as a concise integration note, not a full assembly manual.
