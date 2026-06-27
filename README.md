# Micromouse 2025

Autonomous maze-solving robot built for the 2025 Micromouse competition. Custom PCB, Teensy 4.1, and a full online Flood Fill implementation with PID-based motion control.

---

## Hardware

- **Teensy 4.1** as the main controller
- **3× VL53L0X** Time-of-Flight sensors (front, left, right) — all on one I²C bus, assigned unique addresses at boot via XSHUT pins
- **BNO055 IMU** for absolute yaw (external crystal enabled)
- **2× DC motors** with quadrature encoders
- Custom PCB (KiCad files included)

---

## How it works

The robot uses an **online Flood Fill** algorithm. Before moving, it runs a BFS from the goal cells (the central 2×2 of the 16×16 maze) to assign every cell a distance value. At each step it reads the three sensors, updates the wall map, and if it finds a new wall it reruns the BFS before deciding where to go next. This way it always has an up-to-date shortest path even as the maze reveals itself.

Motion is handled by three PID loops running together each cycle:
- **Forward PID** — controls wheel speed via encoder feedback, with distance-based speed ramping (slows down as the front sensor reading drops)
- **Turning PID** — executes 90° and 180° turns using absolute yaw from the BNO055, with proper angle wraparound so it doesn't get confused near 0°/360°
- **Centering PID** — keeps the robot centered in the corridor by blending a correction term into the motor PWM outputs based on the difference between left and right sensor readings

---

## Files

```
main_control.ino         — setup/loop, wall detection, navigation decisions
floodfill.h/.cpp         — maze state, BFS, solve()
sensors.h/.cpp           — VL53L0X + BNO055 init and reads
motor_control.h/.cpp     — PWM, encoder ISRs, turn routines
forward_pid.h/.cpp       — speed PID
turning_pid.h/.cpp       — turn PID
centering_pid.h/.cpp     — wall centering PID
VL53L0X_SingleTest.ino   — standalone sensor test
micromouse2.kicad_sch    — schematic
micromouse2.kicad_pcb    — PCB layout
```

---

## Setup

Install [Teensyduino](https://www.pjrc.com/teensy/td_download.html), open `main_control.ino`, set board to Teensy 4.1, and install these libraries via the Library Manager:
- `VL53L0X` (Pololu)
- `Adafruit_BNO055` + `Adafruit_Sensor`

On boot the robot waits 5 seconds before starting — enough time to place it at the maze entrance and switch on motor power.

Key constants to tune for your physical setup are `WALL_THRESHOLD` (165 mm), `FRONT_WALL_THRESHOLD` (100 mm), and `TICKS_PER_MM` (7.38) in `floodfill.h`.
