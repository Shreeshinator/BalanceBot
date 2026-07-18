# BalanceBot

BalanceBot is an ESP32-powered self-balancing robot project built for learning and fast iteration.  
It uses a Smartelex 9DOF board (ISM330DHCX IMU), a TB6612FNG motor driver, and a web-based tuning panel so you can adjust control settings from your phone in real time.

## Highlights

- No external IMU libraries required (direct register reads)
- PID balancing control with anti-windup safeguards
- Kalman-based angle estimation from gyro + accelerometer data
- Built-in Wi-Fi access point and browser UI for live tuning
- One-click **Capture Setpoint** for non-perfect IMU mounting
- Safety cutoff when tilt exceeds the configured safe angle

## Hardware

- ESP32 DevKit V1 (30-pin)
- Smartelex 9DOF (ISM330DHCX + MMC5983MA)
- TB6612FNG dual H-bridge motor driver
- 2 × BO geared motors + wheels
- 2-wheel chassis (included: `Chassis 1.stl`)
- Motor battery pack (typically 6–9V)
- ESP32 power source (USB power bank or regulated 5V)

> Important: motor power and ESP32 logic power should be separate, but all grounds must be shared.

## Quick Start

1. Wire the IMU and TB6612FNG to the ESP32 (see `GUIDE.md` for full pin map and diagrams).
2. Open `BalanceBot.ino` in Arduino IDE.
3. Install **ESP32 by Espressif** board support if needed.
4. Select **ESP32 Dev Module** and upload the sketch.
5. Connect to Wi-Fi: **BalanceBot** (password: **12345678**).
6. Open **http://192.168.4.1**.
7. Hold the robot at its balance point, click **Capture Setpoint**, then **Enable Robot**.
8. Tune Kp/Ki/Kd from the web UI.

## Repository Contents

- `BalanceBot.ino` — complete firmware for balancing, motor control, and web UI
- `GUIDE.md` — full beginner guide (wiring, tuning, troubleshooting)
- `Chassis 1.stl` — chassis model file

## Tuning Notes

- Start with low values and increase gradually.
- Tune **Kp** first, then **Kd**, then **Ki**.
- If motors twitch but do not move, increase `MIN_MOTOR_PWM`.
- If direction is reversed, adjust `MOTOR_LEFT_DIR` / `MOTOR_RIGHT_DIR`.

## License

No license file is currently defined in this repository. Add one if you plan to share or distribute this project publicly.
