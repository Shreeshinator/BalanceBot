# Self-Balancing Robot — No-Nonsense Beginner's Guide

## What You're Building

A two-wheeled robot that balances on its own like a Segway. An IMU measures how far the robot is tilting; a PID controller decides how fast to spin the motors to catch it. The ESP32 also hosts a WiFi hotspot so you can tune PID values from your phone live.

---

## Parts Checklist

| Part | Qty | Notes |
|---|---|---|
| ESP32 DevKit V1 | 1 | 30-pin variant |
| Smartelex 9DOF (ISM330DHCX + MMC5983MA) | 1 | Only the IMU chip is used for balancing |
| TB6612FNG motor driver | 1 | Dual H-bridge |
| Yellow BO gear motors | 2 | With wheels |
| Robot chassis | 1 | Two-wheeled, top-heavy = more stable |
| LiPo 7.4V or 4×AA 6V pack | 1 | Motor supply |
| USB powerbank OR 5V regulator | 1 | ESP32 supply |
| Jumper wires | many | |
| Breadboard | 1 | Optional but helpful |

**Power rule**: ESP32 and TB6612FNG logic (VCC) run on 3.3V–5V. Motors run on their own supply (VM on TB6612FNG). Always share a common GND.

---

## Wiring

### ISM330DHCX → ESP32

```
IMU Board    ESP32 DevKit V1
─────────    ───────────────
VCC      →   3V3
GND      →   GND
SDA      →   GPIO 21
SCL      →   GPIO 22
SDO/ADDR →   (leave unconnected = I2C addr 0x6A)
INT1/INT2→   (not needed)
```

### TB6612FNG → ESP32 + Battery

```
TB6612FNG   ESP32 / Power
─────────   ─────────────────────────
VCC     →   3V3            (logic power)
VM      →   Battery +      (motor power, 4–9V)
GND     →   GND            (common ground!)
STBY    →   GPIO 4         (pull HIGH to enable)
AIN1    →   GPIO 13
AIN2    →   GPIO 14
PWMA    →   GPIO 26        ← Left motor PWM
BIN1    →   GPIO 27
BIN2    →   GPIO 25
PWMB    →   GPIO 32        ← Right motor PWM
AO1/AO2 →  Left motor terminals
BO1/BO2 →  Right motor terminals
```

> ⚠️ **Critical**: Battery GND and ESP32 GND **must be connected together**. If not, the TB6612FNG direction signals won't work.

---

## About the Smartelex 9DOF Board

- **ISM330DHCX**: 3-axis accelerometer + 3-axis gyroscope by ST  
  - Accel range: ±2/4/8/16g  
  - Gyro range: ±125 to ±4000 dps  
  - I2C address: **0x6A** (default, SDO pin low)  
  - Talks I2C at up to 400 kHz — no library needed, the code reads registers directly
- **MMC5983MA**: magnetometer (compass chip) — not used for balancing  

The code uses **no external libraries** for the IMU. It talks directly to the ISM330DHCX over I2C using register addresses. This means no library to install.

---

## How the Balancing Works

```
   IMU reads tilt angle
          ↓
   Complementary filter
   (gyro + accel fusion)
          ↓
   PID controller
   (angle error → motor command)
          ↓
   TB6612FNG drives motors
   (forward if falling forward,
    backward if falling backward)
```

**Complementary filter** fuses two imperfect sensors:
- Gyroscope: accurate over short time, drifts over time
- Accelerometer: gives absolute angle, but noisy when motors vibrate

Formula: `angle = 0.98 × (angle + gyro_rate × dt) + 0.02 × accel_angle`

**PID controller**:
- **P (Proportional)**: the bigger the tilt, the harder the motor pushes — main balancing force
- **I (Integral)**: fixes a robot that always leans slightly one way
- **D (Derivative)**: slows the response down as it approaches balance — stops oscillation

---

## IMU Mounting & The Setpoint Problem

**The problem**: The IMU reads some non-zero angle when the robot is actually at its balance point, because it's mounted at an angle or not perfectly parallel to the wheel axle.

**The solution**: Don't assume setpoint = 0°. Use the web interface to set it.

### Finding the right setpoint:
1. Upload the code and open Serial Monitor (115200 baud)
2. Hold the robot at the exact point where it would balance (you can feel the "sweet spot" — lean it and find where gravity is neutral)
3. Note the angle shown on Serial Monitor
4. Open the web UI and click **"Capture Current Angle as Setpoint"**

The robot will now try to maintain **that specific angle** instead of 0°.

### Finding the right IMU axis (`PITCH_AXIS`):
In the code at the top, there's `#define PITCH_AXIS 0`. This controls which accelerometer axes are used to calculate tilt.

- Tilt the robot forward. The angle should **increase**.
- Tilt the robot backward. The angle should **decrease**.
- If it's reversed: flip `MOTOR_LEFT_DIR` and `MOTOR_RIGHT_DIR` (easier than changing axis)
- If angle doesn't change at all: try `PITCH_AXIS 1`, recompile, reupload

---

## Arduino IDE Setup

1. **Install ESP32 board support** (if not already):
   - File → Preferences → Additional Board URLs → add:  
     `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Tools → Board → Boards Manager → search "esp32" → install "esp32 by Espressif"

2. **No additional libraries needed** — the code uses only built-in ESP32 libraries (Wire, WiFi, WebServer)

3. **Board settings**:
   - Board: `ESP32 Dev Module`
   - Port: your COM/tty port
   - Upload Speed: 921600
   - Flash Frequency: 80MHz
   - Partition Scheme: Default

4. **Upload**: Press and hold the BOOT button on ESP32, click Upload, release BOOT when "Connecting..." appears (some boards do this automatically)

---

## First Run Procedure

```
Step 1: Upload code with Serial Monitor open (115200 baud)
Step 2: You'll see:
        ISM330DHCX WHO_AM_I = 0x6B ✓ OK
        Calibrating gyro — keep robot PERFECTLY STILL for 2 sec...
        Gyro offset: -0.0523 dps
        Initial angle: 12.34°
        WiFi AP "BalanceBot" started. Open: http://192.168.4.1
Step 3: Connect phone/laptop to WiFi: "BalanceBot" (password: 12345678)
Step 4: Open browser → http://192.168.4.1
Step 5: DO NOT enable yet. Hold robot upright, click "Capture Setpoint"
Step 6: Now click "Enable Robot"
Step 7: Robot will try to balance — it will fall many times while tuning PID
```

---

## PID Tuning Guide

Start with all zeros and work up:

### Step 1 — Proportional only
```
Kp = 20, Ki = 0, Kd = 0
```
The robot should react to tilting but oscillate. If it doesn't react at all, increase Kp. If it falls immediately and motors spin max, reduce Kp. Target: robot tries to correct but oscillates quickly.

### Step 2 — Add Derivative
```
Kp = 20, Ki = 0, Kd = 0.5 → increase until oscillation damps
```
Kd slows the response as it approaches vertical. Too much Kd → robot vibrates rapidly (high frequency). Good Kd → robot approaches balance and stops without wobbling.

### Step 3 — Add Integral
```
Ki = 0.2 → increase slowly
```
If the robot stays slightly tilted even when balancing, Ki fixes this. Too much Ki → slow rocking oscillation.

### Typical good starting values for BO motors + lightweight chassis:
```
Kp = 22–35
Ki = 0.3–1.0  
Kd = 0.7–2.0
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| WHO_AM_I = 0x00 or wrong | Check SDA/SCL wiring. Try I2C address 0x6B (SDO high) |
| Motors don't spin | Check STBY is GPIO 4 and HIGH. Check VM has motor power. Check GND is common |
| Motors spin wrong direction | Flip `MOTOR_LEFT_DIR` or `MOTOR_RIGHT_DIR` to -1 |
| Robot immediately falls | Setpoint is wrong — recapture it. Or Kp is too low |
| Robot oscillates rapidly | Kd too low — increase it. Or Kp too high — reduce |
| Robot slowly rocks back/forth | Ki too high — reduce. Or setpoint slightly off |
| IMU not detected | Verify 3.3V on IMU VCC pin. Try slower I2C: change `Wire.setClock(400000)` to `Wire.setClock(100000)` |
| WiFi not appearing | Normal. Reboot ESP32 and wait 5 seconds |
| Motors twitch but robot doesn't move | Increase `MIN_MOTOR_PWM` from 40 to 60 or 70 |
| LEDC compile error | You're on ESP32 Core 3.x. Replace `ledcSetup+ledcAttachPin` with `ledcAttach(pin, freq, bits)` and `ledcWrite(pin, duty)` |

---

## Getting Maximum Stability

1. **Mount the IMU rigidly** — vibration = noisy accel = bad angle estimate. Hot glue or bolt it down. Don't let it flex.

2. **Centre of mass high** — counter-intuitively, a higher CoM (battery on top) makes balancing **easier** because it falls more slowly. Low CoM = fast tipping = hard to catch.

3. **Wheels with grip** — slipping wheels mean the robot can't catch itself.

4. **Adequate motor power** — BO motors with 6V+ battery respond faster. At 3.7V they're sluggish.

5. **Keep the loop fast** — `handleClient()` is the slowest part. Typical loop rate is 150–250 Hz which is fine.

6. **Anti-windup** — already in the code. Don't remove the `pidIntegral` clamp.

7. **Increase compAlpha** — try 0.985 or 0.99 if you have motor vibration noise. Higher = more gyro trust = smoother but slightly more drift.

---

## Files

- `BalanceBot.ino` — the complete code, upload this directly in Arduino IDE

---

*No extra libraries needed. WiFi is hotspot mode (no router required). Tune from any browser.*
