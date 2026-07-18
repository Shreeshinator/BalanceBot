/*
  SELF-BALANCING ROBOT
  ESP32 DevKit V1 + Smartelex 9DOF (ISM330DHCX) + TB6612FNG
  
  Features:
    - No-library IMU (direct register access, no install needed)
    - Kalman filter (accel + gyro fusion)
    - PID controller with anti-windup
    - WiFi Access Point + live web tuner at 192.168.4.1
    - "Capture Setpoint" button for angled IMU mounting
    - Safety cutoff if robot falls too far

  TB6612FNG  →  ESP32 / Power
    VCC  →  3V3 (logic supply)
    VM   →  Battery+ (4–9V motor supply)
    GND  →  GND (shared with ESP32 and battery -)
    STBY →  GPIO 4
    AIN1 →  GPIO 13
    AIN2 →  GPIO 14
    PWMA →  GPIO 26   ← Left Motor speed
    BIN1 →  GPIO 27
    BIN2 →  GPIO 25
    PWMB →  GPIO 32   ← Right Motor speed
  ─────────────────────────────────────────────────
  NOTE: Use a common GND between ESP32 and battery.
  Power ESP32 from its VIN (5V from regulator/USB powerbank)
  or USB. Don't share motor power with ESP32 3.3V rail.
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

//  SECTION 1: USER CONFIG — Edit these first!

// --- IMU Axis Selection ---
// With IMU mounted flat (sensor side up):
//   PITCH_AXIS = 0  → uses atan2(ax, az)  [most common]
// With IMU rotated 90° (mounted on side panel, landscape):
//   PITCH_AXIS = 1  → uses atan2(ay, az)
// With IMU tilted (another orientation):
//   PITCH_AXIS = 2  → uses atan2(ax, ay)
//
// HOW TO FIND THE RIGHT ONE:
//   Open Serial Monitor at 115200. Tilt robot forward/back.
//   The angle should increase when leaning forward and
//   decrease when leaning back. Try each until it works.
#define PITCH_AXIS   0     // 0, 1, or 2 — see above

// Matching gyro axis for pitch rate (usually same as PITCH_AXIS)
//   0 = gx,  1 = gy,  2 = gz
#define GYRO_AXIS    1     // 0, 1, or 2

// --- Motor Direction ---
// If a motor spins the wrong way, flip its sign here.
// Correct behavior: positive output → robot moves forward.
#define MOTOR_LEFT_DIR   1    //  1 or -1
#define MOTOR_RIGHT_DIR -1    //  1 or -1

// --- WiFi ---
#define WIFI_SSID  "BalanceBot"
#define WIFI_PASS  "12345678"

//  SECTION 2: PIN DEFINITIONS

// I2C (ISM330DHCX)
#define PIN_SDA   21
#define PIN_SCL   22

// TB6612FNG Motor Driver
#define PIN_STBY  4     // Standby — HIGH to enable driver

#define PIN_AIN1  13   // Motor A (Left) direction
#define PIN_AIN2  14
#define PIN_PWMA  26    // Motor A speed (PWM)

#define PIN_BIN1  27    // Motor B (Right) direction
#define PIN_BIN2  25
#define PIN_PWMB  32    // Motor B speed (PWM)

// LEDC (ESP32 hardware PWM)
#define LEDC_CHAN_A   0
#define LEDC_CHAN_B   1
#define LEDC_FREQ     10000   // 10 kHz — inaudible, smooth
#define LEDC_BITS     8       // 8-bit resolution (0–255)

//  SECTION 3: ISM330DHCX REGISTER DEFINITIONS

// I2C address (SDO pin unconnected = pulled low = 0x6A)
// If your board has SDO connected to 3.3V, use 0x6B
#define ISM330_ADDR   0x6B

#define REG_WHO_AM_I  0x0F   // Should return 0x6B
#define REG_CTRL1_XL  0x10   // Accelerometer config
#define REG_CTRL2_G   0x11   // Gyroscope config
#define REG_CTRL3_C   0x12   // General config
#define REG_OUTX_L_G  0x22   // Gyro output start (6 bytes)
#define REG_OUTX_L_A  0x28   // Accel output start (6 bytes)

// Sensitivity for our chosen full-scale ranges:
//   Gyro  ±500 dps  → 17.5 mdps/LSB
//   Accel ±2 g      → 0.061 mg/LSB
#define GYRO_SCALE    0.0175f      // dps per raw LSB
#define ACCEL_SCALE   0.000061f    // g   per raw LSB

//  SECTION 4: PID & FILTER DEFAULTS
//
// These are starting points. Tune via the web interface.
//
// Typical tuning procedure:
//   1. Set Ki=0, Kd=0. Increase Kp until robot oscillates.
//   2. Halve Kp. Increase Kd until oscillations damp.
//   3. Increase Ki slowly to remove residual lean.
//   4. Fine-tune setpoint with "Capture" button in web UI.

float Kp = 25.0f;
float Ki = 0.4f;
float Kd = 0.9f;

// Balance angle — NOT 0° if your IMU is mounted at an angle!
// Use the web interface "Capture Setpoint" button to set this
// while holding the robot at its physical balance point.
float setpoint = 0.0f;

// Safety: cut motors if robot leans more than this from setpoint
float maxSafeAngle = 70.0f;

// Complementary filter weight (0 = trust only accel, 1 = trust only gyro)
// 0.98 is the sweet spot for balance bots
float compAlpha = 0.98f;

// Minimum PWM to actually move BO motors (they stall below ~40)
#define MIN_MOTOR_PWM  50

// =========================================================
//  SECTION 5: GLOBAL STATE
// =========================================================

float currentAngle  = 0.0f;
float currentOutput = 0.0f;
float gyroOffset    = 0.0f;   // Calibrated gyro zero

float pidIntegral   = 0.0f;
float pidLastError  = 0.0f;

bool robotEnabled   = false;  // Start disabled — enable from web UI
unsigned long lastLoopUs = 0;

float loopHz = 0.0f;

float kalAngle = 0, kalBias = 0;
float kalP[2][2] = {{0,0},{0,0}};
// Tune these if needed:
const float kalQ_angle = 0.001f;  // Process noise: angle
const float kalQ_bias  = 0.003f;  // Process noise: bias drift
const float kalR_meas  = 0.03f; 

WebServer webServer(80);

// NEW --> dual core rtos split for pid and webserver
TaskHandle_t pidTaskHandle;
//protos
void imuReadAll(float&, float&, float&, float&, float&, float&);
float kalmanUpdate(float measuredAngle, float gyroRate, float dt);
void motorsOff();
void setMotors(float);
bool imuInit();
void calibrateGyro();
void setupWebServer();

void pidTask(void* pvParams) {
  lastLoopUs = micros();
  for (;;) {
    unsigned long nowUs = micros();
    float dt = (nowUs - lastLoopUs) / 1000000.0f;
    lastLoopUs = nowUs;
    dt = constrain(dt, 0.001f, 0.02f);

    float ax, ay, az, gx, gy, gz;
    imuReadAll(ax, ay, az, gx, gy, gz);

    float gyroRate;
    if      (GYRO_AXIS == 0) gyroRate = gx - gyroOffset;
    else if (GYRO_AXIS == 1) gyroRate = gy - gyroOffset;
    else                     gyroRate = gz - gyroOffset;

    float accelAngle;
    if      (PITCH_AXIS == 0) accelAngle = atan2f(ax, az) * 180.0f / M_PI;
    else if (PITCH_AXIS == 1) accelAngle = atan2f(ay, az) * 180.0f / M_PI;
    else                      accelAngle = atan2f(ax, ay) * 180.0f / M_PI;

  static float accelAngleFiltered = 0;
  accelAngleFiltered = 0.7f * accelAngleFiltered + 0.3f * accelAngle;
  // Then use accelAngleFiltered instead of accelAngle in the filter
    currentAngle = kalmanUpdate(accelAngleFiltered, gyroRate, dt);

    loopHz = 0.95f * loopHz + 0.05f * (1.0f / dt);

    float error = currentAngle - setpoint;
    if (!robotEnabled || fabsf(error) > maxSafeAngle) {
      motorsOff();
      pidIntegral = 0.0f;
      currentOutput = 0.0f;
      vTaskDelay(1);
      continue;
    }

    float Pout = Kp * error;
    pidIntegral += error * dt;
    float integralLimit = (Ki > 0.001f) ? (200.0f / Ki) : 5000.0f;
    pidIntegral = constrain(pidIntegral, -integralLimit, integralLimit);
    float Iout = Ki * pidIntegral;
    // D term: use raw gyro directly — WAY cleaner than differencing angle
    float Dout = -Kd * gyroRate;   // <-- see change #2 below

    currentOutput = Pout + Iout + Dout;
    setMotors(currentOutput);

    // Target ~500Hz — adjust this number
    vTaskDelay(pdMS_TO_TICKS(2));
  static unsigned long dbg = 0;
  if (millis() - dbg > 1000) {
    Serial.printf("enabled=%d  angle=%.2f  error=%.2f  output=%.1f\n",
      robotEnabled, currentAngle, currentAngle - setpoint, currentOutput);
    dbg = millis();
  }
  }
}

// =========================================================
//  SECTION 6: IMU FUNCTIONS
// =========================================================

void imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ISM330_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t imuRead(uint8_t reg) {
  Wire.beginTransmission(ISM330_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ISM330_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

// Read all 6 axes in one I2C burst (efficient, no missed frames)
void imuReadAll(float &ax, float &ay, float &az,
                float &gx, float &gy, float &gz) {
  Wire.beginTransmission(ISM330_ADDR);
  Wire.write(REG_OUTX_L_G);
  Wire.endTransmission(false);
  Wire.requestFrom(ISM330_ADDR, (uint8_t)12);

  // Gyro first (3 axes × 2 bytes), then Accel (3 axes × 2 bytes)
  int16_t rawGX = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t rawGY = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t rawGZ = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t rawAX = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t rawAY = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t rawAZ = (int16_t)(Wire.read() | (Wire.read() << 8));

  gx = rawGX * GYRO_SCALE;
  gy = rawGY * GYRO_SCALE;
  gz = rawGZ * GYRO_SCALE;
  ax = rawAX * ACCEL_SCALE;
  ay = rawAY * ACCEL_SCALE;
  az = rawAZ * ACCEL_SCALE;
}

bool imuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);   // 400 kHz fast mode
  delay(20);

  uint8_t id = imuRead(REG_WHO_AM_I);
  Serial.printf("ISM330DHCX WHO_AM_I = 0x%02X", id);
  if (id == 0x6B) {
    Serial.println(" ✓ OK");
  } else {
    Serial.println(" ✗ FAIL — check wiring & I2C address!");
    return false;
  }

  // CTRL1_XL: ODR=208 Hz (0101xxxx), FS=±2g (xx00xxxx)
  // Byte: 0101 0000 = 0x50
  imuWrite(REG_CTRL1_XL, 0x50);

  // CTRL2_G: ODR=208 Hz (0101xxxx), FS=±500 dps (xxx0 10xx)
  // Byte: 0101 0100 = 0x54
  imuWrite(REG_CTRL2_G, 0x54);

  // CTRL3_C: BDU=1 (block data update, bit6), IF_INC=1 (auto addr inc, bit2)
  // Byte: 0100 0100 = 0x44
  imuWrite(REG_CTRL3_C, 0x44);

  delay(100);   // Let ODR settle
  return true;
}

void calibrateGyro() {
  Serial.println("Calibrating gyro — keep robot PERFECTLY STILL for 2 sec...");
  const int N = 400;
  float sum = 0;
  for (int i = 0; i < N; i++) {
    float ax, ay, az, gx, gy, gz;
    imuReadAll(ax, ay, az, gx, gy, gz);
    float g = (GYRO_AXIS == 0) ? gx : (GYRO_AXIS == 1) ? gy : gz;
    sum += g;
    delay(5);
  }
  gyroOffset = sum / N;
  Serial.printf("Gyro offset: %.4f dps\n", gyroOffset);

  // Seed angle from accelerometer so filter starts correct
  float ax, ay, az, gx, gy, gz;
  imuReadAll(ax, ay, az, gx, gy, gz);
  if      (PITCH_AXIS == 0) currentAngle = atan2f(ax, az) * 180.0f / M_PI;
  else if (PITCH_AXIS == 1) currentAngle = atan2f(ay, az) * 180.0f / M_PI;
  else                      currentAngle = atan2f(ax, ay) * 180.0f / M_PI;
  Serial.printf("Initial angle: %.2f°\n", currentAngle);
  kalAngle = currentAngle;
}

// =========================================================
//  SECTION 7: MOTOR CONTROL
// =========================================================

void motorsInit() {
  pinMode(PIN_AIN1, OUTPUT);  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);  // Enable driver

  // ESP32 Arduino Core 2.x LEDC API
  // (If you're on Core 3.x, replace with ledcAttach(pin, freq, bits))
  ledcAttach(PIN_PWMA, LEDC_FREQ, LEDC_BITS);
  ledcAttach(PIN_PWMB, LEDC_FREQ, LEDC_BITS);
}

// Drive one H-bridge channel. speed: -255 to +255
void driveChannel(uint8_t in1, uint8_t in2, uint8_t pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    ledcWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
    ledcWrite(pwmPin, -speed);
  } else {
    // Fast decay (coast): both inputs LOW
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
    ledcWrite(pwmPin, 0);
  }
}

// Apply PID output to both motors.
// Positive output = both motors drive "forward" (catching a forward lean).
void setMotors(float rawOutput) {
  int out = (int)constrain(rawOutput, -255.0f, 255.0f);

  // Dead zone: too small to do anything → zero
  if (abs(out) < 12) {
    out = 0;
  }
// Stiction zone: too small to move but non-trivial → snap to minimum
  else if (abs(out) < MIN_MOTOR_PWM) {
    out = (out > 0) ? MIN_MOTOR_PWM : -MIN_MOTOR_PWM;
  }

  driveChannel(PIN_AIN1, PIN_AIN2, PIN_PWMA, MOTOR_LEFT_DIR  * out);
  driveChannel(PIN_BIN1, PIN_BIN2, PIN_PWMB, MOTOR_RIGHT_DIR * out);
}

void motorsOff() {
  driveChannel(PIN_AIN1, PIN_AIN2, PIN_PWMA, 0);
  driveChannel(PIN_BIN1, PIN_BIN2, PIN_PWMB, 0);
}

// =========================================================
//  SECTION 8: WEB INTERFACE HTML
// =========================================================

// Stored in flash (PROGMEM) to save RAM
const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BalanceBot Tuner</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:system-ui,sans-serif;padding:14px;max-width:480px;margin:auto}
h1{font-size:1.3em;color:#58a6ff;margin-bottom:14px}
.card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:14px;margin-bottom:12px}
.row{display:flex;gap:12px}
.cell{flex:1;text-align:center}
.big{font-size:2em;font-weight:700;line-height:1.1}
.green{color:#3fb950}.red{color:#f85149}.blue{color:#58a6ff}.yellow{color:#d29922}
.sub{font-size:.75em;color:#8b949e;margin-top:2px}
.bar-bg{height:10px;background:#21262d;border-radius:5px;overflow:hidden;margin:10px 0}
.bar{height:100%;background:#58a6ff;border-radius:5px;transition:width .08s}
label{display:block;font-size:.8em;color:#8b949e;margin:10px 0 3px}
input[type=number]{width:100%;background:#21262d;color:#c9d1d9;border:1px solid #30363d;border-radius:6px;padding:7px 10px;font-size:.95em}
input[type=number]:focus{border-color:#58a6ff;outline:none}
.btn{width:100%;border:none;border-radius:7px;padding:10px;font-size:.95em;cursor:pointer;margin-top:6px;font-weight:600}
.btn-blue{background:#1f6feb;color:#fff}
.btn-green{background:#238636;color:#fff}
.btn-red{background:#da3633;color:#fff}
.btn-gray{background:#21262d;color:#c9d1d9;border:1px solid #30363d}
.tip{font-size:.78em;color:#8b949e;line-height:1.5}
.tip b{color:#d29922}
.status{font-size:.8em;margin-top:8px}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:5px}
</style>
</head>
<body>
<h1>⚖️ BalanceBot Tuner</h1>

<div class="card">
  <div class="row">
    <div class="cell">
      <div class="sub">Tilt Angle</div>
      <div class="big" id="ang" style="color:#58a6ff">--</div>
      <div class="sub">degrees</div>
    </div>
    <div class="cell">
      <div class="sub">Motor Drive</div>
      <div class="big" id="out" style="color:#3fb950">--</div>
      <div class="sub">/ ±255</div>
    </div>
    <div class="cell">
      <div class="sub">Loop Rate</div>
      <div class="big" id="hz" style="color:#d29922">--</div>
      <div class="sub">Hz</div>
    </div>
  </div>
  <div class="bar-bg"><div class="bar" id="angbar" style="width:50%"></div></div>
  <div class="status" id="statusline"><span class="dot" style="background:#555" id="dot"></span>Connecting...</div>
</div>

<div class="card">
  <b style="color:#c9d1d9">PID Parameters</b>
  <label>Kp — Proportional (main restoring force, 15-40 typical)</label>
  <input type="number" id="kp" step="0.5" min="0" max="200">
  <label>Ki — Integral (removes steady lean, 0-2 typical)</label>
  <input type="number" id="ki" step="0.05" min="0" max="20">
  <label>Kd — Derivative (damps oscillation, 0.5-3 typical)</label>
  <input type="number" id="kd" step="0.05" min="0" max="20">
  <label>Setpoint (°) — angle where robot balances</label>
  <input type="number" id="sp" step="0.1">
  <label>Max Safe Angle (°) — motors cut if exceeded</label>
  <input type="number" id="ma" step="1" min="5" max="90">
  <button class="btn btn-blue" onclick="applyPID()">✓ Apply Parameters</button>
  <button class="btn btn-gray" onclick="captureSetpoint()" style="margin-top:5px">
    📐 Capture Current Angle as Setpoint
  </button>
</div>

<div class="card row" style="gap:8px">
  <button class="btn btn-green" onclick="setEn(1)">▶ Enable Robot</button>
  <button class="btn btn-red" onclick="setEn(0)">■ Disable</button>
</div>

<div class="card">
  <b style="color:#d29922">Tuning Guide</b>
  <p class="tip" style="margin-top:8px">
    <b>Step 1</b> — Set Ki=0, Kd=0. Raise Kp until robot tries to balance but oscillates.<br>
    <b>Step 2</b> — Halve Kp. Raise Kd to damp oscillations.<br>
    <b>Step 3</b> — Raise Ki slowly (0.1 at a time) to fix residual lean.<br>
    <b>Step 4</b> — If motors twitch but robot doesn't move, raise MIN_MOTOR_PWM in code.<br>
    <b>Setpoint</b> — Hold robot at real balance point, click Capture. Re-enable.
  </p>
</div>

<script>
let liveAngle=0, lastTime=Date.now(), loopCnt=0;

async function poll(){
  try{
    const d=await(await fetch('/data')).json();
    liveAngle=d.angle;
    document.getElementById('ang').textContent=d.angle.toFixed(1);
    document.getElementById('out').textContent=(d.output>=0?'+':'')+Math.round(d.output);
    document.getElementById('hz').textContent=Math.round(d.hz);
    // Angle bar: ±30° mapped to 0-100%
    const pct=Math.min(Math.max(50+d.angle*100/60,2),98);
    document.getElementById('angbar').style.width=pct+'%';
    const col=d.enabled?'#3fb950':'#f85149';
    document.getElementById('dot').style.background=col;
    document.getElementById('statusline').innerHTML=
      '<span class="dot" style="background:'+col+'" id="dot"></span>'+
      (d.enabled?'🟢 Robot ENABLED':'🔴 Robot DISABLED')+
      ' | Setpoint: '+d.sp.toFixed(1)+'°';
    // Sync inputs on first load
    if(!document.getElementById('kp').dataset.set){
      ['kp','ki','kd','sp','ma'].forEach(k=>{
        document.getElementById(k).value=d[k];
      });
      document.getElementById('kp').dataset.set='1';
    }
  }catch(e){}
}

function applyPID(){
  const p=new URLSearchParams({
    kp:document.getElementById('kp').value,
    ki:document.getElementById('ki').value,
    kd:document.getElementById('kd').value,
    sp:document.getElementById('sp').value,
    ma:document.getElementById('ma').value
  });
  fetch('/set?'+p).then(poll);
}

function captureSetpoint(){
  document.getElementById('sp').value=liveAngle.toFixed(2);
  fetch('/set?sp='+liveAngle.toFixed(3))
    .then(()=>alert('Setpoint locked to '+liveAngle.toFixed(2)+'°'));
}

function setEn(v){fetch('/enable?v='+v).then(poll);}

setInterval(poll,120);
poll();
</script>
</body></html>
)rawhtml";

// =========================================================
//  SECTION 9: WEB SERVER SETUP
// =========================================================

   // Measured loop rate for web display

void setupWebServer() {
  webServer.on("/", []() {
    webServer.send_P(200, "text/html", INDEX_HTML);
  });

  // Live telemetry JSON endpoint
  webServer.on("/data", []() {
    String j = "{";
    j += "\"angle\":"  + String(currentAngle,  3) + ",";
    j += "\"output\":" + String(currentOutput, 1) + ",";
    j += "\"hz\":"     + String(loopHz,         1) + ",";
    j += "\"enabled\":" + String(robotEnabled ? "true" : "false") + ",";
    j += "\"kp\":"     + String(Kp,         2) + ",";
    j += "\"ki\":"     + String(Ki,         4) + ",";
    j += "\"kd\":"     + String(Kd,         4) + ",";
    j += "\"sp\":"     + String(setpoint,   3) + ",";
    j += "\"ma\":"     + String(maxSafeAngle, 1);
    j += "}";
    webServer.send(200, "application/json", j);
  });

  // Apply PID params
  webServer.on("/set", []() {
    if (webServer.hasArg("kp")) Kp           = webServer.arg("kp").toFloat();
    if (webServer.hasArg("ki")) Ki           = webServer.arg("ki").toFloat();
    if (webServer.hasArg("kd")) Kd           = webServer.arg("kd").toFloat();
    if (webServer.hasArg("sp")) setpoint     = webServer.arg("sp").toFloat();
    if (webServer.hasArg("ma")) maxSafeAngle = webServer.arg("ma").toFloat();
    pidIntegral = 0.0f;  // Reset integral whenever params change
    webServer.send(200, "text/plain", "OK");
  });

  // Enable / Disable
  webServer.on("/enable", []() {
    if (webServer.hasArg("v")) {
      robotEnabled = (webServer.arg("v").toInt() == 1);
      if (!robotEnabled) { motorsOff(); pidIntegral = 0.0f; }
    }
    webServer.send(200, "text/plain", "OK");
  });

  webServer.begin();
}

// Optionl testing kalman filter
// ---- Kalman Filter State ----
// float kalAngle = 0, kalBias = 0;
// float kalP[2][2] = {{0,0},{0,0}};
// // Tune these if needed:
// const float kalQ_angle = 0.001f;  // Process noise: angle
// const float kalQ_bias  = 0.003f;  // Process noise: bias drift
// const float kalR_meas  = 0.03f;   // Measurement noise (higher = smoother, laggier)

float kalmanUpdate(float measuredAngle, float gyroRate, float dt) {
  // Predict step
  kalAngle += dt * (gyroRate - kalBias);
  kalP[0][0] += dt * (dt*kalP[1][1] - kalP[0][1] - kalP[1][0] + kalQ_angle);
  kalP[0][1] -= dt * kalP[1][1];
  kalP[1][0] -= dt * kalP[1][1];
  kalP[1][1] += kalQ_bias * dt;

  // Update step
  float S  = kalP[0][0] + kalR_meas;
  float K0 = kalP[0][0] / S;
  float K1 = kalP[1][0] / S;
  float innov = measuredAngle - kalAngle;
  kalAngle += K0 * innov;
  kalBias  += K1 * innov;
  float P00_temp = kalP[0][0];
  float P01_temp = kalP[0][1];
  kalP[0][0] -= K0 * P00_temp;
  kalP[0][1] -= K0 * P01_temp;
  kalP[1][0] -= K1 * P00_temp;
  kalP[1][1] -= K1 * P01_temp;
  return kalAngle;
}

// =========================================================
//  SECTION 10: SETUP
// =========================================================
void testMotorsIsolated() {
  Serial.println("--- LEFT FORWARD ---");
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
  ledcWrite(PIN_PWMA, 200); delay(1000); ledcWrite(PIN_PWMA, 0);

  Serial.println("--- LEFT REVERSE ---");
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, HIGH);
  ledcWrite(PIN_PWMA, 200); delay(1000); ledcWrite(PIN_PWMA, 0);

  Serial.println("--- RIGHT FORWARD ---");
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
  ledcWrite(PIN_PWMB, 200); delay(1000); ledcWrite(PIN_PWMB, 0);

  Serial.println("--- RIGHT REVERSE ---");
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, HIGH);
  ledcWrite(PIN_PWMB, 200); delay(1000); ledcWrite(PIN_PWMB, 0);
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Reset reason: %d\n", esp_reset_reason());
  Serial.println("\n\n=== BalanceBot Startup ===");

  // Motors safe first (always!)
  motorsInit();
  motorsOff();
  Serial.println("Motors: OK");

  // IMU
  if (!imuInit()) {
    Serial.println("HALTED — fix IMU before continuing.");
    while (true) delay(1000);
  }
  calibrateGyro();

  // WiFi — creates its own hotspot, no router needed
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("WiFi AP \"%s\" started. Open: http://%s\n",
                WIFI_SSID, ip.toString().c_str());

  setupWebServer();
  lastLoopUs = micros();

  Serial.println("\nREADY. Steps:");
  Serial.println(" 1. Connect phone/PC to WiFi: " WIFI_SSID);
  Serial.println(" 2. Open browser: http://192.168.4.1");
  Serial.println(" 3. Hold robot at balance point → click 'Capture Setpoint'");
  Serial.println(" 4. Click 'Enable Robot'");
  Serial.println(" 5. Tune PID per the guide on the page");
  xTaskCreatePinnedToCore(pidTask, "PID", 4096, NULL,
                          /*priority*/ 2, &pidTaskHandle, /*core*/ 1);
  testMotorsIsolated();
}

// =========================================================
//  SECTION 11: MAIN CONTROL LOOP
// =========================================================

void loop() {
  webServer.handleClient();
  vTaskDelay(1);   // yield to RTOS, WiFi runs freely on Core 0
}
// void loop() { //testing
//     digitalWrite(PIN_STBY, HIGH);

//     digitalWrite(PIN_AIN1, HIGH);
//     digitalWrite(PIN_AIN2, LOW);
//     ledcWrite(PIN_PWMA, 255);

//     digitalWrite(PIN_BIN1, HIGH);
//     digitalWrite(PIN_BIN2, LOW);
//     ledcWrite(PIN_PWMB, 255);

//     delay(1000);
// }