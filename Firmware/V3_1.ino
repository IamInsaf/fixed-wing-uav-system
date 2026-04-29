// ================================================================
//  FIXED WING UAV — FLIGHT CONTROLLER v3.0
//  ESP32 Dev Module
// ================================================================
//
//  MODES (CH5)
//    CH5 < 1500   → MANUAL   — pilot controls everything
//    CH5 = 1500   → MISSION  — follows GPS waypoints
//    CH5 > 1500   → RTH      — fly home, orbit above home point
//
//  RTH AUTO-TRIGGERS
//    RC signal lost > 200ms
//    Battery voltage < 10.5V (3S cutoff)
//
//  RTH BEHAVIOUR
//    1. Climb to safe altitude (30m) if below it
//    2. Fly toward home GPS coordinates
//    3. Within 30m of home → start orbiting at 25° bank
//    4. Orbit indefinitely until pilot flips CH5 to manual
//
//  ARCHITECTURE
//  Core 0 (priority 24) — Control loop 500Hz
//    reads target_roll, target_pitch, target_throttle
//    runs stabilization PID against IMU
//    writes MCPWM outputs
//
//  Core 1 (priority 5) — Sensors + Guidance
//    iBUS, IMU, Baro, GPS, Battery
//    Mode manager (CH5)
//    Navigation (bearing, distance, heading error)
//    RTH state machine
//    Mission waypoint manager
//    RPi telemetry (fire and forget)
//
// ── PINOUT ───────────────────────────────────────────────────────
//  iBUS (FS-IA6B)    → GPIO16  (UART2 RX)
//  GPS TX            → GPIO32  (UART1 RX)
//  GPS RX            → GPIO33  (UART1 TX — optional)
//  IMU SDA           → GPIO21  (I2C)
//  IMU SCL           → GPIO22  (I2C)
//  Elevator servo    → GPIO13  (MCPWM Unit0 Timer0)
//  Aileron L servo   → GPIO14  (MCPWM Unit0 Timer1)
//  Aileron R servo   → GPIO27  (MCPWM Unit0 Timer2)
//  ESC left          → GPIO25  (MCPWM Unit1 Timer0)
//  ESC right         → GPIO26  (MCPWM Unit1 Timer1)
//  RPi UART TX       → GPIO1   (optional, fire-and-forget)
//  Battery voltage   → GPIO34  (ADC, 100k/47k divider)
//
// ── LIBRARIES ────────────────────────────────────────────────────
//  MPU6050_light   by rfetick     (Library Manager)
//  Adafruit BMP5xx by Adafruit    (Library Manager)
//  Adafruit BusIO  by Adafruit    (auto-installed)
//  TinyGPSPlus     by Mikal Hart  (Library Manager)
//
// ================================================================

#include <Wire.h>
#include <HardwareSerial.h>
#include "driver/mcpwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_task_wdt.h"
#include <MPU6050_light.h>
#include <Adafruit_BMP5xx.h>
#include <TinyGPSPlus.h>
#include <math.h>

// =================================================================
//  CONFIGURATION — tune these for your airframe
// =================================================================

// ── IMU ───────────────────────────────────────────────────────────
#define COMP_ALPHA          0.98f   // complementary filter

// ── RC ────────────────────────────────────────────────────────────
#define RC_DEADZONE         10      // µs — ignore tiny stick movements
#define RC_TIMEOUT_MS       200     // ms — signal loss threshold

// ── AUTO MODE LIMITS ─────────────────────────────────────────────
// Maximum bank and pitch in autonomous modes
// Keep conservative — you can always increase later
#define AUTO_MAX_ROLL_DEG   30.0f   // max bank angle in auto
#define AUTO_MAX_PITCH_DEG  15.0f   // max pitch angle in auto
#define AUTO_CRUISE_THROTTLE 1500   // throttle in cruise (µs)
#define AUTO_CLIMB_THROTTLE  1700   // throttle when climbing
#define AUTO_MIN_THROTTLE    1300   // never go below this in auto

// ── RTH ───────────────────────────────────────────────────────────
#define RTH_SAFE_ALTITUDE   30.0f   // meters — climb to this before flying home
#define RTH_ORBIT_RADIUS    30.0f   // meters — start orbiting within this
#define RTH_ORBIT_BANK      25.0f   // degrees — bank angle during orbit
#define RTH_HOME_MIN_SATS   6       // minimum satellites to set home

// ── BATTERY ───────────────────────────────────────────────────────
#define VBAT_RTH_TRIGGER    10.5f   // volts — auto RTH below this (3S)
#define VDIV_RATIO          (147.0f / 47.0f)
#define ADC_REF             3.3f
#define ADC_RESOLUTION      4095.0f

// ── LOOP RATES ────────────────────────────────────────────────────
#define CONTROL_HZ          500
#define CONTROL_PERIOD_MS   (1000 / CONTROL_HZ)
#define IMU_READ_MS         2
#define BARO_READ_MS        20
#define GPS_PARSE_MS        1
#define NAV_UPDATE_MS       200     // navigation runs at 5Hz
#define RPI_SEND_MS         10
#define DEBUG_PRINT_MS      500

// ── SERIAL ────────────────────────────────────────────────────────
#define SERIAL_BAUD         500000
#define GPS_BAUD            9600

// =================================================================
//  PINS
// =================================================================
#define IBUS_RX_PIN         16
#define IBUS_TX_PIN         17
#define GPS_RX_PIN          32
#define GPS_TX_PIN          33
#define PIN_SERVO_PITCH     13
#define PIN_SERVO_AIL_L     14
#define PIN_SERVO_AIL_R     27
#define PIN_ESC_LEFT        25
#define PIN_ESC_RIGHT       26
#define I2C_SDA             21
#define I2C_SCL             22
#define PIN_VBAT            34
#define RPI_TX_PIN          1
#define RPI_RX_PIN          3

// =================================================================
//  FLIGHT MODES
// =================================================================
typedef enum {
  MODE_MANUAL  = 0,
  MODE_MISSION = 1,
  MODE_RTH     = 2
} FlightMode;

// RTH sub-states
typedef enum {
  RTH_CLIMB = 0,   // climbing to safe altitude
  RTH_FLY   = 1,   // flying toward home
  RTH_ORBIT = 2    // orbiting above home
} RTHState;

// =================================================================
//  WAYPOINTS — define your mission here
//  Add as many as needed, plane visits them in order
// =================================================================
struct Waypoint {
  float lat;
  float lon;
  float alt_m;      // target altitude at this waypoint
};

// ── Edit these to your actual waypoints ──────────────────────────
// Get coordinates from Google Maps — right click → copy coordinates
const Waypoint WAYPOINTS[] = {
  { 9.4030f, 76.3530f, 40.0f },   // WP1
  { 9.4040f, 76.3540f, 40.0f },   // WP2
  { 9.4050f, 76.3530f, 40.0f },   // WP3
};
const int WAYPOINT_COUNT = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
#define WAYPOINT_ARRIVAL_RADIUS  20.0f  // meters — switch to next WP

// =================================================================
//  PID CONTROLLER — reusable class
// =================================================================
class PID {
public:
  float kp, ki, kd;
  float outMin, outMax;
  float integral;
  float lastError;
  uint32_t lastTime;

  PID(float p, float i, float d, float mn, float mx)
    : kp(p), ki(i), kd(d), outMin(mn), outMax(mx),
      integral(0), lastError(0), lastTime(0) {}

  float compute(float error) {
    uint32_t now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f; // clamp dt

    integral  += error * dt;
    integral   = constrain(integral, outMin / ki, outMax / ki);

    float derivative = (error - lastError) / dt;
    lastError = error;

    float out = kp * error + ki * integral + kd * derivative;
    return constrain(out, outMin, outMax);
  }

  void reset() {
    integral  = 0;
    lastError = 0;
    lastTime  = millis();
  }
};

// ── PID instances ─────────────────────────────────────────────────
// Roll stabilization — error in degrees → aileron correction (µs)
// Tune kp first. Start low, increase until responsive but not oscillating
PID rollPID(8.0f, 0.5f, 2.0f, -400.0f, 400.0f);

// Pitch stabilization — error in degrees → elevator correction (µs)
PID pitchPID(8.0f, 0.5f, 2.0f, -400.0f, 400.0f);

// Altitude hold — error in meters → pitch angle target (degrees)
PID altPID(2.0f, 0.1f, 1.0f, -10.0f, 10.0f);

// Heading / navigation — heading error (degrees) → roll target (degrees)
PID navPID(0.5f, 0.02f, 0.1f, -AUTO_MAX_ROLL_DEG, AUTO_MAX_ROLL_DEG);

// =================================================================
//  SHARED VOLATILE STATE
//  Written by Core 1, read by Core 0
// =================================================================

// RC raw channels
volatile uint16_t ch_roll      = 1500;
volatile uint16_t ch_pitch     = 1500;
volatile uint16_t ch_throttle  = 1000;
volatile uint16_t ch_mode      = 1000;  // CH5 — mode switch
volatile bool     rcConnected  = false;
volatile uint32_t lastGoodPacket = 0;

// IMU
volatile float imu_roll        = 0.0f;
volatile float imu_pitch       = 0.0f;
volatile float imu_yaw         = 0.0f;
volatile bool  imuReady        = false;

// Barometer
volatile float altitude_m      = 0.0f;
volatile float pressure_hpa    = 0.0f;
volatile float baro_temp_c     = 0.0f;
volatile bool  baroReady       = false;

// GPS
volatile float  gps_lat        = 0.0f;
volatile float  gps_lon        = 0.0f;
volatile float  gps_alt        = 0.0f;
volatile float  gps_course     = 0.0f;  // degrees, direction of travel
volatile float  gps_speed_mps  = 0.0f;
volatile uint8_t gps_sats      = 0;
volatile bool   gps_fix        = false;
volatile bool   gpsReady       = false;

// Home position
volatile float  home_lat       = 0.0f;
volatile float  home_lon       = 0.0f;
volatile float  home_alt       = 0.0f;
volatile bool   homeSet        = false;

// Battery
volatile float  vbat           = 0.0f;
volatile bool   lowBattery     = false;

// Mode
volatile FlightMode currentMode = MODE_MANUAL;
volatile RTHState   rthState    = RTH_CLIMB;
volatile bool       autoRTH     = false;  // set true when auto-triggered

// Autonomous targets — written by Core 1 guidance, read by Core 0
// In manual mode these are set directly from RC sticks
// In auto modes these are set by navigation/RTH guidance
volatile float   target_roll_deg   = 0.0f;
volatile float   target_pitch_deg  = 0.0f;
volatile uint16_t target_throttle  = 1000;

// Mission state
volatile int  currentWaypoint = 0;
volatile float dist_to_wp     = 0.0f;
volatile float bearing_to_wp  = 0.0f;

// Output state (for telemetry)
volatile uint32_t out_pitch   = 1500;
volatile uint32_t out_ail_l   = 1500;
volatile uint32_t out_ail_r   = 1500;
volatile uint32_t out_esc_l   = 1000;
volatile uint32_t out_esc_r   = 1000;

// =================================================================
//  iBUS PARSER
// =================================================================
HardwareSerial ibusSerial(2);
#define IBUS_BUFFSIZE  32
#define UART_BUF_SIZE  512
uint8_t  ibusBuffer[IBUS_BUFFSIZE];
uint16_t ibusChannels[14];
uint8_t  ibusIdx      = 0;
uint32_t ibusLastByte = 0;

void parseIBUS() {
  while (ibusSerial.available()) {
    uint8_t  b   = ibusSerial.read();
    uint32_t now = micros();

    if (now - ibusLastByte > 3000) ibusIdx = 0;
    ibusLastByte = now;

    ibusBuffer[ibusIdx++] = b;
    if (ibusIdx < IBUS_BUFFSIZE) continue;
    ibusIdx = 0;

    if (ibusBuffer[0] != 0x20 || ibusBuffer[1] != 0x40) continue;

    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < 30; i++) checksum -= ibusBuffer[i];
    if (checksum != (uint16_t)(ibusBuffer[30] | ibusBuffer[31] << 8)) continue;

    for (int i = 0; i < 14; i++)
      ibusChannels[i] = ibusBuffer[2 + i * 2] | (ibusBuffer[3 + i * 2] << 8);

    ch_roll       = ibusChannels[0];
    ch_pitch      = ibusChannels[1];
    ch_throttle   = ibusChannels[2];
    ch_mode       = ibusChannels[4];  // CH5
    lastGoodPacket = millis();
    rcConnected   = true;
  }
}

// =================================================================
//  MCPWM — direct hardware PWM
// =================================================================
void pwm_us(mcpwm_unit_t unit, mcpwm_timer_t timer,
            mcpwm_generator_t gen, uint32_t us) {
  mcpwm_set_duty_in_us(unit, timer, gen, us);
}

void mcpwm_setup() {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_SERVO_PITCH);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_SERVO_AIL_L);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PIN_SERVO_AIL_R);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, PIN_ESC_LEFT);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, PIN_ESC_RIGHT);

  mcpwm_config_t cfg;
  cfg.frequency    = 50;
  cfg.cmpr_a       = 0;
  cfg.cmpr_b       = 0;
  cfg.duty_mode    = MCPWM_DUTY_MODE_0;
  cfg.counter_mode = MCPWM_UP_COUNTER;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &cfg);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &cfg);
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &cfg);
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &cfg);

  pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1500);
  pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 1500);
  pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 1500);
  pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
  pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 1000);
}

// =================================================================
//  FAILSAFE
// =================================================================
void applyFailsafe() {
  pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1500);
  pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 1500);
  pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 1500);
  pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
  pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 1000);
}

// =================================================================
//  NAVIGATION MATHS
// =================================================================

// Bearing from current position to target (0-360 degrees)
float calcBearing(float curLat, float curLon,
                  float tgtLat, float tgtLon) {
  float dLon = radians(tgtLon - curLon);
  float lat1 = radians(curLat);
  float lat2 = radians(tgtLat);
  float x    = sin(dLon) * cos(lat2);
  float y    = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  float brng = degrees(atan2(x, y));
  return fmod(brng + 360.0f, 360.0f);
}

// Distance in meters between two GPS coordinates
float calcDistance(float curLat, float curLon,
                   float tgtLat, float tgtLon) {
  const float R = 6371000.0f;
  float dLat = radians(tgtLat - curLat);
  float dLon = radians(tgtLon - curLon);
  float a    = sin(dLat / 2) * sin(dLat / 2) +
               cos(radians(curLat)) * cos(radians(tgtLat)) *
               sin(dLon / 2) * sin(dLon / 2);
  float c    = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
  return R * c;
}

// Heading error — shortest turn from current course to target bearing
// Positive = turn right, negative = turn left
float calcHeadingError(float currentCourse, float targetBearing) {
  float error = targetBearing - currentCourse;
  if (error >  180.0f) error -= 360.0f;
  if (error < -180.0f) error += 360.0f;
  return error;
}

// =================================================================
//  IMU
// =================================================================
MPU6050 mpu(Wire);
float   cf_roll  = 0.0f;
float   cf_pitch = 0.0f;
float   cf_yaw   = 0.0f;
uint32_t imuLastUs = 0;

void readIMU() {
  mpu.update();
  uint32_t now = micros();
  float dt = (now - imuLastUs) / 1000000.0f;
  imuLastUs = now;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.002f;

  cf_roll  = COMP_ALPHA * (cf_roll  + mpu.getGyroX() * dt)
           + (1.0f - COMP_ALPHA) * mpu.getAngleX();
  cf_pitch = COMP_ALPHA * (cf_pitch + mpu.getGyroY() * dt)
           + (1.0f - COMP_ALPHA) * mpu.getAngleY();
  cf_yaw  += mpu.getGyroZ() * dt;
  if (cf_yaw >  180.0f) cf_yaw -= 360.0f;
  if (cf_yaw < -180.0f) cf_yaw += 360.0f;

  imu_roll  = cf_roll;
  imu_pitch = cf_pitch;
  imu_yaw   = cf_yaw;
  imuReady  = true;
}

// =================================================================
//  BAROMETER
// =================================================================
Adafruit_BMP5xx bmp;
float seaLevelPressure = 1013.25f;
float baroHomeAlt      = 0.0f;    // altitude at home — set when home is locked
float relativeAlt      = 0.0f;    // altitude above home

void readBaro() {
  if (bmp.performReading()) {
    pressure_hpa = bmp.pressure;
    baro_temp_c  = bmp.temperature;
    altitude_m   = bmp.readAltitude(seaLevelPressure);
    relativeAlt  = altitude_m - baroHomeAlt;
    baroReady    = true;
  }
}

// =================================================================
//  GPS
// =================================================================
HardwareSerial gpsSerial(1);
TinyGPSPlus    gps;

void readGPS() {
  while (gpsSerial.available())
    gps.encode(gpsSerial.read());

  if (gps.location.isValid() && gps.location.age() < 2000) {
    gps_lat = gps.location.lat();
    gps_lon = gps.location.lng();
    gps_fix = true;
  } else {
    gps_fix = false;
  }

  if (gps.altitude.isValid())
    gps_alt = gps.altitude.meters();

  if (gps.course.isValid() && gps.speed.mps() > 0.5f)
    gps_course = gps.course.deg();

  if (gps.speed.isValid())
    gps_speed_mps = gps.speed.mps();

  if (gps.satellites.isValid())
    gps_sats = gps.satellites.value();

  // Set home on first good fix with enough satellites
  if (!homeSet && gps_fix && gps_sats >= RTH_HOME_MIN_SATS) {
    home_lat    = gps_lat;
    home_lon    = gps_lon;
    home_alt    = gps_alt;
    baroHomeAlt = altitude_m;  // snapshot baro altitude as home reference
    homeSet     = true;
    Serial.printf("[NAV] Home set: %.6f, %.6f  Alt:%.1fm  Sats:%d\n",
                  (float)home_lat, (float)home_lon,
                  (float)home_alt, (int)gps_sats);
  }
  gpsReady = true;
}

// =================================================================
//  BATTERY
// =================================================================
void readBattery() {
  int raw   = analogRead(PIN_VBAT);
  float adc = (raw / ADC_RESOLUTION) * ADC_REF;
  vbat      = adc * VDIV_RATIO;
  lowBattery = (vbat > 0.5f && vbat < VBAT_RTH_TRIGGER);
}

// =================================================================
//  MODE MANAGER
//  Reads CH5, handles auto-RTH triggers
//  Returns the active FlightMode
// =================================================================
FlightMode updateMode() {
  uint16_t ch5 = ch_mode;

  // ── Safety override — auto RTH conditions ─────────────────────
  bool rcLost   = !rcConnected;
  bool batLow   = lowBattery;

  if (rcLost || batLow) {
    if (!autoRTH) {
      autoRTH = true;
      rthState = RTH_CLIMB;
      rollPID.reset();
      pitchPID.reset();
      altPID.reset();
      navPID.reset();
      if (rcLost) Serial.println("[MODE] AUTO RTH — RC LOST");
      if (batLow) Serial.printf("[MODE] AUTO RTH — LOW BATTERY %.2fV\n",
                                (float)vbat);
    }
    return MODE_RTH;
  }

  // ── RC restored — clear auto RTH ──────────────────────────────
  autoRTH = false;

  // ── CH5 based mode ────────────────────────────────────────────
  FlightMode newMode;
  if      (ch5 < 1400) newMode = MODE_MANUAL;
  else if (ch5 < 1600) newMode = MODE_MISSION;
  else                 newMode = MODE_RTH;

  // Reset PIDs on mode transition
  if (newMode != currentMode) {
    rollPID.reset();
    pitchPID.reset();
    altPID.reset();
    navPID.reset();
    if (newMode == MODE_RTH)     rthState = RTH_CLIMB;
    if (newMode == MODE_MISSION) currentWaypoint = 0;
    Serial.printf("[MODE] → %s\n",
      newMode == MODE_MANUAL  ? "MANUAL" :
      newMode == MODE_MISSION ? "MISSION" : "RTH");
  }

  return newMode;
}

// =================================================================
//  RTH GUIDANCE
//  Updates target_roll_deg, target_pitch_deg, target_throttle
// =================================================================
void runRTH() {
  float curLat = gps_lat;
  float curLon = gps_lon;
  float curAlt = relativeAlt;  // altitude above home

  // If no GPS fix — hold wings level, maintain altitude, hope for fix
  if (!gps_fix) {
    target_roll_deg  = 0.0f;
    target_pitch_deg = 0.0f;
    target_throttle  = AUTO_CRUISE_THROTTLE;
    return;
  }

  float distHome    = calcDistance(curLat, curLon, home_lat, home_lon);
  float bearingHome = calcBearing(curLat, curLon, home_lat, home_lon);

  // ── Step 1: Climb to safe altitude ────────────────────────────
  if (rthState == RTH_CLIMB) {
    if (curAlt < RTH_SAFE_ALTITUDE - 2.0f) {
      // Pitch up and add throttle to climb
      float altError       = RTH_SAFE_ALTITUDE - curAlt;
      target_pitch_deg     = constrain(altError * 0.5f, 0.0f, AUTO_MAX_PITCH_DEG);
      target_roll_deg      = 0.0f;  // wings level during climb
      target_throttle      = AUTO_CLIMB_THROTTLE;
    } else {
      rthState = RTH_FLY;
      Serial.printf("[RTH] Altitude reached %.1fm — flying home\n", curAlt);
    }
  }

  // ── Step 2: Fly toward home ────────────────────────────────────
  if (rthState == RTH_FLY) {
    if (distHome > RTH_ORBIT_RADIUS) {
      // Navigation: heading error → roll command
      float headErr        = calcHeadingError(gps_course, bearingHome);
      target_roll_deg      = navPID.compute(headErr);

      // Altitude hold during transit
      float altError       = RTH_SAFE_ALTITUDE - curAlt;
      target_pitch_deg     = altPID.compute(altError);
      target_throttle      = AUTO_CRUISE_THROTTLE;
    } else {
      rthState = RTH_ORBIT;
      Serial.printf("[RTH] Home reached — starting orbit %.1fm away\n",
                    distHome);
    }
  }

  // ── Step 3: Orbit above home ───────────────────────────────────
  if (rthState == RTH_ORBIT) {
    // Orbit = fly perpendicular to bearing home + constant bank
    // Add 90 degrees to bearing to fly a circle around home
    float orbitBearing   = fmod(bearingHome + 90.0f, 360.0f);
    float orbitHeadErr   = calcHeadingError(gps_course, orbitBearing);

    // Bank proportional to heading error, clamp to orbit bank angle
    target_roll_deg      = constrain(
                             navPID.compute(orbitHeadErr),
                             -RTH_ORBIT_BANK,
                              RTH_ORBIT_BANK);

    // Hold altitude
    float altError       = RTH_SAFE_ALTITUDE - curAlt;
    target_pitch_deg     = altPID.compute(altError);
    target_throttle      = AUTO_CRUISE_THROTTLE;
  }
}

// =================================================================
//  MISSION GUIDANCE
//  Flies through waypoints in order
// =================================================================
void runMission() {
  if (!gps_fix || currentWaypoint >= WAYPOINT_COUNT) {
    // No fix or mission complete — hold wings level
    target_roll_deg  = 0.0f;
    target_pitch_deg = 0.0f;
    target_throttle  = AUTO_CRUISE_THROTTLE;
    return;
  }

  float curLat = gps_lat;
  float curLon = gps_lon;
  float curAlt = relativeAlt;

  Waypoint wp  = WAYPOINTS[currentWaypoint];

  float dist   = calcDistance(curLat, curLon, wp.lat, wp.lon);
  float brng   = calcBearing(curLat, curLon, wp.lat, wp.lon);

  // Update shared state for telemetry
  dist_to_wp   = dist;
  bearing_to_wp = brng;

  // Arrived at waypoint?
  if (dist < WAYPOINT_ARRIVAL_RADIUS) {
    Serial.printf("[MISSION] WP%d reached — %.1fm away\n",
                  currentWaypoint + 1, dist);
    currentWaypoint++;

    if (currentWaypoint >= WAYPOINT_COUNT) {
      Serial.println("[MISSION] All waypoints complete — switching to RTH");
      // Auto switch to RTH when mission is done
      // CH5 stays in mission position but guidance goes RTH
      rthState = RTH_CLIMB;
      return;
    }
    Serial.printf("[MISSION] Next: WP%d\n", currentWaypoint + 1);
    navPID.reset();
    return;
  }

  // Navigate to current waypoint
  float headErr        = calcHeadingError(gps_course, brng);
  target_roll_deg      = navPID.compute(headErr);

  // Altitude hold at waypoint target altitude
  float altError       = wp.alt_m - curAlt;
  target_pitch_deg     = altPID.compute(altError);
  target_throttle      = AUTO_CRUISE_THROTTLE;
}

// =================================================================
//  CORE 0 — CONTROL TASK (500Hz, untouchable)
//
//  Reads target_roll_deg and target_pitch_deg (set by guidance)
//  Runs PID against IMU actual angles
//  Writes MCPWM outputs
// =================================================================
void controlTask(void* param) {
  esp_task_wdt_delete(NULL);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

  // ESC arming
  uint32_t armStart = millis();
  while (millis() - armStart < 3000) {
    pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
    pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 1000);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  Serial.println("[CTRL] ESCs armed. Control loop running.");

  for (;;) {
    // ── Signal loss → failsafe ────────────────────────────────
    if (rcConnected && millis() - lastGoodPacket > RC_TIMEOUT_MS) {
      rcConnected = false;
    }

    // Hard failsafe — only if RC lost AND no home set AND not in RTH
    // If home is set, RTH handles it gracefully via mode manager
    if (!rcConnected && !homeSet) {
      applyFailsafe();
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
      continue;
    }

    // ── Snapshot shared state once ────────────────────────────
    FlightMode mode     = currentMode;
    float      tgtRoll  = target_roll_deg;
    float      tgtPitch = target_pitch_deg;
    uint16_t   tgtThr   = target_throttle;
    float      actRoll  = imu_roll;
    float      actPitch = imu_pitch;

    uint32_t p, al, ar, e;

    if (mode == MODE_MANUAL) {
      // ── MANUAL — passthrough with deadzone ───────────────────
      uint16_t roll    = ch_roll;
      uint16_t pitch   = ch_pitch;
      uint16_t throttle = ch_throttle;

      p  = constrain(pitch,    1000, 2000);
      if (p > 1500 - RC_DEADZONE && p < 1500 + RC_DEADZONE) p = 1500;

      al = constrain(roll,     1000, 2000);
      if (al > 1500 - RC_DEADZONE && al < 1500 + RC_DEADZONE) al = 1500;
      ar = 3000 - al;  // aileron R always opposite of L

      e  = constrain(throttle, 1000, 2000);

    } else {
      // ── AUTO (MISSION or RTH) — stabilized PID ───────────────
      // Roll PID: target angle vs actual angle → aileron correction
      float rollError  = tgtRoll - actRoll;
      float rollCorr   = rollPID.compute(rollError);

      // Pitch PID: target angle vs actual angle → elevator correction
      float pitchError = tgtPitch - actPitch;
      float pitchCorr  = pitchPID.compute(pitchError);

      // Center (1500) ± PID correction
      al = (uint32_t)constrain(1500 + rollCorr,  1000, 2000);
      ar = (uint32_t)constrain(1500 - rollCorr,  1000, 2000); // opposite
      p  = (uint32_t)constrain(1500 + pitchCorr, 1000, 2000);
      e  = constrain((uint32_t)tgtThr, AUTO_MIN_THROTTLE, 2000);
    }

    // ── Write to hardware ─────────────────────────────────────
    pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, p);
    pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, al);
    pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, ar);
    pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, e);
    pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, e);

    out_pitch = p;
    out_ail_l = al;
    out_ail_r = ar;
    out_esc_l = e;
    out_esc_r = e;

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// =================================================================
//  CORE 1 — SENSORS + GUIDANCE TASK
// =================================================================
void commsTask(void* param) {
  esp_task_wdt_delete(NULL);

  uint32_t lastImuRead  = 0;
  uint32_t lastBaroRead = 0;
  uint32_t lastBatRead  = 0;
  uint32_t lastNavUpdate = 0;
  uint32_t lastRPiSend  = 0;
  uint32_t lastPrint    = 0;

  for (;;) {
    uint32_t now = millis();

    // ── iBUS — drain every iteration ─────────────────────────
    parseIBUS();

    // ── GPS — drain every iteration ──────────────────────────
    readGPS();

    // ── IMU — every 2ms ──────────────────────────────────────
    if (now - lastImuRead >= IMU_READ_MS) {
      lastImuRead = now;
      readIMU();
    }

    // ── Barometer — every 20ms ────────────────────────────────
    if (now - lastBaroRead >= BARO_READ_MS) {
      lastBaroRead = now;
      readBaro();
    }

    // ── Battery — every 1 second ──────────────────────────────
    if (now - lastBatRead >= 1000) {
      lastBatRead = now;
      readBattery();
    }

    // ── Navigation / guidance — every 200ms ───────────────────
    if (now - lastNavUpdate >= NAV_UPDATE_MS) {
      lastNavUpdate = now;

      // Determine mode
      currentMode = updateMode();

      // Run appropriate guidance
      switch (currentMode) {
        case MODE_MANUAL:
          // Manual — targets set directly from RC in Core 0
          // Nothing to compute here
          break;

        case MODE_MISSION:
          runMission();
          break;

        case MODE_RTH:
          runRTH();
          break;
      }
    }

    // ── RPi telemetry — every 10ms ────────────────────────────
    if (now - lastRPiSend >= RPI_SEND_MS) {
      lastRPiSend = now;
      // sendToRPi(); // uncomment when RPi connected
    }

    // ── Serial debug — every 500ms ────────────────────────────
    if (now - lastPrint >= DEBUG_PRINT_MS) {
      lastPrint = now;
      printDebug();
    }

    vTaskDelay(1);
  }
}

// =================================================================
//  DEBUG PRINT
// =================================================================
void printDebug() {
  const char* modeStr =
    currentMode == MODE_MANUAL  ? "MANUAL " :
    currentMode == MODE_MISSION ? "MISSION" : "RTH    ";

  const char* rthStr =
    rthState == RTH_CLIMB ? "CLIMB" :
    rthState == RTH_FLY   ? "FLY  " : "ORBIT";

  Serial.println("─────────────────────────────────────────────");
  Serial.printf("MODE  %s  %s\n",
    modeStr,
    currentMode == MODE_RTH ? rthStr : "");

  Serial.printf("RC    %s  Roll:%4d  Pitch:%4d  Thr:%4d  CH5:%4d\n",
    rcConnected ? "[OK]  " : "[LOST]",
    (int)ch_roll, (int)ch_pitch, (int)ch_throttle, (int)ch_mode);

  if (imuReady)
    Serial.printf("IMU   Roll:%6.1f  Pitch:%6.1f  Yaw:%6.1f  deg\n",
      (float)imu_roll, (float)imu_pitch, (float)imu_yaw);

  if (baroReady)
    Serial.printf("BARO  Alt:%6.1fm  Rel:%5.1fm  Temp:%.1fC\n",
      (float)altitude_m, relativeAlt, (float)baro_temp_c);

  if (gps_fix)
    Serial.printf("GPS   Fix:YES  Sats:%2d  Lat:%.6f  Lon:%.6f  Crs:%.1f\n",
      (int)gps_sats, (float)gps_lat, (float)gps_lon, (float)gps_course);
  else
    Serial.printf("GPS   Fix:NO   Sats:%2d\n", (int)gps_sats);

  Serial.printf("HOME  %s  Lat:%.6f  Lon:%.6f\n",
    homeSet ? "SET  " : "NOTSET",
    (float)home_lat, (float)home_lon);

  Serial.printf("VBAT  %.2fV  %s\n",
    (float)vbat,
    lowBattery ? "LOW BATTERY — RTH" : "OK");

  if (currentMode != MODE_MANUAL) {
    Serial.printf("TGT   Roll:%5.1f  Pitch:%5.1f  Thr:%4d\n",
      (float)target_roll_deg, (float)target_pitch_deg,
      (int)target_throttle);
    if (currentMode == MODE_MISSION && currentWaypoint < WAYPOINT_COUNT)
      Serial.printf("WP    %d/%d  Dist:%.1fm  Bearing:%.1fdeg\n",
        currentWaypoint + 1, WAYPOINT_COUNT,
        (float)dist_to_wp, (float)bearing_to_wp);
  }
}

// =================================================================
//  SENSOR INIT
// =================================================================
bool initIMU() {
  byte status = mpu.begin();
  if (status != 0) {
    Serial.printf("[IMU] Failed, status=%d\n", status);
    return false;
  }
  Serial.println("[IMU] Calibrating — keep UAV still...");
  delay(3000);
  mpu.calcOffsets(true, true);
  imuLastUs = micros();
  Serial.println("[IMU] Ready.");
  return true;
}

bool initBaro() {
  if (!bmp.begin_I2C()) {
    Serial.println("[BARO] Not found — check wiring");
    return false;
  }
  Serial.println("[BARO] Ready.");
  return true;
}

void initGPS() {
  gpsSerial.setRxBufferSize(512);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[GPS] UART started on GPIO32");
}

// =================================================================
//  SETUP
// =================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println("\n================================================");
  Serial.println("  Fixed Wing UAV — Flight Controller v3.0");
  Serial.println("  Manual | Mission | RTH");
  Serial.println("================================================\n");

  // Kill WiFi + BT
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  Serial.println("[OK] WiFi + BT disabled");

  esp_task_wdt_deinit();
  Serial.println("[OK] Watchdog disabled");

  // Safe defaults
  for (int i = 0; i < 14; i++) ibusChannels[i] = 1500;
  ibusChannels[2] = 1000;

  // iBUS
  ibusSerial.setRxBufferSize(UART_BUF_SIZE);
  ibusSerial.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);
  Serial.println("[OK] iBUS on GPIO16");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Serial.println("[OK] I2C at 400kHz");

  // Battery ADC
  pinMode(PIN_VBAT, INPUT);
  analogReadResolution(12);
  Serial.println("[OK] Battery ADC on GPIO34");

  // Sensors
  bool imuOk  = initIMU();
  bool baroOk = initBaro();
  initGPS();

  if (!imuOk)  Serial.println("[WARN] No IMU — auto modes disabled");
  if (!baroOk) Serial.println("[WARN] No barometer — altitude hold disabled");

  Serial.println("[OK] Waiting for GPS fix to set home...");
  Serial.println("[OK] CH5 < 1400 = MANUAL | ~1500 = MISSION | > 1600 = RTH");

  // MCPWM
  mcpwm_setup();
  Serial.println("[OK] MCPWM configured");

  Serial.println("\n[OK] Launching tasks...\n");

  xTaskCreatePinnedToCore(controlTask, "ctrl",  8192, NULL, 24, NULL, 0);
  xTaskCreatePinnedToCore(commsTask,  "comms", 8192, NULL,  5, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}
