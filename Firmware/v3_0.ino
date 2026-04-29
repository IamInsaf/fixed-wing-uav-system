// ================================================================
//  FIXED WING UAV — FLIGHT CONTROLLER v3.1
//  ESP32 Dev Module
// ================================================================
//
//  MODES (CH5)
//    CH5 < 1400   → MANUAL   — pilot controls everything
//    CH5 1400-1600 → MISSION  — follows GPS waypoints
//    CH5 > 1600   → RTH      — fly home, orbit above home point
//
//  YAW
//    CH4 stick → differential thrust (manual mode only)
//    RTH and Mission — no yaw mix, equal throttle both motors
//
//  RTH AUTO-TRIGGERS
//    RC signal lost > 200ms
//    Battery voltage < 10.5V
//
//  LED (GPIO2 — single LED)
//    Slow blink  (1×)  = Manual
//    Double blink (2×) = Mission
//    Triple blink (3×) = RTH flying home
//    Solid on          = RTH orbiting
//    Fast blink        = RC lost / failsafe
//    SOS pattern       = Low battery
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
//  LED indicator     → GPIO2
//  RPi UART TX       → GPIO1   (optional, fire-and-forget)
//  Battery voltage   → GPIO34  (ADC, 100k/47k divider)
//
// ── WIRING ───────────────────────────────────────────────────────
//  FS-IA6B VCC    → 5V BEC rail     FS-IA6B GND   → GND
//  FS-IA6B iBUS   → GPIO16
//  MPU6050 VCC    → ESP32 3V3       MPU6050 GND   → GND
//  MPU6050 SDA    → GPIO21          MPU6050 SCL   → GPIO22
//  MPU6050 ADO    → GND             (sets addr 0x68)
//  BMP581 VIN     → ESP32 3V3       BMP581 GND    → GND
//  BMP581 SDI     → GPIO21          BMP581 SCK    → GPIO22
//  BMP581 SDO     → GND             (sets addr 0x76)
//  BMP581 CS      → 3V3             (enables I2C mode)
//  NEO-6M VCC     → 5V or 3V3      NEO-6M GND    → GND
//  NEO-6M TX      → GPIO32
//  LED anode      → GPIO2 → 330Ω → LED → GND
//  Battery divider: LiPo+ → 100kΩ → GPIO34 → 47kΩ → GND
//
// ── LIBRARIES ────────────────────────────────────────────────────
//  MPU6050_light   by rfetick     (Library Manager)
//  Adafruit BMP5xx by Adafruit    (Library Manager)
//  Adafruit BusIO  by Adafruit    (auto-installed)
//  TinyGPSPlus     by Mikal Hart  (Library Manager)
//
// ── POWER ────────────────────────────────────────────────────────
//  LiPo → ESC left + right (direct, thick wire)
//  LiPo → ESC left BEC (5V/2A) → ESP32 VIN, FS-IA6B, Servos ×3
//  LiPo → UBEC (5V/3A) → Raspberry Pi only
//  ESC right BEC red wire → CUT (not needed)
//  ESP32 3V3 → MPU6050, BMP581, NEO-6M
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
//  CONFIGURATION
// =================================================================

// ── IMU ───────────────────────────────────────────────────────────
#define COMP_ALPHA            0.98f

// ── RC ────────────────────────────────────────────────────────────
#define RC_DEADZONE           10
#define RC_TIMEOUT_MS         200

// ── DIFFERENTIAL THRUST YAW ───────────────────────────────────────
// Max µs difference between left and right motor
// Start at 100, increase if yaw feels weak
#define YAW_MIX               100

// ── AUTO MODE LIMITS ──────────────────────────────────────────────
#define AUTO_MAX_ROLL_DEG     30.0f
#define AUTO_MAX_PITCH_DEG    15.0f
#define AUTO_CRUISE_THROTTLE  1500
#define AUTO_CLIMB_THROTTLE   1700
#define AUTO_MIN_THROTTLE     1300

// ── RTH ───────────────────────────────────────────────────────────
#define RTH_SAFE_ALTITUDE     30.0f
#define RTH_ORBIT_RADIUS      30.0f
#define RTH_ORBIT_BANK        25.0f
#define RTH_HOME_MIN_SATS     6

// ── BATTERY ───────────────────────────────────────────────────────
#define VBAT_RTH_TRIGGER      10.5f
#define VDIV_RATIO            (147.0f / 47.0f)
#define ADC_REF               3.3f
#define ADC_RESOLUTION        4095.0f

// ── LOOP RATES ────────────────────────────────────────────────────
#define CONTROL_HZ            500
#define CONTROL_PERIOD_MS     (1000 / CONTROL_HZ)
#define IMU_READ_MS           2
#define BARO_READ_MS          20
#define NAV_UPDATE_MS         200
#define RPI_SEND_MS           10
#define DEBUG_PRINT_MS        500

// ── SERIAL ────────────────────────────────────────────────────────
#define SERIAL_BAUD           500000
#define GPS_BAUD              9600

// =================================================================
//  PINS
// =================================================================
#define IBUS_RX_PIN           16
#define IBUS_TX_PIN           17
#define GPS_RX_PIN            32
#define GPS_TX_PIN            33
#define PIN_SERVO_PITCH       13
#define PIN_SERVO_AIL_L       14
#define PIN_SERVO_AIL_R       27
#define PIN_ESC_LEFT          25
#define PIN_ESC_RIGHT         26
#define PIN_LED               2
#define I2C_SDA               21
#define I2C_SCL               22
#define PIN_VBAT              34
#define RPI_TX_PIN            1
#define RPI_RX_PIN            3

// =================================================================
//  FLIGHT MODES
// =================================================================
typedef enum {
  MODE_MANUAL  = 0,
  MODE_MISSION = 1,
  MODE_RTH     = 2
} FlightMode;

typedef enum {
  RTH_CLIMB = 0,
  RTH_FLY   = 1,
  RTH_ORBIT = 2
} RTHState;

// =================================================================
//  WAYPOINTS
//  Edit these to your actual mission coordinates
//  Get from Google Maps: right click → copy coordinates
// =================================================================
struct Waypoint {
  float lat;
  float lon;
  float alt_m;
};

const Waypoint WAYPOINTS[] = {
  { 9.4030f, 76.3530f, 40.0f },
  { 9.4040f, 76.3540f, 40.0f },
  { 9.4050f, 76.3530f, 40.0f },
};
const int WAYPOINT_COUNT = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
#define WAYPOINT_ARRIVAL_RADIUS  20.0f

// =================================================================
//  PID CONTROLLER
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
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;

    integral += error * dt;
    integral  = constrain(integral, outMin / ki, outMax / ki);

    float derivative = (error - lastError) / dt;
    lastError = error;

    return constrain(kp * error + ki * integral + kd * derivative,
                     outMin, outMax);
  }

  void reset() {
    integral  = 0;
    lastError = 0;
    lastTime  = millis();
  }
};

// ── PID instances ─────────────────────────────────────────────────
PID rollPID (8.0f, 0.5f, 2.0f, -400.0f,  400.0f);
PID pitchPID(8.0f, 0.5f, 2.0f, -400.0f,  400.0f);
PID altPID  (2.0f, 0.1f, 1.0f,  -10.0f,   10.0f);
PID navPID  (0.5f, 0.02f, 0.1f, -AUTO_MAX_ROLL_DEG, AUTO_MAX_ROLL_DEG);

// =================================================================
//  SHARED VOLATILE STATE
// =================================================================

// RC channels
volatile uint16_t ch_roll      = 1500;
volatile uint16_t ch_pitch     = 1500;
volatile uint16_t ch_throttle  = 1000;
volatile uint16_t ch_yaw       = 1500;  // CH4 — differential thrust
volatile uint16_t ch_mode      = 1000;  // CH5 — flight mode
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
volatile float   gps_lat       = 0.0f;
volatile float   gps_lon       = 0.0f;
volatile float   gps_alt       = 0.0f;
volatile float   gps_course    = 0.0f;
volatile float   gps_speed_mps = 0.0f;
volatile uint8_t gps_sats      = 0;
volatile bool    gps_fix       = false;
volatile bool    gpsReady      = false;

// Home
volatile float home_lat        = 0.0f;
volatile float home_lon        = 0.0f;
volatile float home_alt        = 0.0f;
volatile bool  homeSet         = false;

// Battery
volatile float vbat            = 0.0f;
volatile bool  lowBattery      = false;

// Mode
volatile FlightMode currentMode = MODE_MANUAL;
volatile RTHState   rthState    = RTH_CLIMB;
volatile bool       autoRTH     = false;

// Autonomous targets (written by Core 1, read by Core 0)
volatile float   target_roll_deg  = 0.0f;
volatile float   target_pitch_deg = 0.0f;
volatile uint16_t target_throttle = 1000;

// Mission state
volatile int   currentWaypoint = 0;
volatile float dist_to_wp      = 0.0f;
volatile float bearing_to_wp   = 0.0f;

// Output state
volatile uint32_t out_pitch    = 1500;
volatile uint32_t out_ail_l    = 1500;
volatile uint32_t out_ail_r    = 1500;
volatile uint32_t out_esc_l    = 1000;
volatile uint32_t out_esc_r    = 1000;

// =================================================================
//  iBUS PARSER
// =================================================================
HardwareSerial ibusSerial(2);
#define IBUS_BUFFSIZE   32
#define UART_BUF_SIZE   512

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
    if (checksum != (uint16_t)(ibusBuffer[30] | ibusBuffer[31] << 8))
      continue;

    for (int i = 0; i < 14; i++)
      ibusChannels[i] = ibusBuffer[2 + i*2] | (ibusBuffer[3 + i*2] << 8);

    ch_roll       = ibusChannels[0];
    ch_pitch      = ibusChannels[1];
    ch_throttle   = ibusChannels[2];
    ch_yaw        = ibusChannels[3];  // CH4 — yaw / differential thrust
    ch_mode       = ibusChannels[4];  // CH5 — flight mode
    lastGoodPacket = millis();
    rcConnected   = true;
  }
}

// =================================================================
//  MCPWM
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
float calcBearing(float curLat, float curLon,
                  float tgtLat, float tgtLon) {
  float dLon = radians(tgtLon - curLon);
  float lat1 = radians(curLat);
  float lat2 = radians(tgtLat);
  float x    = sin(dLon) * cos(lat2);
  float y    = cos(lat1) * sin(lat2) -
               sin(lat1) * cos(lat2) * cos(dLon);
  return fmod(degrees(atan2(x, y)) + 360.0f, 360.0f);
}

float calcDistance(float curLat, float curLon,
                   float tgtLat, float tgtLon) {
  const float R = 6371000.0f;
  float dLat = radians(tgtLat - curLat);
  float dLon = radians(tgtLon - curLon);
  float a    = sin(dLat/2)*sin(dLat/2) +
               cos(radians(curLat)) * cos(radians(tgtLat)) *
               sin(dLon/2)*sin(dLon/2);
  return R * 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
}

float calcHeadingError(float currentCourse, float targetBearing) {
  float error = targetBearing - currentCourse;
  if (error >  180.0f) error -= 360.0f;
  if (error < -180.0f) error += 360.0f;
  return error;
}

// =================================================================
//  LED — single LED, blink patterns encode mode
// =================================================================
void initLED() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
}

void updateLED() {
  uint32_t now = millis();

  // Priority 1 — RC lost
  if (!rcConnected) {
    digitalWrite(PIN_LED, (now % 200) < 100);
    return;
  }

  // Priority 2 — low battery SOS
  // S=3 short(200ms) O=3 long(600ms) S=3 short, 600ms gap = 4000ms cycle
  if (lowBattery) {
    uint32_t t  = now % 4000;
    bool     on = false;
    if      (t < 200)                on = true;
    else if (t >= 400  && t < 600)   on = true;
    else if (t >= 800  && t < 1000)  on = true;
    else if (t >= 1200 && t < 1800)  on = true;
    else if (t >= 2000 && t < 2600)  on = true;
    else if (t >= 2800 && t < 3400)  on = true;
    digitalWrite(PIN_LED, on);
    return;
  }

  // Priority 3 — flight mode
  switch (currentMode) {

    case MODE_MANUAL:
      // Single slow blink — 200ms on, 1800ms off
      digitalWrite(PIN_LED, (now % 2000) < 200);
      break;

    case MODE_MISSION: {
      // Double blink — two 150ms flashes, long pause
      uint32_t t = now % 1500;
      digitalWrite(PIN_LED, (t < 150) || (t >= 300 && t < 450));
      break;
    }

    case MODE_RTH:
      if (rthState == RTH_ORBIT) {
        // Solid on — safely orbiting home
        digitalWrite(PIN_LED, HIGH);
      } else {
        // Triple blink — actively flying home
        uint32_t t = now % 2000;
        digitalWrite(PIN_LED,
          (t < 150) ||
          (t >= 300 && t < 450) ||
          (t >= 600 && t < 750));
      }
      break;
  }
}

// =================================================================
//  IMU
// =================================================================
MPU6050  mpu(Wire);
float    cf_roll  = 0.0f;
float    cf_pitch = 0.0f;
float    cf_yaw   = 0.0f;
uint32_t imuLastUs = 0;

void readIMU() {
  mpu.update();
  uint32_t now = micros();
  float dt = (now - imuLastUs) / 1000000.0f;
  imuLastUs = now;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.002f;

  // Roll and pitch — complementary filter (unchanged)
  cf_roll  = COMP_ALPHA * (cf_roll  + mpu.getGyroX() * dt)
           + (1.0f - COMP_ALPHA) * mpu.getAngleX();
  cf_pitch = COMP_ALPHA * (cf_pitch + mpu.getGyroY() * dt)
           + (1.0f - COMP_ALPHA) * mpu.getAngleY();

  // Yaw — motion gated integration
  // Only integrate when gyro Z reading exceeds noise threshold
  // This prevents tiny noise from accumulating into drift
  float gyroZ = mpu.getGyroZ();
  #define YAW_GYRO_THRESHOLD  0.3f   // deg/s — below this = noise, ignore
  if (fabsf(gyroZ) > YAW_GYRO_THRESHOLD) {
    cf_yaw += gyroZ * dt;
  }

  // Wrap yaw to -180..+180
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
float baroHomeAlt      = 0.0f;
float relativeAlt      = 0.0f;

void readBaro() {
 /** if (bmp.performReading()) {
    pressure_hpa = bmp.pressure / 100.0f;   // Pa → hPa
    baro_temp_c  = bmp.temperature;
    altitude_m   = bmp.readAltitude(seaLevelPressure);
    relativeAlt  = altitude_m - baroHomeAlt;
    baroReady    = true;
  } **/
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

  gpsReady = true;

  // Lock home on first good fix
  if (!homeSet && gps_fix && gps_sats >= RTH_HOME_MIN_SATS) {
    home_lat    = gps_lat;
    home_lon    = gps_lon;
    home_alt    = gps_alt;
    baroHomeAlt = altitude_m;
    homeSet     = true;
    Serial.printf("[NAV] Home locked: %.6f, %.6f  Alt:%.1fm  Sats:%d\n",
                  (float)home_lat, (float)home_lon,
                  (float)home_alt, (int)gps_sats);
  }
}

// =================================================================
//  BATTERY
// =================================================================
void readBattery() {
  vbat       = 12.0f;   // dummy value — voltage monitor disabled
  lowBattery = false;   // never trigger low battery RTH
}

// =================================================================
//  MODE MANAGER
// =================================================================
FlightMode updateMode() {
  bool rcLost = !rcConnected;
  bool batLow = lowBattery;

  // Auto RTH triggers
  if (rcLost || batLow) {
    if (!autoRTH) {
      autoRTH  = true;
      rthState = RTH_CLIMB;
      rollPID.reset(); pitchPID.reset();
      altPID.reset();  navPID.reset();
      if (rcLost) Serial.println("[MODE] AUTO RTH — RC LOST");
      if (batLow) Serial.printf("[MODE] AUTO RTH — LOW BATT %.2fV\n",
                                (float)vbat);
    }
    return MODE_RTH;
  }

  autoRTH = false;

  // CH5 mode selection
  FlightMode newMode;
  if      (ch_mode < 1400) newMode = MODE_MANUAL;
  else if (ch_mode < 1600) newMode = MODE_MISSION;
  else                     newMode = MODE_RTH;

  // Reset PIDs on mode change
  if (newMode != currentMode) {
    rollPID.reset(); pitchPID.reset();
    altPID.reset();  navPID.reset();
    if (newMode == MODE_RTH)     rthState = RTH_CLIMB;
    if (newMode == MODE_MISSION) currentWaypoint = 0;
    Serial.printf("[MODE] → %s\n",
      newMode == MODE_MANUAL  ? "MANUAL"  :
      newMode == MODE_MISSION ? "MISSION" : "RTH");
  }

  return newMode;
}

// =================================================================
//  RTH GUIDANCE
//  No yaw mix — equal throttle both motors for safety
// =================================================================
void runRTH() {
  // No GPS fix — hold level
  if (!gps_fix) {
    target_roll_deg   = 0.0f;
    target_pitch_deg  = 0.0f;
    target_throttle   = AUTO_CRUISE_THROTTLE;
    return;
  }

  float distHome    = calcDistance(gps_lat, gps_lon,
                                   home_lat, home_lon);
  float bearingHome = calcBearing(gps_lat, gps_lon,
                                  home_lat, home_lon);
  float curAlt      = relativeAlt;

  // ── Step 1: Climb ────────────────────────────────────────────
  if (rthState == RTH_CLIMB) {
    if (curAlt < RTH_SAFE_ALTITUDE - 2.0f) {
      float altError       = RTH_SAFE_ALTITUDE - curAlt;
      target_pitch_deg     = constrain(altError * 0.5f,
                                       0.0f, AUTO_MAX_PITCH_DEG);
      target_roll_deg      = 0.0f;
      target_throttle      = AUTO_CLIMB_THROTTLE;
    } else {
      rthState = RTH_FLY;
      Serial.printf("[RTH] Altitude %.1fm reached — flying home\n",
                    curAlt);
    }
  }

  // ── Step 2: Fly home ─────────────────────────────────────────
  if (rthState == RTH_FLY) {
    if (distHome > RTH_ORBIT_RADIUS) {
      float headErr        = calcHeadingError(gps_course, bearingHome);
      target_roll_deg      = navPID.compute(headErr);
      float altError       = RTH_SAFE_ALTITUDE - curAlt;
      target_pitch_deg     = altPID.compute(altError);
      target_throttle      = AUTO_CRUISE_THROTTLE;
    } else {
      rthState = RTH_ORBIT;
      Serial.printf("[RTH] Home reached %.1fm — orbiting\n", distHome);
    }
  }

  // ── Step 3: Orbit ────────────────────────────────────────────
  if (rthState == RTH_ORBIT) {
    float orbitBearing   = fmod(bearingHome + 90.0f, 360.0f);
    float orbitHeadErr   = calcHeadingError(gps_course, orbitBearing);
    target_roll_deg      = constrain(navPID.compute(orbitHeadErr),
                                     -RTH_ORBIT_BANK, RTH_ORBIT_BANK);
    float altError       = RTH_SAFE_ALTITUDE - curAlt;
    target_pitch_deg     = altPID.compute(altError);
    target_throttle      = AUTO_CRUISE_THROTTLE;
  }
}

// =================================================================
//  MISSION GUIDANCE
// =================================================================
void runMission() {
  if (!gps_fix || currentWaypoint >= WAYPOINT_COUNT) {
    target_roll_deg  = 0.0f;
    target_pitch_deg = 0.0f;
    target_throttle  = AUTO_CRUISE_THROTTLE;

    // Mission complete — go RTH
    if (currentWaypoint >= WAYPOINT_COUNT) {
      Serial.println("[MISSION] Complete — switching to RTH");
      rthState = RTH_CLIMB;
      currentMode = MODE_RTH;
    }
    return;
  }

  Waypoint wp  = WAYPOINTS[currentWaypoint];
  float dist   = calcDistance(gps_lat, gps_lon, wp.lat, wp.lon);
  float brng   = calcBearing(gps_lat, gps_lon, wp.lat, wp.lon);

  dist_to_wp    = dist;
  bearing_to_wp = brng;

  // Arrived at waypoint
  if (dist < WAYPOINT_ARRIVAL_RADIUS) {
    Serial.printf("[MISSION] WP%d reached — %.1fm\n",
                  currentWaypoint + 1, dist);
    currentWaypoint++;
    navPID.reset();
    return;
  }

  float headErr        = calcHeadingError(gps_course, brng);
  target_roll_deg      = navPID.compute(headErr);
  float altError       = wp.alt_m - relativeAlt;
  target_pitch_deg     = altPID.compute(altError);
  target_throttle      = AUTO_CRUISE_THROTTLE;
}

// =================================================================
//  CORE 0 — CONTROL TASK (500Hz)
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
  Serial.println("[CTRL] ESCs armed. Running.");

  for (;;) {
    // Signal loss check
    if (rcConnected && millis() - lastGoodPacket > RC_TIMEOUT_MS)
      rcConnected = false;

    // Hard failsafe — no RC and no home set
    if (!rcConnected && !homeSet) {
      applyFailsafe();
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
      continue;
    }

    // Snapshot shared state
    FlightMode mode     = currentMode;
    float      tgtRoll  = target_roll_deg;
    float      tgtPitch = target_pitch_deg;
    uint16_t   tgtThr   = target_throttle;
    float      actRoll  = imu_roll;
    float      actPitch = imu_pitch;

    uint32_t p, al, ar;
    uint32_t e_l, e_r;

    if (mode == MODE_MANUAL) {
      // ── MANUAL — passthrough + differential thrust yaw ────────
      uint16_t roll     = ch_roll;
      uint16_t pitch    = ch_pitch;
      uint16_t throttle = ch_throttle;
      uint16_t yaw      = ch_yaw;

      // Elevator
      p = constrain((uint32_t)pitch, 1000, 2000);
      if (p > 1500 - RC_DEADZONE && p < 1500 + RC_DEADZONE) p = 1500;

      // Ailerons
      al = constrain((uint32_t)roll, 1000, 2000);
      if (al > 1500 - RC_DEADZONE && al < 1500 + RC_DEADZONE) al = 1500;
      ar = al;  // always opposite

      // Differential thrust yaw — CH4
      int yawMix = map((int)yaw, 1000, 2000, -YAW_MIX, YAW_MIX);
      if (yaw > 1500 - RC_DEADZONE && yaw < 1500 + RC_DEADZONE)
        yawMix = 0;

      int baseThrottle = (int)constrain((uint32_t)throttle, 1000, 2000);
      e_l = (uint32_t)constrain(baseThrottle + yawMix, 1000, 2000);
      e_r = (uint32_t)constrain(baseThrottle - yawMix, 1000, 2000);

    } else {
      // ── AUTO (MISSION or RTH) — stabilized PID, NO yaw mix ────
      float rollCorr  = rollPID.compute(tgtRoll  - actRoll);
      float pitchCorr = pitchPID.compute(tgtPitch - actPitch);

      al  = (uint32_t)constrain(1500 + rollCorr,  1000, 2000);
      ar  = (uint32_t)constrain(1500 - rollCorr,  1000, 2000);
      p   = (uint32_t)constrain(1500 + pitchCorr, 1000, 2000);

      // Equal throttle both motors — no yaw mix in auto
      uint32_t autoThr = constrain((uint32_t)tgtThr,
                                   AUTO_MIN_THROTTLE, 2000);
      e_l = autoThr;
      e_r = autoThr;
    }

    // Write to hardware
    pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, p);
    pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, al);
    pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, ar);
    pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, e_l);
    pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, e_r);

    out_pitch = p;
    out_ail_l = al;
    out_ail_r = ar;
    out_esc_l = e_l;
    out_esc_r = e_r;

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// =================================================================
//  CORE 1 — SENSORS + GUIDANCE TASK
// =================================================================
void commsTask(void* param) {
  esp_task_wdt_delete(NULL);

  uint32_t lastImuRead   = 0;
  uint32_t lastBaroRead  = 0;
  uint32_t lastBatRead   = 0;
  uint32_t lastNavUpdate = 0;
  uint32_t lastRPiSend   = 0;
  uint32_t lastPrint     = 0;

  for (;;) {
    uint32_t now = millis();

    // iBUS — every iteration
    parseIBUS();

    // GPS — every iteration
    readGPS();

    // IMU — every 2ms
    if (now - lastImuRead >= IMU_READ_MS) {
      lastImuRead = now;
      readIMU();
    }

    // Barometer — every 20ms
    if (now - lastBaroRead >= BARO_READ_MS) {
      lastBaroRead = now;
      readBaro();
    }

    // Battery — every 1 second
    if (now - lastBatRead >= 1000) {
      lastBatRead = now;
      readBattery();
    }

    // Navigation + mode — every 200ms
    if (now - lastNavUpdate >= NAV_UPDATE_MS) {
      lastNavUpdate = now;
      currentMode   = updateMode();

      switch (currentMode) {
        case MODE_MANUAL:
          break;
        case MODE_MISSION:
          runMission();
          break;
        case MODE_RTH:
          runRTH();
          break;
      }
    }

    // LED — every iteration (uses millis internally)
    updateLED();

    // RPi telemetry — every 10ms fire and forget
    if (now - lastRPiSend >= RPI_SEND_MS) {
      lastRPiSend = now;
      // sendToRPi(); // uncomment when RPi connected
    }

    // Serial debug — every 500ms
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
  const char* rthStr  =
    rthState == RTH_CLIMB ? "CLIMB" :
    rthState == RTH_FLY   ? "FLY  " : "ORBIT";

  Serial.println("─────────────────────────────────────────────");
  Serial.printf("MODE  %s  %s\n",
    modeStr, currentMode == MODE_RTH ? rthStr : "");

  Serial.printf("RC    %s  R:%4d  P:%4d  T:%4d  Y:%4d  CH5:%4d\n",
    rcConnected ? "[OK]  " : "[LOST]",
    (int)ch_roll, (int)ch_pitch, (int)ch_throttle,
    (int)ch_yaw,  (int)ch_mode);

  if (imuReady)
    Serial.printf("IMU   Roll:%6.1f  Pitch:%6.1f  Yaw:%6.1f deg\n",
      (float)imu_roll, (float)imu_pitch, (float)imu_yaw);
  else
    Serial.println("IMU   not ready");

  if (baroReady)
    Serial.printf("BARO  Alt:%6.1fm  Rel:%5.1fm  Temp:%.1fC\n",
      (float)altitude_m, relativeAlt, (float)baro_temp_c);
  else
    Serial.println("BARO  not ready");

  if (gps_fix)
    Serial.printf("GPS   Fix:YES  Sats:%2d  Lat:%.6f  Lon:%.6f  Crs:%.1f\n",
      (int)gps_sats, (float)gps_lat, (float)gps_lon, (float)gps_course);
  else
    Serial.printf("GPS   Fix:NO   Sats:%2d\n", (int)gps_sats);

  Serial.printf("HOME  %s  Lat:%.6f  Lon:%.6f\n",
    homeSet ? "SET   " : "NOTSET",
    (float)home_lat, (float)home_lon);

  Serial.printf("VBAT  %.2fV  %s\n",
    (float)vbat, lowBattery ? "!! LOW BATTERY !!" : "OK");

  if (currentMode != MODE_MANUAL)
    Serial.printf("TGT   Roll:%5.1f  Pitch:%5.1f  Thr:%4d\n",
      (float)target_roll_deg, (float)target_pitch_deg,
      (int)target_throttle);

  if (currentMode == MODE_MISSION && currentWaypoint < WAYPOINT_COUNT)
    Serial.printf("WP    %d/%d  Dist:%.1fm  Bearing:%.1fdeg\n",
      currentWaypoint + 1, WAYPOINT_COUNT,
      (float)dist_to_wp, (float)bearing_to_wp);

  Serial.printf("OUT   P:%4lu  AL:%4lu  AR:%4lu  EL:%4lu  ER:%4lu\n",
    out_pitch, out_ail_l, out_ail_r, out_esc_l, out_esc_r);
}

// =================================================================
//  SENSOR INIT
// =================================================================
bool initIMU() {
  byte status = mpu.begin();
  if (status != 0) {
    Serial.printf("[IMU] Failed status=%d\n", status);
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


  Serial.println("[BARO] Disabled temporarily.");
  return false;
 /** while (!bmp.begin(0x47, &Wire)) {
    Serial.println("[BARO] Not found at 0x47 — check wiring and CS pin");
    delay(1000);
  }

  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

  Serial.println("[BARO] BMP581 ready.");
  return true; **/
}

void initGPS() {
  gpsSerial.setRxBufferSize(512);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[GPS] Started on GPIO32.");
}

// =================================================================
//  SETUP
// =================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println("\n================================================");
  Serial.println("  Fixed Wing UAV — Flight Controller v3.1");
  Serial.println("  Manual | Mission | RTH | Diff Thrust Yaw");
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

  // LED
  initLED();
  Serial.println("[OK] LED on GPIO2");

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

  if (!imuOk)  Serial.println("[WARN] No IMU");
  if (!baroOk) Serial.println("[WARN] No barometer");

  Serial.println("[OK] Waiting for GPS fix to lock home...");
  Serial.println("[OK] CH5: <1400=MANUAL  1400-1600=MISSION  >1600=RTH");
  Serial.println("[OK] CH4: yaw (diff thrust) — manual mode only");

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
