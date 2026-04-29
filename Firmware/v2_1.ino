// ================================================================
//  FIXED WING UAV — FLIGHT CONTROLLER v2.1
//  ESP32 Dev Module
// ================================================================
//
//  ARCHITECTURE
//  Core 0 (priority 24) — Control loop 500Hz
//    iBUS → target values → MCPWM outputs
//
//  Core 1 (priority 5) — Sensors + Comms
//    IMU, Barometer, UART to RPi (optional), Serial debug
//
//  RPi is fully optional — flies perfectly without it
//  UART to RPi is fire-and-forget, never blocks
//
// ── PINOUT ───────────────────────────────────────────────────────
//  iBUS (FS-IA6B)    → GPIO16  (UART2 RX)
//  IMU SDA           → GPIO21  (I2C)
//  IMU SCL           → GPIO22  (I2C)
//  Elevator servo    → GPIO13  (MCPWM Unit0 Timer0)
//  Aileron L servo   → GPIO14  (MCPWM Unit0 Timer1)
//  Aileron R servo   → GPIO27  (MCPWM Unit0 Timer2)
//  ESC left          → GPIO25  (MCPWM Unit1 Timer0)
//  ESC right         → GPIO26  (MCPWM Unit1 Timer1)
//  RPi UART TX       → GPIO1   (optional, fire-and-forget)
//  Battery voltage   → GPIO34  (ADC, via 100k/47k divider)
//
// ── WIRING ───────────────────────────────────────────────────────
//  FS-IA6B VCC  → 5V (UBEC)      FS-IA6B GND → GND
//  FS-IA6B iBUS → GPIO16
//  MPU6050 VCC  → 3.3V           MPU6050 GND → GND
//  MPU6050 SDA  → GPIO21         MPU6050 SCL → GPIO22
//  BMP5xx  VCC  → 3.3V           BMP5xx  GND → GND
//  BMP5xx  SDA  → GPIO21         BMP5xx  SCL → GPIO22
//  NRF24L01 → handled by RPi, not ESP32
//  GPS      → handled by RPi, not ESP32
//
// ── LIBRARIES ────────────────────────────────────────────────────
//  MPU6050_light  by rfetick       (Library Manager)
//  Adafruit BMP5xx by Adafruit     (Library Manager)
//  Adafruit BusIO  by Adafruit     (auto-installed with BMP5xx)
//
// ── POWER ────────────────────────────────────────────────────────
//  LiPo → ESC left + right (direct, thick wire)
//  LiPo → UBEC 1 (5V 2A) → ESP32 VIN, FS-IA6B, Servos x3
//  LiPo → UBEC 2 (5V 3A) → Raspberry Pi (separate, isolated)
//  ESP32 3.3V pin → MPU6050, BMP5xx
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

// =================================================================
//  CONFIGURATION
// =================================================================

// Complementary filter — higher = trust gyro more (less accel noise)
#define COMP_ALPHA          0.98f

// Dead zone — ignore stick movement smaller than this
#define RC_DEADZONE         10

// Signal loss threshold
#define RC_TIMEOUT_MS       200

// Loop rates
#define CONTROL_HZ          500
#define CONTROL_PERIOD_MS   (1000 / CONTROL_HZ)   // 2ms
#define IMU_READ_MS         2
#define BARO_READ_MS        20
#define RPi_SEND_MS         10      // 100Hz to RPi
#define DEBUG_PRINT_MS      500

// Serial
#define SERIAL_BAUD         500000

// Battery monitor — voltage divider ratio
// 100k + 47k divider: Vbat * (47 / 147) = Vadc
// ADC max 3.3V = 12s full (adjust for your divider)
#define VDIV_RATIO          (147.0f / 47.0f)
#define ADC_REF             3.3f
#define ADC_RESOLUTION      4095.0f

// =================================================================
//  PINS
// =================================================================
#define IBUS_RX_PIN         16
#define IBUS_TX_PIN         17
#define PIN_SERVO_PITCH     13
#define PIN_SERVO_AIL_L     14
#define PIN_SERVO_AIL_R     27
#define PIN_ESC_LEFT        25
#define PIN_ESC_RIGHT       26
#define I2C_SDA             21
#define I2C_SCL             22
#define PIN_VBAT            34
#define RPI_TX_PIN          1       // ESP32 TX → RPi RX
#define RPI_RX_PIN          3       // RPi TX  → ESP32 RX (reserved)

// =================================================================
//  SHARED VOLATILE STATE
//  Written by Core 1 sensors, read by Core 0 control loop
// =================================================================

// RC (written by Core 1 iBUS parser)
volatile uint16_t ch_roll       = 1500;
volatile uint16_t ch_pitch      = 1500;
volatile uint16_t ch_throttle   = 1000;
volatile bool     rcConnected   = false;
volatile uint32_t lastGoodPacket = 0;

// IMU angles — degrees, complementary filter output
volatile float    imu_roll      = 0.0f;
volatile float    imu_pitch     = 0.0f;
volatile float    imu_yaw       = 0.0f;
volatile bool     imuReady      = false;

// Barometer
volatile float    altitude_m    = 0.0f;
volatile float    pressure_hpa  = 0.0f;
volatile float    baro_temp_c   = 0.0f;
volatile bool     baroReady     = false;

// Battery
volatile float    vbat          = 0.0f;

// Current servo outputs (written by Core 0, read by Core 1 for RPi)
volatile uint32_t out_pitch     = 1500;
volatile uint32_t out_ail_l     = 1500;
volatile uint32_t out_ail_r     = 1500;
volatile uint32_t out_esc_l     = 1000;
volatile uint32_t out_esc_r     = 1000;

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
    if (checksum != (uint16_t)(ibusBuffer[30] | ibusBuffer[31] << 8)) continue;

    for (int i = 0; i < 14; i++)
      ibusChannels[i] = ibusBuffer[2 + i*2] | (ibusBuffer[3 + i*2] << 8);

    ch_roll       = ibusChannels[0];
    ch_pitch      = ibusChannels[1];
    ch_throttle   = ibusChannels[2];
    lastGoodPacket = millis();
    rcConnected   = true;
  }
}

// =================================================================
//  MCPWM — direct hardware PWM, zero overhead
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
//  MAPPING
// =================================================================
inline uint32_t rcToServoPulse(uint16_t rc, bool reversed) {
  rc = constrain(rc, 1000, 2000);
  if (rc > 1500 - RC_DEADZONE && rc < 1500 + RC_DEADZONE) rc = 1500;
  return reversed ? (3000 - rc) : rc;
}

inline uint32_t rcToESCPulse(uint16_t rc) {
  return constrain((uint32_t)rc, 1000, 2000);
}

// =================================================================
//  FAILSAFE — centers surfaces, kills motors
// =================================================================
void applyFailsafe() {
  pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1500);
  pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 1500);
  pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 1500);
  pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
  pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 1000);
  out_pitch = 1500; out_ail_l = 1500; out_ail_r = 1500;
  out_esc_l = 1000; out_esc_r = 1000;
}

// =================================================================
//  IMU — MPU6050 complementary filter
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

  // Complementary filter
  cf_roll  = COMP_ALPHA * (cf_roll  + mpu.getGyroX() * dt)
           + (1.0f - COMP_ALPHA) * mpu.getAngleX();
  cf_pitch = COMP_ALPHA * (cf_pitch + mpu.getGyroY() * dt)
           + (1.0f - COMP_ALPHA) * mpu.getAngleY();

  // Yaw — gyro integration only (no magnetometer)
  cf_yaw += mpu.getGyroZ() * dt;
  if (cf_yaw >  180.0f) cf_yaw -= 360.0f;
  if (cf_yaw < -180.0f) cf_yaw += 360.0f;

  imu_roll  = cf_roll;
  imu_pitch = cf_pitch;
  imu_yaw   = cf_yaw;
  imuReady  = true;
}

// =================================================================
//  BAROMETER — BMP5xx
// =================================================================
Adafruit_BMP5xx bmp;
float seaLevelPressure = 1013.25f; // hPa — update for your location

void readBaro() {
  if (bmp.performReading()) {
    pressure_hpa = bmp.pressure;
    baro_temp_c  = bmp.temperature;
    altitude_m   = bmp.readAltitude(seaLevelPressure);
    baroReady    = true;
  }
}

// =================================================================
//  BATTERY VOLTAGE
// =================================================================
void readBattery() {
  int raw   = analogRead(PIN_VBAT);
  float adc = (raw / ADC_RESOLUTION) * ADC_REF;
  vbat      = adc * VDIV_RATIO;
}

// =================================================================
//  RPi TELEMETRY — fire and forget, never blocks
//  Plane flies identically whether RPi is connected or not
// =================================================================
#pragma pack(1)
struct FCtoRPi {
  uint16_t rc_roll;
  uint16_t rc_pitch;
  uint16_t rc_throttle;
  float    imu_roll;
  float    imu_pitch;
  float    imu_yaw;
  float    altitude_m;
  float    pressure_hpa;
  float    vbat;
  uint32_t out_pitch;
  uint32_t out_ail_l;
  uint32_t out_ail_r;
  uint32_t out_esc;
  uint32_t timestamp_ms;
  uint8_t  flags;       // bit0=rc_ok bit1=imu_ok bit2=baro_ok
  uint8_t  checksum;    // XOR of all previous bytes
};
#pragma pack()

HardwareSerial rpiSerial(0); // UART0 — GPIO1 TX, GPIO3 RX

void sendToRPi() {
  FCtoRPi pkt;
  pkt.rc_roll     = ch_roll;
  pkt.rc_pitch    = ch_pitch;
  pkt.rc_throttle = ch_throttle;
  pkt.imu_roll    = imu_roll;
  pkt.imu_pitch   = imu_pitch;
  pkt.imu_yaw     = imu_yaw;
  pkt.altitude_m  = altitude_m;
  pkt.pressure_hpa = pressure_hpa;
  pkt.vbat        = vbat;
  pkt.out_pitch   = out_pitch;
  pkt.out_ail_l   = out_ail_l;
  pkt.out_ail_r   = out_ail_r;
  pkt.out_esc     = out_esc_l;
  pkt.timestamp_ms = millis();

  uint8_t flags = 0;
  if (rcConnected) flags |= (1 << 0);
  if (imuReady)   flags |= (1 << 1);
  if (baroReady)  flags |= (1 << 2);
  pkt.flags = flags;

  // XOR checksum
  uint8_t* raw = (uint8_t*)&pkt;
  uint8_t  cs  = 0;
  for (size_t i = 0; i < sizeof(pkt) - 1; i++) cs ^= raw[i];
  pkt.checksum = cs;

  // Write to UART — non-blocking, TX buffer handles it
  rpiSerial.write((uint8_t*)&pkt, sizeof(pkt));
}

// =================================================================
//  SENSOR INIT
// =================================================================
bool initIMU() {
  byte status = mpu.begin();
  if (status != 0) {
    Serial.printf("[IMU] MPU6050 failed, status=%d\n", status);
    return false;
  }
  Serial.println("[IMU] Calibrating — keep UAV still for 3 seconds...");
  delay(3000);
  mpu.calcOffsets(true, true);
  imuLastUs = micros();
  Serial.println("[IMU] Ready.");
  return true;
}

bool initBaro() {
  if (!bmp.begin()) {
    Serial.println("[BARO] BMP5xx not found — check wiring and CS pin");
    return false;
  }
  Serial.println("[BARO] BMP5xx ready.");
  return true;
}


// =================================================================
//  CORE 0 — CONTROL TASK (500Hz, real-time, untouchable)
// =================================================================
void controlTask(void* param) {
  esp_task_wdt_delete(NULL);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

  // ESC arming — min throttle for 3s
  // vTaskDelayUntil keeps exact timing even during arming
  Serial.println("[CTRL] Arming ESCs...");
  uint32_t armStart = millis();
  while (millis() - armStart < 3000) {
    pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
    pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 1000);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  Serial.println("[CTRL] ESCs armed. Control loop running.");

  for (;;) {
    // ── Signal loss check ─────────────────────────────────────
    if (rcConnected && millis() - lastGoodPacket > RC_TIMEOUT_MS) {
      rcConnected = false;
      Serial.println("[CTRL] RC SIGNAL LOST — failsafe");
    }

    // ── Failsafe ──────────────────────────────────────────────
    if (!rcConnected) {
      applyFailsafe();
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
      continue;
    }

    // ── Snapshot volatile vars once per iteration ─────────────
    uint16_t roll     = ch_roll;
    uint16_t pitch    = ch_pitch;
    uint16_t throttle = ch_throttle;

    // ── Compute outputs ───────────────────────────────────────
    uint32_t p  = rcToServoPulse(pitch,    false);
    uint32_t al = rcToServoPulse(roll,     false);
    uint32_t ar = rcToServoPulse(roll,     false);   // not reversed
    uint32_t e  = rcToESCPulse(throttle);

    // ── Write to hardware ─────────────────────────────────────
    pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, p);
    pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, al);
    pwm_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, ar);
    pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, e);
    pwm_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, e);

    // ── Update output state (for RPi telemetry) ───────────────
    out_pitch = p;
    out_ail_l = al;
    out_ail_r = ar;
    out_esc_l = e;
    out_esc_r = e;

    // ── Sleep until exact next 2ms tick — no drift ────────────
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// =================================================================
//  CORE 1 — COMMS + SENSORS TASK
// =================================================================
void commsTask(void* param) {
  esp_task_wdt_delete(NULL);

  uint32_t lastImuRead  = 0;
  uint32_t lastBaroRead = 0;
  uint32_t lastBatRead  = 0;
  uint32_t lastRPiSend  = 0;
  uint32_t lastPrint    = 0;

  for (;;) {
    uint32_t now = millis();

    // ── iBUS — drain every iteration ─────────────────────────
    parseIBUS();

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

    // ── RPi telemetry — every 10ms, fire and forget ───────────
    if (now - lastRPiSend >= RPi_SEND_MS) {
      lastRPiSend = now;
      sendToRPi();
    }

    // ── Serial debug — every 500ms ────────────────────────────
    if (now - lastPrint >= DEBUG_PRINT_MS) {
      lastPrint = now;

      Serial.println("─────────────────────────────────────────────");
      Serial.printf("RC    %s  Roll:%4d  Pitch:%4d  Thr:%4d\n",
        rcConnected ? "[OK]  " : "[LOST]",
        (int)ch_roll, (int)ch_pitch, (int)ch_throttle);

      if (imuReady)
        Serial.printf("IMU   Roll:%6.1f  Pitch:%6.1f  Yaw:%6.1f  deg\n",
          (float)imu_roll, (float)imu_pitch, (float)imu_yaw);
      else
        Serial.println("IMU   not ready");

      if (baroReady)
        Serial.printf("BARO  Alt:%6.1fm  Press:%7.2fhPa  Temp:%.1fC\n",
          (float)altitude_m, (float)pressure_hpa, (float)baro_temp_c);
      else
        Serial.println("BARO  not ready");

      Serial.printf("VBAT  %.2fV\n", (float)vbat);
      Serial.printf("OUT   Pitch:%4lu  AilL:%4lu  AilR:%4lu  ESC:%4lu\n",
        out_pitch, out_ail_l, out_ail_r, out_esc_l);
    }

    // Yield — 1ms, allows FreeRTOS to run housekeeping
    vTaskDelay(1);
  }
}

// =================================================================
//  SETUP
// =================================================================
void setup() {
  // UART0 for debug serial AND RPi telemetry
  // During flight: RPi receives binary packets on GPIO1
  // During bench testing with no RPi: Serial.print works normally
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println("\n================================================");
  Serial.println("  Fixed Wing UAV — Flight Controller v2.1");
  Serial.println("================================================\n");

  // ── Kill WiFi and Bluetooth — frees Core 0 completely ────────
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  Serial.println("[OK] WiFi + BT disabled");

  // ── Disable watchdog ──────────────────────────────────────────
  esp_task_wdt_deinit();
  Serial.println("[OK] Watchdog disabled");

  // ── Safe defaults ─────────────────────────────────────────────
  for (int i = 0; i < 14; i++) ibusChannels[i] = 1500;
  ibusChannels[2] = 1000;

  // ── iBUS UART — large buffer prevents packet loss ─────────────
  ibusSerial.setRxBufferSize(UART_BUF_SIZE);
  ibusSerial.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);
  Serial.println("[OK] iBUS UART on GPIO16");

  // ── I2C — 400kHz fast mode ────────────────────────────────────
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Serial.println("[OK] I2C at 400kHz");

  // ── Battery ADC ───────────────────────────────────────────────
  pinMode(PIN_VBAT, INPUT);
  analogReadResolution(12);
  Serial.println("[OK] Battery ADC on GPIO34");

  // ── Init sensors ──────────────────────────────────────────────
  bool imuOk  = initIMU();
  bool baroOk = initBaro();

  if (!imuOk)  Serial.println("[WARN] Flying without IMU data");
  if (!baroOk) Serial.println("[WARN] Flying without barometer");

  // ── MCPWM ─────────────────────────────────────────────────────
  mcpwm_setup();
  Serial.println("[OK] MCPWM hardware PWM configured");

  Serial.println("\n[OK] All systems go. Launching tasks...\n");

  // ── Core 0 — control task, highest priority ───────────────────
  xTaskCreatePinnedToCore(
    controlTask, "ctrl",
    8192,
    NULL,
    24,       // near-realtime priority
    NULL,
    0         // Core 0
  );

  // ── Core 1 — sensors + comms, low priority ────────────────────
  xTaskCreatePinnedToCore(
    commsTask, "comms",
    8192,
    NULL,
    5,        // low — yields to system and control
    NULL,
    1         // Core 1
  );
}

void loop() {
  // Arduino loop task killed — not needed
  vTaskDelete(NULL);
}
