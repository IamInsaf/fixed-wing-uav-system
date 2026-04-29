# fixed-wing-uav-system
RC long range fixed wing aircraft with autonomous flight 

> A fully custom fixed wing UAV built from scratch with autonomous flight capabilities including GPS waypoint navigation, Return to Home, and real-time telemetry. Built around an ESP32 flight controller with Raspberry Pi companion computer.

---

## Project overview

This project is a complete ground-up design and build of a fixed wing UAV. The aircraft is capable of manual RC flight, autonomous GPS waypoint missions, and automatic Return to Home (RTH) triggered by signal loss, low battery, or pilot command. All flight controller firmware is written in C++ for the ESP32 using FreeRTOS dual-core architecture for guaranteed real-time performance.

The project covers every aspect of UAV development — aerodynamic design, airframe fabrication, electronics integration, PCB design, firmware development, sensor fusion, and autonomous navigation algorithms.

---

## Current status

| System | Status |
|---|---|
| Airframe design | Complete — dimensions finalized |
| Flight controller firmware v3.1 | Complete |
| Manual flight | Tested — working |
| IMU stabilization | Implemented |
| Barometer | Implemented |
| GPS | Tested and confirmed working |
| RTH mode | Implemented — untested in flight |
| Mission mode | Implemented — untested in flight |
| Differential thrust yaw | Implemented |
| LED mode indicator | Implemented |
| PCB design | In progress |
| Airframe fabrication | In progress |
| First autonomous flight | Pending |
| Raspberry Pi integration | Planned |
| FPV / VTX | Planned |

---

## Aircraft specifications

### Airframe — twin boom pusher

| Parameter | Value |
|---|---|
| Configuration | High wing, twin boom, pusher motor |
| Wingspan | 1200mm |
| Wing chord | 220mm (constant) |
| Wing area | 0.264 m² |
| Wing dihedral | 3° each side |
| Pod length | 480mm |
| Pod width | 130mm |
| Pod height | 85mm |
| Boom material | 8mm aluminum rod |
| Boom length | 400mm total |
| Effective moment arm | 403mm |
| Horizontal tail span | 420mm |
| Horizontal tail chord | 120mm |
| Elevator depth | 45mm |
| Vertical stabilizer height | 160mm (×2) |
| Vertical stabilizer chord | 140mm |
| CG position | 66mm from wing leading edge |
| Estimated flying weight | ~847g |

### Propulsion

| Parameter | Value |
|---|---|
| Motor | 1000KV brushless DC |
| Propeller | 1045 pusher |
| Battery | 3S LiPo 2200mAh |
| ESC | SimonK 30A |
| Estimated thrust | ~1200g |
| Thrust to weight ratio | ~1.42 |

### Control surfaces

| Surface | Actuation | GPIO |
|---|---|---|
| Elevator | 1× servo | GPIO13 |
| Aileron left | 1× servo | GPIO14 |
| Aileron right | 1× servo (reversed) | GPIO27 |
| Yaw | Differential thrust | GPIO25 / GPIO26 |

---

## Electronics

### Flight controller

| Component | Details |
|---|---|
| Main MCU | ESP32 Dev Module |
| IMU | MPU6050 (I2C 0x68) |
| Barometer | BMP581 7SEMI (I2C 0x47) |
| GPS | NEO-6M (UART1 GPIO32) |
| RC receiver | FS-IA6B via iBUS (UART2 GPIO16) |
| ESC ×2 | SimonK 30A |
| Servo ×3 | Standard |
| LED indicator | Single LED GPIO2 |
| Battery monitor | 100kΩ/47kΩ divider → GPIO34 |

### Companion computer (planned)

| Component | Details |
|---|---|
| SBC | Raspberry Pi 4B |
| Connection | UART (GPIO1 TX / GPIO3 RX) |
| Role | Telemetry, GPS processing, NRF24L01 radio, logging, future CV |
| Power | Dedicated UBEC 5V 3A |

### Power system

| Rail | Source | Powers |
|---|---|---|
| LiPo direct | LiPo B+ | Both ESC motor power |
| 5V rail A | ESC left BEC (2A) | ESP32, servos, receiver, sensors |
| 5V rail B | External UBEC (3A) | Raspberry Pi only |
| 3.3V | ESP32 internal regulator | MPU6050, BMP581, NEO-6M |

---

## Pin assignment — ESP32

| GPIO | Function | Direction | Notes |
|---|---|---|---|
| GPIO16 | iBUS RX | Input | UART2 from FS-IA6B |
| GPIO21 | I2C SDA | Bidirectional | MPU6050 + BMP581 shared |
| GPIO22 | I2C SCL | Bidirectional | MPU6050 + BMP581 shared |
| GPIO32 | GPS RX | Input | UART1 from NEO-6M TX |
| GPIO33 | GPS TX | Output | UART1 to NEO-6M RX (optional) |
| GPIO34 | Battery ADC | Input | Via 100k/47k voltage divider |
| GPIO1 | RPi UART TX | Output | Fire and forget telemetry |
| GPIO3 | RPi UART RX | Input | Commands from RPi |
| GPIO13 | Elevator servo | Output | MCPWM Unit0 Timer0 |
| GPIO14 | Aileron L servo | Output | MCPWM Unit0 Timer1 |
| GPIO27 | Aileron R servo | Output | MCPWM Unit0 Timer2 (reversed) |
| GPIO25 | ESC left | Output | MCPWM Unit1 Timer0 |
| GPIO26 | ESC right | Output | MCPWM Unit1 Timer1 |
| GPIO2 | LED indicator | Output | Single LED mode display |

---

## Firmware architecture

### Dual core FreeRTOS

```
Core 0 — priority 24 — 500Hz control loop
├── Reads volatile: ch_roll, ch_pitch, ch_throttle, ch_yaw
├── Reads volatile: target_roll_deg, target_pitch_deg, target_throttle
├── Reads volatile: imu_roll, imu_pitch
├── Manual mode: RC passthrough + differential thrust yaw
├── Auto mode: PID stabilization against IMU angles
└── Writes MCPWM outputs to 3 servos + 2 ESCs

Core 1 — priority 5 — sensors + guidance
├── iBUS parser (non-blocking state machine)
├── MPU6050 read every 2ms (complementary filter)
├── BMP581 read every 20ms
├── NEO-6M GPS drain every iteration
├── Battery voltage every 1s
├── Navigation + mode manager every 200ms
├── LED indicator every iteration
└── RPi UART telemetry every 10ms (fire and forget)
```

### Flight modes — CH5

| CH5 value | Mode | Behaviour |
|---|---|---|
| < 1400µs | MANUAL | RC passthrough, differential thrust yaw on CH4 |
| 1400–1600µs | MISSION | GPS waypoint following, altitude hold |
| > 1600µs | RTH | Climb → fly home → orbit |

### Auto RTH triggers

- RC signal lost for more than 200ms
- Battery voltage drops below 10.5V
- Mission complete (automatically switches to RTH)

### RTH sequence

```
1. CLIMB  — pitch up, full throttle, wings level until 30m altitude
2. FLY    — navigate toward home GPS coordinate using navPID → aileron
3. ORBIT  — within 30m of home, constant 25° bank, circle indefinitely
4. OVERRIDE — CH5 to manual = instant pilot control return
```

### LED blink codes

| Pattern | Meaning |
|---|---|
| Single slow blink | Manual mode |
| Double blink | Mission mode |
| Triple blink | RTH — flying home |
| Solid on | RTH — orbiting |
| Fast blink | RC signal lost |
| SOS (···−−−···) | Low battery |

---

## PID controllers

| PID | Input | Output | Kp | Ki | Kd |
|---|---|---|---|---|---|
| rollPID | roll angle error (deg) | aileron correction (µs) | 8.0 | 0.5 | 2.0 |
| pitchPID | pitch angle error (deg) | elevator correction (µs) | 8.0 | 0.5 | 2.0 |
| altPID | altitude error (m) | pitch target (deg) | 2.0 | 0.1 | 1.0 |
| navPID | heading error (deg) | roll target (deg) | 0.5 | 0.02 | 0.1 |

All PID values are starting points. Tune in flight — increase Kp until responsive, add Kd to stop oscillation, add Ki to remove steady state error.

---

## Sensor wiring

### MPU6050

| MPU6050 pin | Connects to |
|---|---|
| VCC | ESP32 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |
| ADO | GND (sets I2C address 0x68) |
| XDA, XCL, INT | not connected |

### BMP581 (7SEMI breakout)

| BMP581 pin | Connects to |
|---|---|
| VIN | ESP32 3V3 |
| GND | GND |
| SDI | GPIO21 (SDA) |
| SCK | GPIO22 (SCL) |
| SDO | GND (sets I2C address 0x47) |
| CS | 3V3 (enables I2C mode — critical) |
| INT | not connected |

### NEO-6M GPS

| GPS pin | Connects to |
|---|---|
| VCC | 5V or 3V3 |
| GND | GND |
| TX | GPIO32 |
| RX | GPIO33 (optional) |

### FS-IA6B receiver

| Receiver | Connects to |
|---|---|
| VCC | 5V BEC rail |
| GND | GND |
| iBUS port (side) | GPIO16 |

---

## Libraries required

Install all via Arduino IDE Library Manager:

| Library | Author | Purpose |
|---|---|---|
| MPU6050_light | rfetick | IMU reading |
| Adafruit BMP5xx | Adafruit | Barometer |
| Adafruit BusIO | Adafruit | I2C abstraction (auto-installed) |
| TinyGPSPlus | Mikal Hart | GPS NMEA parsing |

No external libraries needed for MCPWM, FreeRTOS, WiFi disable — all built into ESP32 Arduino core.

---

## Firmware versions

| Version | Changes |
|---|---|
| v1.0 | Basic iBUS read + servo passthrough + ESC output |
| v1.1 | Replaced ESP32Servo with MCPWM hardware PWM, FreeRTOS dual core, WiFi/BT disabled |
| v1.2 | Fixed random stall — larger UART buffer, watchdog disabled |
| v2.0 | Added MPU6050 IMU, BMP581 barometer, GPS, dual core sensor architecture |
| v2.1 | Added RPi UART telemetry, battery monitor, BMP5xx library fix, yaw drift fix |
| v3.0 | Added flight modes (manual/mission/RTH), PID stabilization, navigation, waypoint manager |
| v3.1 | Added differential thrust yaw (CH4), single LED indicator, no yaw in RTH/mission |

---

## Airframe build order

1. Cut all foam pieces from PVC sheet per cutting list
2. Build pod box — 480×130×85mm, leave top removable for electronics access
3. Cut wing halves 600×220mm, join at center with 3° dihedral each side
4. Insert 6mm spar rod at 55mm from leading edge through full span
5. Mount wing on top of pod, leading edge at 150mm from nose
6. Attach aluminum booms to wing underside at 90mm each side of centerline
7. Build motor pylon 70mm tall, mount at wing trailing edge between booms
8. Build horizontal tail 420×120mm, attach between boom tips
9. Attach vertical stabilizers 160×140mm at each tip of horizontal tail
10. Mount motor on pylon, prop facing rearward
11. Load electronics into pod — battery in nose zone
12. Balance at 66mm from wing leading edge — adjust battery position

---

## Pre-flight checklist

- [ ] Battery fully charged (4.2V per cell, 12.6V total for 3S)
- [ ] Transmitter on before powering UAV
- [ ] Throttle stick fully down before power on
- [ ] Serial Monitor confirms IMU calibrated and RC channels responding
- [ ] CH5 switch confirmed — values change when flipped
- [ ] Aileron direction correct — roll right = left aileron up, right aileron down
- [ ] Elevator direction correct — pitch back = elevator up
- [ ] ESC arming beeps heard (3 second sequence on boot)
- [ ] GPS fix confirmed — LED solid after searching
- [ ] Home locked — Serial Monitor shows "Home set: lat, lon"
- [ ] Props on and secure — correct rotation direction for pusher
- [ ] CG checked — balances at 66mm from wing leading edge
- [ ] RTH test — flip CH5 to RTH, confirm LED triple blink

---

## Planned future additions

- [ ] Raspberry Pi integration — UART telemetry receiver, NRF24L01 ground station
- [ ] FPV camera + 5.8GHz VTX
- [ ] Ground station web dashboard
- [ ] PID auto-tune from flight data
- [ ] Airspeed sensor for stall protection
- [ ] Parachute deployment on critical failure

---

## Repository structure

```
/
├── firmware/
│   ├── fc_v1.ino          — basic passthrough
│   ├── fc_v2_1.ino        — sensors + dual core
│   └── fc_v3_1.ino        — full autonomous (latest)
├── docs/
│   └── uav_dimensions_final.md
└── README.md
```

---

## Notes

- The ESP32 UART0 (GPIO1/GPIO3) is shared between USB serial and RPi UART. Disconnect RPi before flashing firmware.
- BMP581 CS pin must be connected to 3V3. If left floating the sensor stays in SPI mode and will not respond on I2C.
- ESC right BEC red wire must be cut or taped — two BECs on the same 5V rail will damage each other.
- GPS antenna must face sky with clear view. Does not need to point in any direction — orientation does not affect GPS readings.
- Yaw differential thrust (CH4) is active in manual mode only. RTH and mission use equal throttle on both motors for safety.
- Never fly in auto mode without GPS fix confirmed and home position locked.

