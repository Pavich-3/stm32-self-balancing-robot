# Self-Balancing Robot — Project Context
 
## Project Overview
STM32-based self-balancing two-wheeled robot. Portfolio project demonstrating embedded systems expertise.
Written in **C++17** with HAL (direct register access where performance-critical).
 
---
 
## Hardware
 
| Component | Details |
|---|---|
| Main MCU | NUCLEO-F756ZG (Cortex-M7, 216 MHz, 320KB RAM, 1MB Flash) |
| Interface MCU | NUCLEO-L432KC (Cortex-M4, 80 MHz, 64KB RAM, 256KB Flash) |
| IMU | BMI270 — I2C, 1 kHz sampling, accelerometer + gyroscope |
| Motors | 2× DC motor with quadrature encoder, 170 RPM, 12V |
| Display | TFT LCD — SPI, connected to L432KC |
| Power | 3× 18650 Li-ion in series (~11.1V nominal, 12.6V max) |
| Motor driver | TB6612FNG or L298N (12V supply direct from battery) |
| Logic power | Buck converter 12V → 5V → NUCLEO boards |
 
---
 
## MCU Responsibility Split
 
### F756ZG — Control Core (real-time critical)
- FreeRTOS task scheduler
- IMU reading via I2C + DMA at 1 kHz
- Complementary filter + Kalman filter for tilt angle
- Cascaded PID: outer loop (angle) → inner loop (motor speed)
- PWM output to motor driver (TIM1/TIM8, complementary channels, dead-time)
- Quadrature encoder reading (TIM2, TIM3 in encoder mode)
- UART communication to L432KC (telemetry TX, parameter RX)
### L432KC — Interface Node
- TFT display driver via SPI + DMA
- Navigation menu (view telemetry, tune PID gains)
- Receives telemetry from F756ZG over UART
- Sends updated PID parameters back to F756ZG
- UART bridge to PC for logging/debugging
---
 
## Software Architecture
 
### RTOS Task Structure (F756ZG)
```
IMU Task        — priority 4 (highest), 1 kHz, reads BMI270, runs filter
PID Task        — priority 3, 1 kHz, cascaded PID, outputs PWM duty cycle
Encoder Task    — priority 3, reads encoder counts, calculates wheel speed
Comms Task      — priority 2, UART TX telemetry / RX parameter updates
Watchdog Task   — priority 1, monitors task health, triggers safe shutdown
```
 
### Key Algorithms
- **Angle estimation**: Complementary filter first, then Kalman filter
  - Complementary: `angle = α*(angle + gyro*dt) + (1-α)*accel_angle`
  - α ≈ 0.98 typical starting point
- **Cascaded PID**:
  - Outer loop: setpoint = 0° tilt, output = target wheel speed
  - Inner loop (per motor): setpoint = target speed, output = PWM duty
  - Anti-windup: integral clamping
  - Derivative: low-pass filtered to reduce noise
- **Encoder**: hardware timer encoder mode (no CPU interrupts needed)
---
 
## C++ Conventions
 
```cpp
// No heap allocation — ever
// No: new, delete, std::vector, std::string, std::map, std::function
// Yes: std::array, std::optional, constexpr, enum class, templates
 
// Compiler flags (always active)
// -fno-exceptions  -fno-rtti  -Os
 
// Object lifecycle: default constructor does nothing
// Explicit init() method called after HAL_Init() in main()
class Peripheral {
public:
    Peripheral() = default;
    bool init(...);  // called manually, never in constructor
};
 
// Static-only allocation
static MotorController motorLeft;
static MotorController motorRight;
static PidController   anglePid;
static PidController   speedPidLeft;
static PidController   speedPidRight;
static KalmanFilter    kalman;
 
// Performance-critical paths: direct register access
inline int32_t Encoder::getCount() const {
    return static_cast<int32_t>(htim_->Instance->CNT);
}
 
// constexpr instead of #define
constexpr float  SAMPLE_RATE_HZ     = 1000.0f;
constexpr float  SAMPLE_PERIOD_S    = 1.0f / SAMPLE_RATE_HZ;
constexpr float  COMPLEMENTARY_ALPHA = 0.98f;
constexpr float  PWM_MAX_DUTY       = 0.95f;
 
// enum class for all state/direction types
enum class MotorDirection { Forward, Backward, Coast, Brake };
enum class RobotState     { Balancing, Fallen, Initializing, Error };
```
 
---
 
## Communication Protocol (UART between boards)
 
Simple binary framing with COBS or start/stop bytes.
 
```cpp
// Telemetry packet: F756ZG → L432KC, sent every 50 ms
struct TelemetryPacket {
    float    tilt_angle_deg;
    float    tilt_rate_dps;
    float    speed_left_rpm;
    float    speed_right_rpm;
    float    pwm_duty_left;
    float    pwm_duty_right;
    float    pid_output_angle;
    RobotState state;
    uint8_t  checksum;
} __attribute__((packed));
 
// Parameter packet: L432KC → F756ZG (on user change)
struct ParamPacket {
    float   kp_angle, ki_angle, kd_angle;
    float   kp_speed, ki_speed, kd_speed;
    float   target_speed_mps;
    uint8_t checksum;
} __attribute__((packed));
```
 
---
 
## Project Structure
 
```
stm32-self-balancing-robot/
├── F756ZG/                         # Main control firmware
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── main.h
│   │   │   ├── FreeRTOSConfig.h
│   │   │   └── robot_config.h      # All constexpr tuning parameters
│   │   └── Src/
│   │       └── main.cpp
│   ├── Drivers/                    # STM32 HAL + CMSIS (generated)
│   └── App/                        # Application layer
│       ├── Inc/
│       │   ├── imu/
│       │   │   ├── Bmi270.hpp
│       │   │   └── KalmanFilter.hpp
│       │   ├── control/
│       │   │   ├── PidController.hpp
│       │   │   └── BalanceController.hpp
│       │   ├── motors/
│       │   │   ├── MotorController.hpp
│       │   │   └── Encoder.hpp
│       │   └── comms/
│       │       └── UartComms.hpp
│       └── Src/
│           ├── imu/...
│           ├── control/...
│           ├── motors/...
│           └── comms/...
│
├── L432KC/                         # Interface firmware
│   ├── Core/...
│   └── App/
│       ├── display/
│       │   ├── TftDriver.hpp
│       │   └── MenuSystem.hpp
│       └── comms/
│           └── UartComms.hpp
│
├── docs/
│   ├── architecture.md
│   ├── pid_tuning.md
│   └── wiring_diagram.md
│
└── README.md
```
 
---
 
## What NOT to suggest
 
- No dynamic memory allocation (`new`, `delete`, heap-based containers)
- No C++ exceptions or RTTI
- No `std::function`, `std::string`, `std::vector`
- No virtual functions in ISR or PID/motor control paths
- No HAL for performance-critical single operations (encoder read, GPIO toggle in ISR) — use direct register access
- Do not mix LL and HAL for the same peripheral
- No CAN bus (not used in this project)
- No Bluetooth / WiFi modules
---
 
## Style Guide
 
- Filenames: `PascalCase.hpp` / `PascalCase.cpp`
- Classes: `PascalCase`
- Methods: `camelCase`
- Constants/constexpr: `UPPER_SNAKE_CASE`
- Member variables: `camelCase_` (trailing underscore)
- All public methods documented with Doxygen `/** */` comments
- `#pragma once` instead of include guards
