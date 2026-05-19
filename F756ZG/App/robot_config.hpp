#pragma once

#include <cstdint>

constexpr uint16_t CONTROL_LOOP_HZ = 500u;
constexpr uint16_t CONTROL_TELEMETRY_HZ = 50u;
constexpr float CONTROL_PERIOD_S =  1.0f / CONTROL_LOOP_HZ;
constexpr uint16_t CONTROL_PERIOD_MS = 1000u / CONTROL_LOOP_HZ;

constexpr float IMU_COMPLEMENTARY_ALPHA = 0.996f;
constexpr float IMU_KALMAN_Q_ANGLE = 0.0f;
constexpr float IMU_KALMAN_Q_BIAS = 0.0f;
constexpr float IMU_KALMAN_R_MEASURE = 0.0f;

constexpr float PID_ANGLE_KP = 0.0f;
constexpr float PID_ANGLE_KI = 0.0f;
constexpr float PID_ANGLE_KD = 0.0f;
constexpr float PID_SPEED_KP = 0.0f;
constexpr float PID_SPEED_KI = 0.0f;
constexpr float PID_SPEED_KD = 0.0f;

constexpr float SAFETY_FALL_ANGLE_THRESHOLD = 30.0f;
constexpr float SAFETY_FALL_ANGLE_HYSTERESIS = 5.0f;

constexpr uint8_t MOTOR_ENCODER_PPR = 11;
constexpr float MOTOR_WHEEL_DIAMETER_MM = 67.3f;
constexpr float MOTOR_WHEEL_BASE_MM = 200.0f;

constexpr uint16_t PWM_FREQ_HZ = 20000u;
constexpr uint32_t PWM_TIM_TICK_HZ = 216000000u;

