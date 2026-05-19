#pragma once

#include <cmath>
#include "stm32f7xx_hal.h"
#include "robot_config.hpp"

class Encoder {
private:
    static constexpr uint16_t center = 32768;

    TIM_HandleTypeDef* htim_;
    float speedRPM_ = 0.0f;
    float speedMPS_ = 0.0f;
    int32_t totalCount_ = 0;

public:
    explicit Encoder(TIM_HandleTypeDef* htim) : htim_(htim) {}

    void init() {
        __HAL_TIM_SET_COUNTER(htim_, center);
        HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
    }

    void update() {
        int32_t delta = static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_)) - static_cast<int32_t>(center);
        totalCount_ += delta;
        __HAL_TIM_SET_COUNTER(htim_, center);

        speedRPM_ = (static_cast<float>(delta) / static_cast<float>(MOTOR_ENCODER_PPR)) * static_cast<float>(CONTROL_LOOP_HZ) * 60;
        speedMPS_ = speedRPM_ * M_PI * MOTOR_WHEEL_DIAMETER_MM / 1000.0f / 60.0f;
    }

    float getRPM() const { return speedRPM_; }
    float getMPS() const { return speedMPS_; }
    int32_t getTotalCount() const { return totalCount_; }
};
