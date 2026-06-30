#pragma once

#include "stm32f4xx_hal.h"
#include "robot_config.hpp"

class MotorController {
private:
    TIM_HandleTypeDef* htim_;
    uint32_t channelIN1_;
    uint32_t channelIN2_;

public:
    explicit MotorController(TIM_HandleTypeDef* htim, uint32_t channelIN1, uint32_t channelIN2) : htim_(htim), channelIN1_(channelIN1), channelIN2_(channelIN2) {}

    void init() {
        HAL_TIM_PWM_Start(htim_, channelIN1_);
        HAL_TIM_PWM_Start(htim_, channelIN2_);
    }

    void setDuty(float duty) {
        uint32_t CCR = fabsf(duty) * PWM_ARR;
        if (duty > 0.0f) {
            __HAL_TIM_SET_COMPARE(htim_, channelIN1_, CCR);
            __HAL_TIM_SET_COMPARE(htim_, channelIN2_, 0);
        } else if (duty < 0.0f) {
            __HAL_TIM_SET_COMPARE(htim_, channelIN1_, 0);
            __HAL_TIM_SET_COMPARE(htim_, channelIN2_, CCR);
        } else {
            stop();
        }
    }

    void stop() {
        __HAL_TIM_SET_COMPARE(htim_, channelIN1_, 0);
        __HAL_TIM_SET_COMPARE(htim_, channelIN2_, 0);
    }
};
