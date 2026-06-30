#pragma once

#include <cstdint>

class PIDController {
private:
	static constexpr float INTEGRAL_LIMIT = 1000.0f;
	float Kp_, Ki_, Kd_;
	float integral_ = 0.0f, prevError_ = 0.0f;

public:
	PIDController(float Kp, float Ki, float Kd);

	float update(float error, float dt=0.01f);
};

