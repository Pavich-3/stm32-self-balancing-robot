#include "PIDController.hpp"

PIDController::PIDController(float Kp, float Ki, float Kd) : Kp_(Kp), Ki_(Ki), Kd_(Kd) {}

float PIDController::update(float error, float dt) {
	integral_ += error * dt;

	if (integral_ > INTEGRAL_LIMIT) integral_ = INTEGRAL_LIMIT;
	if (integral_ < -INTEGRAL_LIMIT) integral_ = -INTEGRAL_LIMIT;

	float derivative = (error - prevError_) / dt;
	float pidError = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
	prevError_ = error;

	return pidError;
}
