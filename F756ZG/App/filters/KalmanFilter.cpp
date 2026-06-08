#include "KalmanFilter.hpp"
#include <cmath>

float KalmanFilter::update(float gyroX, float accX, float accY, float accZ, float dt) {
	float accAngle = (180.0f / M_PI) * atan(accX / sqrt(pow(accY, 2) + pow(accZ, 2)));

	x_ += gyroX * dt;
	P_ += Q_;

	float K = P_ / (P_ + R_);
	x_ += K * (accAngle - x_);
	P_ = (1 - K) * P_;

	return x_;
}
