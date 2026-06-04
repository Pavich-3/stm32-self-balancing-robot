#include "ComplementaryFilter.hpp"

#include <cmath>

float ComplementaryFilter::update(float gyroX, float accX, float accY, float accZ, float dt) {
	float accAngle = (180.0f / M_PI) * atan(accX / sqrt(pow(accY, 2) + pow(accZ, 2)));

	float angle = alpha_ * (currentAngle_ + gyroX * dt) + (1.0f - alpha_) * accAngle;
	currentAngle_ = angle;

	return angle;
}
