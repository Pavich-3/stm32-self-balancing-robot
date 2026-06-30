#pragma once

#include <cstdint>

class ComplementaryFilter {
private:
	static constexpr float alpha_ = 0.98f;
	float currentAngle_{0.0f};

public:
	ComplementaryFilter() = default;

	float update(float gyroX, float accX, float accY, float accZ, float dt = 0.01f);
};
