#pragma once

class KalmanFilter {
private:
	static constexpr float Q_ = 0.001f;
	static constexpr float R_ = 0.03f;

	float x_ = 0.0f, P_=1.0f;

public:
	float update(float gyroX, float accX, float accY, float accZ, float dt=0.01f);
};
