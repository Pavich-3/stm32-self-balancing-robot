#include "app.hpp"
#include "stm32f7xx_hal.h"
#include "imu/MPU-6050.hpp"
#include "filters/ComplementaryFilter.hpp"
#include "filters/KalmanFilter.hpp"

extern I2C_HandleTypeDef hi2c1;

static MPU6050 mpu6050{&hi2c1};
static ComplementaryFilter complementaryFilter;
static KalmanFilter kalmanFilter;

uint8_t appInit() {
	MPU6050CALLBACK result = mpu6050.init();
	if (result == MPU6050CALLBACK::MPU6050_OK) {
		mpu6050.calibrate(100);
		return 0;
	} else {
		return 1;
	}
}

void appRun(float* accX, float* accY, float* accZ, float* gyrX, float* gyrY, float* gyrZ, float* angle) {
	mpu6050.update();
	*accX = mpu6050.getAccX();
	*accY = mpu6050.getAccY();
	*accZ = mpu6050.getAccZ();

	*gyrX = mpu6050.getGyrX();
	*gyrY = mpu6050.getGyrY();
	*gyrZ = mpu6050.getGyrZ();

	*angle = complementaryFilter.update(*gyrX, *accX, *accY, *accZ);
}
