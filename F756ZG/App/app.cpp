#include "app.hpp"
#include "stm32f7xx_hal.h"
#include "imu/BMI270.hpp"

extern I2C_HandleTypeDef hi2c1;

static BMI270 bmi270{&hi2c1, nullptr};

uint8_t appInit() {
	uint8_t reslt = bmi270.init();
	printf("rslt: %d\n", (int)reslt);

	if (reslt == BMI2_OK) {
		return 1;
	} else {
		return 0;
	}
}

void appRun(float* accX, float* accY, float* accZ, float* gyrX, float* gyrY, float* gyrZ) {
	bmi270.update();
	*accX = bmi270.getAccX();
	*accY = bmi270.getAccY();
	*accZ = bmi270.getAccZ();

	*gyrX = bmi270.getGyrX();
	*gyrY = bmi270.getGyrY();
	*gyrZ = bmi270.getGyrZ();
}
