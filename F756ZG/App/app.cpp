#include "app.hpp"
#include "stm32f7xx_hal.h"
#include "imu/BMI270.hpp"

extern I2C_HandleTypeDef hi2c1;

static BMI270 bmi270{&hi2c1, nullptr};

uint8_t appInit() {
	uint8_t reslt = bmi270.init();

	if (reslt == BMI2_OK) {
		return 1;
	} else {
		return 0;
	}
}
