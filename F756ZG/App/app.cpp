#include "app.hpp"
#include "stm32f7xx_hal.h"
#include "imu/MPU-6050.hpp"
#include "filters/ComplementaryFilter.hpp"
#include "filters/KalmanFilter.hpp"
#include "control/PIDController.hpp"
#include "motors/Encoder.hpp"
#include "motors/MotorController.hpp"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static MPU6050 mpu6050{&hi2c1};
static ComplementaryFilter complementaryFilter;
static KalmanFilter kalmanFilter;
static PIDController pid{1.0f, 0.0f, 0.0f};
static MotorController motor1{&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2};
static MotorController motor2{&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4};
static Encoder encoderMotor1{&htim3};
static Encoder ecndoerMotor2{&htim4};

static float prevTick = 0.0f;

uint8_t appInit() {
	MPU6050CALLBACK result = mpu6050.init();
	if (result == MPU6050CALLBACK::MPU6050_OK) {
		mpu6050.calibrate(100);
		return 0;
	} else {
		return 1;
	}
}

void appRun(float* accX, float* accY, float* accZ, float* gyrX, float* gyrY, float* gyrZ, float* angle, float* output) {
	if (prevTick == 0) {
	    prevTick = HAL_GetTick();
	    return;
	}

	mpu6050.update();
	*accX = mpu6050.getAccX();
	*accY = mpu6050.getAccY();
	*accZ = mpu6050.getAccZ();

	*gyrX = mpu6050.getGyrX();
	*gyrY = mpu6050.getGyrY();
	*gyrZ = mpu6050.getGyrZ();

	uint32_t currTick = HAL_GetTick();
	float dt = (float)(currTick - prevTick) / 1000.0f;
	prevTick = currTick;

	*angle = complementaryFilter.update(*gyrX, *accX, *accY, *accZ, dt);

	float error = 0.0f - *angle;
	*output = pid.update(error, dt);

	motor1.init();
	motor1.setDuty(100.0f);
}
