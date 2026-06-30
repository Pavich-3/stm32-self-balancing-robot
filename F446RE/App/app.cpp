#include "app.hpp"
#include "stm32f4xx_hal.h"
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

static PIDController pid{15.0f, 0.0f, 0.0f};
static MotorController motor1{&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2};
static MotorController motor2{&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4};
static Encoder encoderMotor1{&htim3};
static Encoder encoderMotor2{&htim4};

float angle = 0;
float output = 0;

uint8_t appInit() {
	MPU6050CALLBACK result = mpu6050.init();
	if (result == MPU6050CALLBACK::MPU6050_OK) {
		mpu6050.calibrate(100);
	} else {
		return 1;
	}
	motor1.init();
	motor2.init();
	encoderMotor1.init();
	encoderMotor2.init();

	return 0;
}

void appRun() {
	mpu6050.update();
	float accX = mpu6050.getAccX();
	float accY = mpu6050.getAccY();
	float accZ = mpu6050.getAccZ();

	float gyrX = mpu6050.getGyrX();

	angle = kalmanFilter.update(gyrX, accX, accY, accZ, CONTROL_PERIOD_S);
	if (fabs(angle) < SAFETY_FALL_ANGLE_THRESHOLD) {
		float error = 0.0f - angle;
		output = pid.update(error, CONTROL_PERIOD_S);

		motor1.setDuty(output);
		motor2.setDuty(-output);
	} else {
		motor1.stop();
		motor2.stop();
	}
}
