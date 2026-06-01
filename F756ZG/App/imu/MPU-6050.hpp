#pragma once

#include <cstdint>
#include "stm32f7xx_hal.h"
#include "robot_config.hpp"

#define MPU6050_WHO_I_AM 0x68

#define MPU6050_WHO_I_AM_ADDR 0x75
#define MPU6050_PWR_MGMT_1_ADDR 0x6B
#define MPU6050_SIGNAL_PATH_RESET_ADDR 0x68
#define MPU6050_CONFIGURATION_ADDR 0x1A
#define MPU6050_GYRO_CONFIG_ADDR 0x1B
#define MPU6050_ACCEL_CONFIG_ADDR 1C

static constexpr uint16_t mpu6050Address = (b1101000 << 1);
//static constexpr uint16_t mpu6050Address = (b1101001 << 1);

static enum class MPU6050CLKSEL : uint8_t {
	INTERNAL = 0,
	PLL_X = 1,
	PLL_Y = 2,
	PLL_Z = 3,
	PLL_EXTERNAL_32_768KHZ = 4,
	PLL_EXTERNAL_19_2MHz = 5,
	STOP_CLOCK = 7
};

static enum class PWR_MGMT_1 : uint8_t {
	MPU6050_DEVICE_RESET = (1 << 7),
	MPU6050_SLEEP = (1 << 6),
	MPU6050_CYCLE = (1 << 5),
	TEMP_DIS = (1 << 3),
	CLKSEL = MPU6050CLKSEL::PLL_X
};

static enum class SIGNAL_PATH_RESET : uint8_t {
	MPU6050_GYRO_RESET = (1 << 2),
	MPU6050_ACCEL_RESET = (1 << 1),
	MPU6050_TEMP_RESET = 0
};

static enum class MPU6050_EXT_SYNC_SET : uint8_t {
	INPUT_DISABLED = 0,
	TEMP_OUT_L = (1 << 3),
	GYRO_XOUT_L = (2 << 3),
	GYRO_YOUT_L = (3 << 3),
	GYRO_ZOUT_L = (4 << 3),
	ACCEL_XOUT_L = (5 << 3),
	ACCEL_YOUT_L = (6 << 3),
	ACCEL_ZOUT_L = (7 << 3)
};

static enum class MPU6050_DLPF_CFG : uint8_t {
	DLPF_CFG_0 = 0,
	DLPF_CFG_1 = 1,
	DLPF_CFG_2 = 2,
	DLPF_CFG_3 = 3,
	DLPF_CFG_4 = 4,
	DLPF_CFG_5 = 5,
	DLPF_CFG_6 = 6,
	DLPF_CFG_7 = 7
};

static enum class CONFIG : uint8_t {
	EXT_SYNC_SET = INPUT_DISABLED,
	DLPF_CFG = MPU6050_DLPF_CFG::DLPF_CFG_2
};

static enum class MPU6050_FS_SEL : uint8_t {
	FS_SEL_250 = 0,
	FS_SEL_500 = 1,
	FS_SEL_1000 = 2,
	FS_SEL_2000 = 3
};

static enum class GYRO_CONFIG : uint8_t {
	XG_ST = (1 << 7),
	YG_ST = (1 << 6),
	ZG_ST = (1 << 5),
	FS_SEL = (MPU6050_FS_SEL::FS_SEL_500 << 3)
};

static enum class MPU6050_AFS_SEL : uint8_t {
	AFS_SEL_2 = 0,
	AFS_SEL_4 = 1,
	AFS_SEL_8 = 2,
	AFS_SEL_16 = 3
};

static enum class ACCEL_CONFIG : uint8_t {
	XA_ST = (1 << 7),
	YA_ST = (1 << 6),
	ZA_ST = (1 << 5),
	AFS_SEL = (MPU6050_AFS_SEL::AFS_SEL_4 << 3)
};

class MPU6050 {
public:
	I2C_HandleTypeDef *hi2c_;

	uint8_t rawData_[14] = {0};

	float acc_x_{0}, acc_y_{0}, acc_z_{0};
	float gyr_x_{0}, gyr_y_{0}, gyr_z_{0};

	void i2cWrite(uint16_t reg, uint8_t * data, int16_t len);
	void i2cRead(uint16_t reg, uint8_t *data, uint16_t len);

private:
	MPU6050(I2C_HandleTypeDef* hi2c);

	void init();
	void update();

	float getAccX() const { return acc_x; }
	float getAccY() const { return acc_y; }
	float getAccZ() const { return acc_z; }

	float getGyrX() const { return acc_x; }
	float getGyrY() const { return gyr_y; }
	float getGyrZ() const { return gyr_z; }
};
