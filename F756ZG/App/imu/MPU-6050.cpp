#include "MPU-6050.hpp"

MPU6050::MPU6050(I2C_HandleTypeDef* hi2c) : hi2c_(hi2c) {}

void MPU6050::init() {
	uint8_t cheapID = 0;
	i2cRead(MPU6050_WHO_I_AM_ADDR, &cheapID, 1);
	if (cheapID != MPU6050_WHO_I_AM) {
		return;
	}

	i2cWrite(MPU6050_PWR_MGMT_1_ADDR, PWR_MGMT_1::MPU6050_DEVICE_RESET, 1);

	uint8_t pathReset = (
			SIGNAL_PATH_RESET::MPU6050_ACCEL_RESET |
			SIGNAL_PATH_RESET::MPU6050_GYRO_RESET |
			SIGNAL_PATH_RESET::MPU6050_TEMP_RESET
		);
	i2cWrite(MPU6050_SIGNAL_PATH_RESET_ADDR, &pathReset, 1);

	uint8_t powerManagment = (PWR_MGMT_1::MPU6050_CYCLE | PWR_MGMT_1::CLKSEL);
	i2cWrite(MPU6050_PWR_MGMT_1_ADDR, &powerManagment, 1);

	uint8_t config = (CONFIG::EXT_SYNC_SET | CONFIG::DLPF_CFG);
	i2cWrite(MPU6050_CONFIGURATION_ADDR, &config, 1);

	uint8_t gyroConfig = GYRO_CONFIG::FS_SEL;
	i2cWrite(MPU6050_GYRO_CONFIG_ADDR, &gyroConfig, 1);

	uint8_t accConfig = ACCEL_CONFIG::AFS_SEL;
	i2cWrite(MPU6050_ACCEL_CONFIG_ADDR, &accConfig, 1);
}

void MPU6050::i2cWrite(uint16_t reg, uint8_t * data, int16_t len) {
	HAL_I2C_Mem_Write(hi2c_, mpu6050Address, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void MPU6050::i2cRead(uint16_t reg, uint8_t *data, uint16_t len) {
	HAL_I2C_Mem_Read(hi2c_, mpu6050Address, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}
