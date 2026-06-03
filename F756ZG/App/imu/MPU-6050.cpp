#include "MPU-6050.hpp"
#include "stm32f7xx_hal.h"

static constexpr uint16_t mpu6050Address = (0b1101000 << 1);
//static constexpr uint16_t mpu6050Address = (b1101001 << 1);
static constexpr uint8_t MPU6050_WHO_I_AM = 0x68;

MPU6050::MPU6050(I2C_HandleTypeDef* hi2c) : hi2c_(hi2c) {}

MPU6050CALLBACK MPU6050::i2cWrite(uint16_t reg, uint8_t * data, uint16_t len) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c_, mpu6050Address, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
	if (status == HAL_OK) {
		return MPU6050CALLBACK::MPU6050_OK;
	} else {
		return MPU6050CALLBACK::MPU6050_I2C_ERROR;
	}
}

MPU6050CALLBACK MPU6050::i2cRead(uint16_t reg, uint8_t *data, uint16_t len) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c_, mpu6050Address, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
	if (status == HAL_OK) {
		return MPU6050CALLBACK::MPU6050_OK;
	} else {
		return MPU6050CALLBACK::MPU6050_I2C_ERROR;
	}
}

MPU6050CALLBACK MPU6050::init() {
	MPU6050CALLBACK status;
	uint8_t cheapID = 0;
	status = i2cRead(MPU6050REG::MPU6050_WHO_I_AM_ADDR, &cheapID, 1);
	if (status == MPU6050CALLBACK::MPU6050_I2C_ERROR) {
		return status;
	}
	if (cheapID != MPU6050_WHO_I_AM) {
		return MPU6050CALLBACK::MPU6050_INVALID_CHIP_ID;
	}

	uint8_t reset = MPU6050REG::PWR_MGMT_1::MPU6050_DEVICE_RESET;
	status = i2cWrite(MPU6050REG::MPU6050_PWR_MGMT_1_ADDR, &reset, 1);
	HAL_Delay(100);

	uint8_t pathReset = (
			MPU6050REG::SIGNAL_PATH_RESET::MPU6050_ACCEL_RESET |
			MPU6050REG::SIGNAL_PATH_RESET::MPU6050_GYRO_RESET |
			MPU6050REG::SIGNAL_PATH_RESET::MPU6050_TEMP_RESET
		);
	status = i2cWrite(MPU6050REG::MPU6050_SIGNAL_PATH_RESET_ADDR, &pathReset, 1);
	HAL_Delay(100);

	uint8_t powerManagment = MPU6050REG::PWR_MGMT_1::CLKSEL;
	status = i2cWrite(MPU6050REG::MPU6050_PWR_MGMT_1_ADDR, &powerManagment, 1);

	uint8_t config = (MPU6050REG::CONFIG::EXT_SYNC_SET | MPU6050REG::CONFIG::DLPF_CFG);
	status = i2cWrite(MPU6050REG::MPU6050_CONFIGURATION_ADDR, &config, 1);

	uint8_t gyroConfig = MPU6050REG::GYRO_CONFIG::FS_SEL;
	status = i2cWrite(MPU6050REG::MPU6050_GYRO_CONFIG_ADDR, &gyroConfig, 1);

	uint8_t accConfig = MPU6050REG::ACCEL_CONFIG::AFS_SEL;
	status = i2cWrite(MPU6050REG::MPU6050_ACCEL_CONFIG_ADDR, &accConfig, 1);

	uint8_t smplrt = 4;
	status = i2cWrite(MPU6050REG::MPU6050_SMPLRT_DIV_ADDR, &smplrt, 1);

	uint8_t int_enable = MPU6050REG::INT_ENABLE::DATA_RDY_EN;
	status = i2cWrite(MPU6050REG::MPU6050_INT_ENABLE_ADDR, &int_enable, 1);

	return MPU6050CALLBACK::MPU6050_OK;
}

void MPU6050::update() {
	MPU6050CALLBACK status = i2cRead(MPU6050REG::MPU6050_ACCEL_XOUT_H_ADDR, rawData_, static_cast<uint16_t>(len));
	if (status == MPU6050CALLBACK::MPU6050_I2C_ERROR) {
		return;
	}

	int16_t acc_x = ((rawData_[0] << 8) | rawData_[1]);
	int16_t acc_y = ((rawData_[2] << 8) | rawData_[3]);
	int16_t acc_z = ((rawData_[4] << 8) | rawData_[5]);
	int16_t gyr_x = ((rawData_[8] << 8) | rawData_[9]);
	int16_t gyr_y = ((rawData_[10] << 8) | rawData_[11]);
	int16_t gyr_z = ((rawData_[12] << 8) | rawData_[13]);

	acc_x_ = acc_x / ACC_SCALE_FACTOR * GRAVITY_MS2;
	acc_y_ = acc_y / ACC_SCALE_FACTOR * GRAVITY_MS2;
	acc_z_ = acc_z / ACC_SCALE_FACTOR * GRAVITY_MS2;

	gyr_x_ = gyr_x / GYR_SCALE_FACTOR;
	gyr_y_ = gyr_y / GYR_SCALE_FACTOR;
	gyr_z_ = gyr_z / GYR_SCALE_FACTOR;

}

