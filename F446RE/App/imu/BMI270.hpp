#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"
#include "BMI270_SensorAPI/bmi270.h"
#include "robot_config.hpp"

//static constexpr uint16_t bmi270_i2c_addr = (BMI2_I2C_PRIM_ADDR << 1);
static constexpr uint16_t bmi270_i2c_addr = (BMI2_I2C_SEC_ADDR << 1);

class BMI270 {
private:
	I2C_HandleTypeDef* hi2c_;
	SPI_HandleTypeDef* hspi_;
	struct bmi2_dev dev_ = {0};
	struct bmi2_sens_data sensor_data_ = {0};

	uint8_t dev_addr_ = 0;
	uint8_t bus_inst_ = 0;

	float acc_x_{0}, acc_y_{0}, acc_z_{0};
	float gyr_x_{0}, gyr_y_{0}, gyr_z_{0};

public:
	BMI270(I2C_HandleTypeDef* hi2c,  SPI_HandleTypeDef* hspi);

	int8_t init();
	void update();
	int8_t set_accel_gyro_config();

	float getAccX() const { return acc_x_; }
	float getAccY() const { return acc_y_; }
	float getAccZ() const { return acc_z_; }
	float getGyrX() const { return gyr_x_; }
	float getGyrY() const { return gyr_y_; }
	float getGyrZ() const { return gyr_z_; }

	static BMI2_INTF_RETURN_TYPE i2cRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
	static BMI2_INTF_RETURN_TYPE i2cWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
	static BMI2_INTF_RETURN_TYPE spiRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
	static BMI2_INTF_RETURN_TYPE spiWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

	static void bmi2_delay_us(uint32_t period, void *intf_ptr);


	/*!
	 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
	 * range 2G, 4G, 8G or 16G.
	 */
	static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

	/*!
	 * @brief This function converts lsb to degree per second for 16 bit gyro at
	 * range 125, 250, 500, 1000 or 2000dps.
	 */
	static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
};
