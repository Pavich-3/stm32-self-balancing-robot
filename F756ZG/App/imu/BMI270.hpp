#pragma once

#include <cstdint>
#include "stm32f7xx_hal.h"
#include "BMI270_SensorAPI/bmi270.h" 
#include "robot_config.hpp"

static constexpr uint16_t bmi270_i2c_addr = (BMI2_I2C_PRIM_ADDR << 1);
//static constexpr uint16_t bmi270_i2c_addr = (BMI2_I2C_SEC_ADDR << 1);

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
	BMI270(I2C_HandleTypeDef* hi2c,  SPI_HandleTypeDef* hspi) : hi2c_(hi2c), hspi_(hspi) {

	}

	int8_t init() {
		int8_t rslt = BMI2_OK;
			/* Bus configuration : I2C */
		if (hi2c_ != nullptr)
		{
			/* To initialize the user I2C function */
			dev_addr_ = BMI2_I2C_PRIM_ADDR;
			dev_.intf = BMI2_I2C_INTF;
			dev_.read = i2cRead;
			dev_.write = i2cWrite;
			dev_.intf_ptr = ((void *)hi2c_);
		}
		/* Bus configuration : SPI */
		else if (hspi_ != nullptr)
		{
			/* To initialize the user SPI function */
			dev_.intf = BMI2_SPI_INTF;
			dev_.read = spiRead;
			dev_.write = spiWrite;
			dev_.intf_ptr = ((void *)hspi_);
		} else {
			rslt = BMI2_E_NULL_PTR;
			return rslt;
		}

		/* Configure delay in microseconds */
		dev_.delay_us = bmi2_delay_us;

		/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
		dev_.read_write_len = READ_WRITE_LEN;

		/* Assign to NULL to load the default config file. */
		dev_.config_file_ptr = NULL;

		uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

		rslt = bmi270_init(&dev_);

		rslt = bmi2_sensor_enable(sensor_list, 2, &dev_);

		return rslt;
	}

	void update() {
		uint8_t rslt = bmi2_get_sensor_data(&sensor_data_, &dev_);

		if ((sensor_data_.status & BMI2_DRDY_ACC) && (sensor_data_.status & BMI2_DRDY_GYR)) {
			/* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
			acc_x_ = lsb_to_mps2(sensor_data_.acc.x, (float)2, dev_.resolution);
			acc_y_ = lsb_to_mps2(sensor_data_.acc.y, (float)2, dev_.resolution);
			acc_z_ = lsb_to_mps2(sensor_data_.acc.z, (float)2, dev_.resolution);

			/* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
			gyr_x_ = lsb_to_dps(sensor_data_.gyr.x, (float)2000, dev_.resolution);
			gyr_y_ = lsb_to_dps(sensor_data_.gyr.y, (float)2000, dev_.resolution);
			gyr_z_ = lsb_to_dps(sensor_data_.gyr.z, (float)2000, dev_.resolution);
		}
	}

	int8_t set_accel_gyro_config()
	{
	    /* Status of api are returned to this variable. */
	    int8_t rslt;

	    /* Structure to define accelerometer and gyro configuration. */
	    struct bmi2_sens_config config[2];

	    /* Configure the type of feature. */
	    config[ACCEL].type = BMI2_ACCEL;
	    config[GYRO].type = BMI2_GYRO;

	    /* Get default configurations for the type of feature selected. */
	    rslt = bmi2_get_sensor_config(config, sizeof(config) / sizeof(config[0]), &dev_);

	    /* Map data ready interrupt to interrupt pin. */
	    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &dev_);

	    if (rslt == BMI2_OK)
	    {
	        /* NOTE: The user can change the following configuration parameters according to their requirement. */
	        /* Set Output Data Rate */
	        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_200HZ;

	        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
	        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

	        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
	         * if it is set to 2, then 2^(bandwidth parameter) samples
	         * are averaged, resulting in 4 averaged samples.
	         * Note1 : For more information, refer the datasheet.
	         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
	         * this has an adverse effect on the power consumed.
	         */
	        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

	        /* Enable the filter performance mode where averaging of samples
	         * will be done based on above set bandwidth and ODR.
	         * There are two modes
	         *  0 -> Ultra low power mode
	         *  1 -> High performance mode(Default)
	         * For more info refer datasheet.
	         */
	        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

	        /* The user can change the following configuration parameters according to their requirement. */
	        /* Set Output Data Rate */
	        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;

	        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
	        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

	        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
	        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

	        /* Enable/Disable the noise performance mode for precision yaw rate sensing
	         * There are two modes
	         *  0 -> Ultra low power mode(Default)
	         *  1 -> High performance mode
	         */
	        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

	        /* Enable/Disable the filter performance mode where averaging of samples
	         * will be done based on above set bandwidth and ODR.
	         * There are two modes
	         *  0 -> Ultra low power mode
	         *  1 -> High performance mode(Default)
	         */
	        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

	        /* Set the accel and gyro configurations. */
	        rslt = bmi2_set_sensor_config(config, 2, &dev_);
	    }

	    return rslt;
	}

	float getAccX() const {
		return acc_x_;
	}

	float getAccY() const {
		return acc_y_;
	}

	float getAccZ() const {
		return acc_z_;
	}

	float getGyrX() const {
		return gyr_x_;
	}

	float getGyrY() const {
		return gyr_y_;
	}

	float getGyrZ() const {
		return gyr_z_;
	}

	static BMI2_INTF_RETURN_TYPE i2cRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
		I2C_HandleTypeDef* hi2c = static_cast<I2C_HandleTypeDef*>(intf_ptr);
		HAL_I2C_Mem_Read(hi2c, bmi270_i2c_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, HAL_MAX_DELAY);

		return BMI2_OK;
	}

	static BMI2_INTF_RETURN_TYPE i2cWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
		I2C_HandleTypeDef* hi2c = static_cast<I2C_HandleTypeDef*>(intf_ptr);
		HAL_I2C_Mem_Write(hi2c, bmi270_i2c_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, HAL_MAX_DELAY);

		return BMI2_OK;
	}

	static BMI2_INTF_RETURN_TYPE spiRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
		SPI_HandleTypeDef* hspi = static_cast<SPI_HandleTypeDef*>(intf_ptr);
		HAL_SPI_Transmit(hspi, &reg_addr, 1, HAL_MAX_DELAY);
		HAL_SPI_Receive(hspi, reg_data, static_cast<uint16_t>(len), HAL_MAX_DELAY);

		return BMI2_OK;
	}

	static BMI2_INTF_RETURN_TYPE spiWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
		SPI_HandleTypeDef* hspi = static_cast<SPI_HandleTypeDef*>(intf_ptr);
		HAL_SPI_Transmit(hspi, &reg_addr, 1, HAL_MAX_DELAY);
		HAL_SPI_Transmit(hspi, reg_data, static_cast<uint16_t>(len), HAL_MAX_DELAY);

		return BMI2_OK;
	}

	static void bmi2_delay_us(uint32_t period, void *intf_ptr)
	{
	    HAL_Delay((period + 999) / 1000);
	}


	/*!
	 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
	 * range 2G, 4G, 8G or 16G.
	 */
	static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
	{
	    double power = 2;

	    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

	    return (GRAVITY_EARTH * val * g_range) / half_scale;
	}

	/*!
	 * @brief This function converts lsb to degree per second for 16 bit gyro at
	 * range 125, 250, 500, 1000 or 2000dps.
	 */
	static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
	{
	    double power = 2;

	    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

	    return (dps / (half_scale)) * (val);
	}
};
