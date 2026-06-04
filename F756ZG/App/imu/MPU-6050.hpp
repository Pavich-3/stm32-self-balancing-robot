#pragma once

#include <cstdint>
#include "stm32f7xx_hal.h"
#include "robot_config.hpp"

enum class MPU6050CALLBACK : int8_t {
	MPU6050_OK,
	MPU6050_INVALID_CHIP_ID,
	MPU6050_I2C_ERROR
};

namespace MPU6050REG {

	constexpr uint8_t MPU6050_WHO_I_AM_ADDR = 0x75;
	constexpr uint8_t MPU6050_PWR_MGMT_1_ADDR = 0x6B;
	constexpr uint8_t MPU6050_SIGNAL_PATH_RESET_ADDR = 0x68;
	constexpr uint8_t MPU6050_CONFIGURATION_ADDR = 0x1A;
	constexpr uint8_t MPU6050_GYRO_CONFIG_ADDR = 0x1B;
	constexpr uint8_t MPU6050_ACCEL_CONFIG_ADDR = 0x1C;
	constexpr uint8_t MPU6050_SMPLRT_DIV_ADDR = 0x19;
	constexpr uint8_t MPU6050_INT_ENABLE_ADDR = 0x38;
	constexpr uint8_t MPU6050_ACCEL_XOUT_H_ADDR = 0x3B;
	constexpr uint8_t MPU6050_ACCEL_XOUT_L_ADDR = 0x3C;
	constexpr uint8_t MPU6050_ACCEL_YOUT_H_ADDR = 0x3D;
	constexpr uint8_t MPU6050_ACCEL_YOUT_L_ADDR = 0x3E;
	constexpr uint8_t MPU6050_ACCEL_ZOUT_H_ADDR = 0x3F;
	constexpr uint8_t MPU6050_ACCEL_ZOUT_L_ADDR = 0x40;
	constexpr uint8_t MPU6050_ACCEL_TEMP_OUT_H_ADDR = 0x41;
	constexpr uint8_t MPU6050_ACCEL_TEMP_OUT_L_ADDR = 0x42;
	constexpr uint8_t MPU6050_GYRO_XOUT_H = 0x43;
	constexpr uint8_t MPU6050_GYRO_XOUT_L = 0x44;
	constexpr uint8_t MPU6050_GYRO_YOUT_H = 0x45;
	constexpr uint8_t MPU6050_GYRO_YOUT_L = 0x46;
	constexpr uint8_t MPU6050_GYRO_ZOUT_H = 0x47;
	constexpr uint8_t MPU6050_GYRO_ZOUT_L = 0x48;

	enum class MPU6050CLKSEL : uint8_t {
		INTERNAL = 0,
		PLL_X = 1,
		PLL_Y = 2,
		PLL_Z = 3,
		PLL_EXTERNAL_32_768KHZ = 4,
		PLL_EXTERNAL_19_2MHz = 5,
		STOP_CLOCK = 7
	};

	namespace PWR_MGMT_1 {
		constexpr uint8_t MPU6050_DEVICE_RESET = (1 << 7);
		constexpr uint8_t MPU6050_SLEEP = (1 << 6);
		constexpr uint8_t MPU6050_CYCLE = (1 << 5);
		constexpr uint8_t TEMP_DIS = (1 << 3);
		constexpr uint8_t CLKSEL = static_cast<uint8_t>(MPU6050CLKSEL::PLL_X);
	}


	namespace SIGNAL_PATH_RESET {
		constexpr uint8_t MPU6050_GYRO_RESET = (1 << 2);
		constexpr uint8_t MPU6050_ACCEL_RESET = (1 << 1);
		constexpr uint8_t MPU6050_TEMP_RESET = (1 << 0);
	}

	enum class MPU6050_EXT_SYNC_SET : uint8_t {
		INPUT_DISABLED = 0,
		TEMP_OUT_L = (1 << 3),
		GYRO_XOUT_L = (2 << 3),
		GYRO_YOUT_L = (3 << 3),
		GYRO_ZOUT_L = (4 << 3),
		ACCEL_XOUT_L = (5 << 3),
		ACCEL_YOUT_L = (6 << 3),
		ACCEL_ZOUT_L = (7 << 3)
	};

	enum class MPU6050_DLPF_CFG : uint8_t {
		DLPF_CFG_0 = 0,
		DLPF_CFG_1 = 1,
		DLPF_CFG_2 = 2,
		DLPF_CFG_3 = 3,
		DLPF_CFG_4 = 4,
		DLPF_CFG_5 = 5,
		DLPF_CFG_6 = 6,
		DLPF_CFG_7 = 7
	};

	namespace CONFIG {
		constexpr uint8_t EXT_SYNC_SET = static_cast<uint8_t>(MPU6050_EXT_SYNC_SET::INPUT_DISABLED);
		constexpr uint8_t DLPF_CFG = static_cast<uint8_t>(MPU6050_DLPF_CFG::DLPF_CFG_2);
	}

	enum class MPU6050_FS_SEL : uint8_t {
		FS_SEL_250 = 0,
		FS_SEL_500 = 1,
		FS_SEL_1000 = 2,
		FS_SEL_2000 = 3
	};

	namespace GYRO_CONFIG {
		constexpr uint8_t XG_ST = (1 << 7);
		constexpr uint8_t YG_ST = (1 << 6);
		constexpr uint8_t ZG_ST = (1 << 5);
		constexpr uint8_t FS_SEL = (static_cast<uint8_t>(MPU6050_FS_SEL::FS_SEL_500) << 3);
	}

	enum class MPU6050_AFS_SEL : uint8_t {
		AFS_SEL_2 = 0,
		AFS_SEL_4 = 1,
		AFS_SEL_8 = 2,
		AFS_SEL_16 = 3
	};

	namespace ACCEL_CONFIG {
		constexpr uint8_t XA_ST = (1 << 7);
		constexpr uint8_t YA_ST = (1 << 6);
		constexpr uint8_t ZA_ST = (1 << 5);
		constexpr uint8_t AFS_SEL = (static_cast<uint8_t>(MPU6050_AFS_SEL::AFS_SEL_4) << 3);
	}

	namespace INT_ENABLE {
		constexpr uint8_t FIFO_OFLOW_EN = (1 << 4);
		constexpr uint8_t I2C_MST_INT_EN = (1 << 3);
		constexpr uint8_t DATA_RDY_EN = (1 << 0);
	}

}

class MPU6050 {
private:
	I2C_HandleTypeDef *hi2c_;
	static constexpr std::size_t len = (
		static_cast<std::size_t>(MPU6050REG::MPU6050_GYRO_ZOUT_L) -
		static_cast<std::size_t>(MPU6050REG::MPU6050_ACCEL_XOUT_H_ADDR) + 1
	);
	static constexpr float GYRO_SENSITIVITY[4] = { 131.0f, 65.5f, 32.8f, 16.4f };
	static constexpr float ACCEL_SENSITIVITY[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
	static constexpr float ACC_SCALE_FACTOR = ACCEL_SENSITIVITY[1];
	static constexpr float GYR_SCALE_FACTOR = GYRO_SENSITIVITY[1];

	uint8_t rawData_[len] = {0};

	int32_t accOffsetX_{0}, accOffsetY_{0}, accOffsetZ_{0};
	int32_t gyrOffsetX_{0}, gyrOffsetY_{0}, gyrOffsetZ_{0};
	float acc_x_{0}, acc_y_{0}, acc_z_{0};
	float gyr_x_{0}, gyr_y_{0}, gyr_z_{0};

	void readRawData();

	MPU6050CALLBACK i2cWrite(uint16_t reg, uint8_t * data, uint16_t len);
	MPU6050CALLBACK i2cRead(uint16_t reg, uint8_t *data, uint16_t len);

public:
	MPU6050(I2C_HandleTypeDef* hi2c);

	MPU6050CALLBACK init();
	void update();
	void calibrate(std::size_t samples);

	float getAccX() const { return acc_x_; }
	float getAccY() const { return acc_y_; }
	float getAccZ() const { return acc_z_; }

	float getGyrX() const { return gyr_x_; }
	float getGyrY() const { return gyr_y_; }
	float getGyrZ() const { return gyr_z_; }
};
