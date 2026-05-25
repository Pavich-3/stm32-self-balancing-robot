#pragma once

#include <cstdint>
#include "stm32f7xx_hal.h"
#include "BMI270_SensorAPI/bmi270.h" 

class BMI270 {
private:
	I2C_HandleTypeDef* hi2c_;
	struct bmi2_dev* dev_;

public:
	BMI270(I2C_HandleTypeDef* hi2c_, struct bmi2_dev dev_) : hi2c_(hi2c_), dev_(dev_) {}

	void init() {
		result = bmi2_interface_init(dev_, BMI2_SPI_INTF);
		bmi2_error_codes_print_result(result);

		rslt = bmi270_init(dev_);
		bmi2_error_codes_print_result(rslt);
	}

	static int8_t bmi2_interface_init(uint8_t intf)
	{
	    int8_t rslt = BMI2_OK;

	    if (dev_ != NULL)
	    {
	        int16_t result = coines_board_init(COINES_COMM_INTF_USB, true);

	        if (result != COINES_SUCCESS) { return COINES_E_UNABLE_OPEN_DEVICE; }

	        /* Bus configuration : I2C */
	        if (intf == BMI2_I2C_INTF)
	        {
	            /* To initialize the user I2C function */
	            dev_addr = BMI2_I2C_PRIM_ADDR;
	            bmi->intf = BMI2_I2C_INTF;
	            bmi->read = bmi2_i2c_read;
	            bmi->write = bmi2_i2c_write;

	            /* SDO to Ground */
	            coines_set_pin_config(COINES_SHUTTLE_PIN_22, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

	            /* Make CSB pin HIGH */
	            coines_set_pin_config(COINES_SHUTTLE_PIN_21, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
	            coines_delay_msec(100);

	            /* SDO pin is made low */
	            coines_set_pin_config(COINES_SHUTTLE_PIN_SDO, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

	            result = coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);

	            bus_inst = COINES_I2C_BUS_0;
	        }
	        /* Bus configuration : SPI */
	        else if (intf == BMI2_SPI_INTF)
	        {
	            printf("SPI Interface \n");

	            /* To initialize the user SPI function */
	            dev_addr = COINES_MINI_SHUTTLE_PIN_2_1;
	            bmi->intf = BMI2_SPI_INTF;
	            bmi->read = bmi2_spi_read;
	            bmi->write = bmi2_spi_write;

	            result = coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE0);

	            coines_set_pin_config(COINES_SHUTTLE_PIN_21, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);

	            bus_inst = COINES_SPI_BUS_0;
	        }

	        if (COINES_SUCCESS == result)
	        {
	            /* Assign device address and bus instance to interface pointer */
	            intf_conf.bus = bus_inst;
	            intf_conf.dev_addr = dev_addr;
	            bmi->intf_ptr = ((void *)&intf_conf);

	            /* Configure delay in microseconds */
	            bmi->delay_us = bmi2_delay_us;

	            /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
	            bmi->read_write_len = READ_WRITE_LEN;

	            /* Assign to NULL to load the default config file. */
	            bmi->config_file_ptr = NULL;
	        }
	        else
	        {
	            rslt = COINES_E_COMM_INIT_FAILED;
	        }
	    }
	    else
	    {
	        rslt = BMI2_E_NULL_PTR;
	    }

	    return rslt;
	}

	static int16_t coines_board_init(enum coines_comm_intf intf_type, bool get_board_info)
	{
	    struct coines_board_info board_info;

	    int16_t result = coines_open_comm_intf(intf_type, NULL);

	    if (result < COINES_SUCCESS)
	    {
	        return result;
	    }

	    if (get_board_info)
	    {
	        result = coines_get_board_info(&board_info);
	    }

	    coines_delay_msec(100);

	    /* Power up the board */
	    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

	    coines_delay_msec(200);

	    return result;
	}

	static BMI2_INTF_RETURN_TYPE i2cRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {

	}

	static BMI2_INTF_RETURN_TYPE i2cWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {

	}


};
