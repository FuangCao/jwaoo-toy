#include <stdint.h>
#include "global_io.h"
#include "gpio.h"
#include "user_periph_setup.h"
#include "i2c.h"
#include "bmi160.h"
#include "uart.h"

bool bmi160_set_enable(bool enable)
{
	int ret;
	uint8_t command;

	if (enable) {
		uint8_t id;

		ret = bmi160_read_register(BMI160_REG_CHIP_ID, &id);
		if (ret < 0) {
			println("Failed to bmi160_read_register: %d", ret);
			return false;
		}

		println("bmi160: chip id = 0x%02x", id);

		switch (id) {
		case BMI160_CHIP_ID:
			println("BMI160 found");
			break;

		case BMI120_CHIP_ID:
			println("BMI120 found");
			break;

		default:
			println("Invalid chip id");
			return false;
		}

		command = 0x11;
	} else {
		command = 0x10;
	}

	ret = bmi160_write_register(BMI160_REG_CMD, command);
	if (ret < 0) {
		println("Failed to bmi160_write_register: %d", ret);
		return false;
	}

	return true;
}

bool bmi160_read_sensor_values(uint8_t values[3])
{
	int ret;
	uint8_t buff[6];

	ret = bmi160_read_data(BMI160_REG_DATA_ACC, buff, sizeof(buff));
	if (ret < 0) {
		println("Failed to bmi160_read_data: %d", ret);
		return false;
	}

	values[0] = buff[1];
	values[1] = buff[3];
	values[2] = buff[5];

	return true;
}
