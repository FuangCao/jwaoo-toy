#include <stdint.h>
#include "global_io.h"
#include "gpio.h"
#include "user_periph_setup.h"
#include "i2c.h"
#include "mpu6050.h"
#include "uart.h"

static int mpu6050_reset(void)
{
	int i;
	int ret;

	ret = mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 1 << 7);
	if (ret < 0) {
		println("Failed to mpu6050_write_register: %d", ret);
		return ret;
	}

	for (i = 0; i < 100; i++) {
		uint8_t value;

		ret = mpu6050_read_register(MPU6050_REG_PWR_MGMT_1, &value);
		if (ret < 0) {
			println("Failed to mpu6050_read_register: %d", ret);
		} else if ((value & (1 << 7)) == 0) {
			return 0;
		} else {
			println("MPU6050_REG_PWR_MGMT_1 = 0x%02x", value);
		}
	}

	return -1;
}

bool mpu6050_set_enable(bool enable)
{
	int ret;
	uint8_t value;

	if (enable) {
		uint8_t id;

		ret = mpu6050_read_register(MPU6050_REG_CHIP_ID, &id);
		if (ret < 0) {
			println("Failed to mpu6050_read_register: %d", ret);
			return false;
		}

		println("mpu6050: chip id = 0x%02x", id);

		if (id != MPU6050_CHIP_ID) {
			println("Invalid chip id");
			return false;
		}

		ret = mpu6050_reset();
		if (ret < 0) {
			println("Failed to mpu6050_reset: %d", ret);
			return false;
		}

		value = (1 << 2);
	} else {
		value = (1 << 6 | 1 << 2);
	}

	ret = mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, value);
	if (ret < 0) {
		println("Failed to mpu6050_write_register: %d", ret);
		return false;
	}

	mpu6050_read_register(MPU6050_REG_PWR_MGMT_1, &value);
	println("MPU6050_REG_PWR_MGMT_1 = 0x%02x", value);
	mpu6050_read_register(MPU6050_REG_ACCEL_CONFIG, &value);
	println("MPU6050_REG_ACCEL_CONFIG = 0x%02x", value);

	return true;
}

bool mpu6050_read_sensor_values(uint8_t values[3])
{
	int ret;
	uint8_t buff[6];

	ret = mpu6050_read_data(MPU6050_REG_ACCEL_XOUT_H, buff, sizeof(buff));
	if (ret < 0) {
		println("Failed to mpu6050_read_data: %d", ret);
		return false;
	}

	values[0] = buff[0];
	values[1] = buff[2];
	values[2] = buff[4];

	return true;
}
