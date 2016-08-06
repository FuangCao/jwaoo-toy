#include <stdint.h>
#include "global_io.h"
#include "gpio.h"
#include "user_periph_setup.h"
#include "i2c.h"
#include "fdc1004.h"
#include "uart.h"

#define FDC1004_GAIN		0xFFFF
#define FDC1004_CAPDAC		7

static int fdc1004_read_u16(uint8_t addr, uint16_t *value)
{
	int ret;
	uint8_t buff[2];

	ret = fdc1004_read_data(addr, buff, sizeof(buff));
	if (ret < 0) {
		return ret;
	}

	*value = ((uint16_t) buff[0]) << 8 | buff[1];

	return 0;
}

static int fdc1004_write_u16(uint8_t addr, uint16_t value)
{
	uint8_t buff[2] = { value >> 8, value & 0xFF };

	return fdc1004_write_data(addr, buff, sizeof(buff));
}

#if 0
static int fdc1004_read_capacity(uint8_t addr, uint32_t *value)
{
	int ret;
	uint8_t buff[4];

	ret = fdc1004_read_data(addr, buff, 2);
	if (ret < 0) {
		return ret;
	}

	ret = fdc1004_read_data(addr + 1, buff + 2, 2);
	if (ret < 0) {
		return ret;
	}

	*value = ((uint32) buff[0]) << 16 | ((uint32_t) buff[1]) << 8 | buff[2];

	return 0;
}
#endif

static int fdc1004_read_id(void)
{
	int ret;
	uint16_t value;

	ret = fdc1004_read_u16(FDC1004_REG_DEVICE_ID, &value);
	if (ret < 0) {
		println("Failed to read FDC1004_REG_DEVICE_ID: %d", ret);
		return ret;
	}

	println("DEVICE_ID = 0x%04x", value);

	if (value != FDC1004_DEVICE_ID) {
		println("Invalid device id");
		return -1;
	}

	ret = fdc1004_read_u16(FDC1004_REG_MANUFACTURER_ID, &value);
	if (ret < 0) {
		println("Failed to read FDC1004_REG_MANUFACTURER_ID: %d", ret);
		return ret;
	}

	println("MANUFACTURER_ID = 0x%04x", value);

	if (value != FDC1004_MANUFACTURER_ID) {
		println("Invalid manufacturer id");
		return -1;
	}

	return 0;
}

static int fdc1004_reset(void)
{
	int i;
	int ret;

	ret = fdc1004_write_u16(FDC1004_REG_FDC_CONF, 1 << 15);
	if (ret < 0) {
		return ret;
	}

	for (i = 0; i < 100; i++) {
		uint16_t value;

		ret = fdc1004_read_u16(FDC1004_REG_FDC_CONF, &value);
		if (ret < 0) {
			println("Failed to read FDC1004_REG_FDC_CONF: %d", ret);
		} else if (value & (1 << 15)) {
			println("FDC1004_REG_FDC_CONF = 0x%04x", value);
		} else {
			return 0;
		}
	}

	return -1;
}

bool fdc1004_set_enable(bool enable)
{
	int ret;

	if (enable) {
		int i;

		ret = fdc1004_read_id();
		if (ret < 0) {
			println("Failed to fdc1004_read_id: %d", ret);
			return false;
		}

		ret = fdc1004_reset();
		if (ret < 0) {
			println("Failed to fdc1004_reset: %d", ret);
			return false;
		}

		for (i = 0; i < 4; i++) {
			ret = fdc1004_write_u16(FDC1004_REG_CONF_MEAS1 + i, i << 13 | 4 << 10 | FDC1004_CAPDAC << 5);
			if (ret < 0) {
				println("Failed to fdc1004_write_u16: %d", ret);
				return false;
			}

			ret = fdc1004_write_u16(FDC1004_REG_GAIN_CAL_CIN1 + i, FDC1004_GAIN);
			if (ret < 0) {
				println("Failed to fdc1004_write_u16: %d", ret);
				return false;
			}
		}

		ret = fdc1004_write_u16(FDC1004_REG_FDC_CONF, 3 << 10 | 1 << 8 | 0x00F0);
		if (ret < 0) {
			println("Failed to fdc1004_write_u16: %d", ret);
			return false;
		}
	} else {
		ret = fdc1004_write_u16(FDC1004_REG_FDC_CONF, 0x0000);
		if (ret < 0) {
			println("Failed to fdc1004_write_u16: %d", ret);
			return false;
		}
	}

	return true;
}

bool fdc1004_read_sensor_values(uint8_t *values)
{
	int ret;
	uint8_t addr;

	for (addr = FDC1004_REG_MEAS1_MSB; addr < FDC1004_REG_MEAS4_LSB; addr += 2) {
		ret = fdc1004_read_data(addr, values, 2);
		if (ret < 0) {
			println("Failed to fdc1004_read_data: %d", ret);
			return false;
		}

		values += FDC1004_VALUE_BYTES;
	}

	return true;
}
