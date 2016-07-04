#include <stdint.h>
#include "global_io.h"
#include "gpio.h"
#include "user_periph_setup.h"
#include "i2c.h"
#include "fdc1004.h"
#include "uart.h"

int fdc1004_read_u16(uint8_t addr, uint16_t *value)
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

int fdc1004_write_u16(uint8_t addr, uint16_t value)
{
	uint8_t buff[2] = { value >> 8, value & 0xFF };

	return fdc1004_write_data(addr, buff, sizeof(buff));
}

int fdc1004_read_capacity(uint8_t addr, uint32_t *value)
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

int fdc1004_read_id(void)
{
	int ret;
	uint16_t value;

	ret = fdc1004_read_u16(REG_DEVICE_ID, &value);
	if (ret < 0) {
		println("Failed to i2c_read_u16 REG_DEVICE_ID: %d", ret);
		return ret;
	}

	println("DEVICE_ID = 0x%04x", value);

	if (value != FDC1004_DEVICE_ID) {
		println("Invalid device id");
		return -1;
	}

	ret = fdc1004_read_u16(REG_MANUFACTURER_ID, &value);
	if (ret < 0) {
		println("Failed to i2c_read_u16 REG_MANUFACTURER_ID: %d", ret);
		return ret;
	}

	println("MANUFACTURER_ID = 0x%04x", value);

	if (value != FDC1004_MANUFACTURER_ID) {
		println("Invalid manufacturer id");
		return -1;
	}

	return 0;
}

int fdc1004_reset(void)
{
	int ret;
	uint16_t value;

	ret = fdc1004_write_u16(REG_FDC_CONF, 0x0000);
	if (ret < 0) {
		return ret;
	}

	while (fdc1004_read_u16(REG_FDC_CONF, &value) < 0 || (value & (1 << 15))) {
		println("REG_FDC_CONF = 0x%04x", value);
	}

	return 0;
}

int fdc1004_get_depth(void)
{
	int ret;
	int depth;

	while (1) {
		uint16_t status;

		ret = fdc1004_read_u16(REG_FDC_CONF, &status);
		if (ret < 0) {
			println("Failed to fdc1004_read_u16: %d", ret);
			return ret;
		}

		if ((status & 0x000F) == 0x000F) {
			break;
		}

		println("status = 0x%04x", status);
	}

	for (depth = 0; depth < 4; depth++) {
		uint16_t value;

		ret = fdc1004_read_u16(REG_MEAS1_MSB + depth * 2, &value);
		if (ret < 0) {
			println("Failed to fdc1004_read_capacity: %d", ret);
			return ret;
		}

		println("value = 0x%x", value);

		if (value < 0x1000) {
			break;
		}
	}

	return depth;
}

int fdc1004_init(void)
{
	int i;
	int ret;

	ret = fdc1004_read_id();
	if (ret < 0) {
		println("Failed to fdc1004_read_id: %d", ret);
		return ret;
	}

	ret = fdc1004_reset();
	if (ret < 0) {
		println("Failed to fdc1004_reset: %d", ret);
		return ret;
	}

	for (i = 0; i < 4; i++) {
		fdc1004_write_u16(REG_CONF_MEAS1 + i, i << 13 | 7 << 10);
	}

	fdc1004_write_u16(REG_FDC_CONF, 3 << 10 | 1 << 8 | 0x00F0);

	return 0;
}
