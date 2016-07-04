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

int fdc1004_init(void)
{
	int ret;

	ret = fdc1004_read_id();
	if (ret < 0) {
		println("Failed to fdc1004_read_id: %d", ret);
		return ret;
	}

	return 0;
}
