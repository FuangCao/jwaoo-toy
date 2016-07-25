#pragma once

#include <i2c.h>

#define FDC1004_I2C_ADDRESS			0x50
#define FDC1004_MANUFACTURER_ID		0x5449
#define FDC1004_DEVICE_ID           0x1004

enum {
	FDC1004_REG_MEAS1_MSB, // 0x0000 MSB portion of Measurement 1
	FDC1004_REG_MEAS1_LSB, // 0x0000 LSB portion of Measurement 1
	FDC1004_REG_MEAS2_MSB, // 0x0000 MSB portion of Measurement 2
	FDC1004_REG_MEAS2_LSB, // 0x0000 LSB portion of Measurement 2
	FDC1004_REG_MEAS3_MSB, // 0x0000 MSB portion of Measurement 3
	FDC1004_REG_MEAS3_LSB, // 0x0000 LSB portion of Measurement 3
	FDC1004_REG_MEAS4_MSB, // 0x0000 MSB portion of Measurement 4
	FDC1004_REG_MEAS4_LSB, // 0x0000 LSB portion of Measurement 4
	FDC1004_REG_CONF_MEAS1, // 0x1C00 Measurement 1 Configuration
	FDC1004_REG_CONF_MEAS2, // 0x1C00 Measurement 2 Configuration
	FDC1004_REG_CONF_MEAS3, // 0x1C00 Measurement 3 Configuration
	FDC1004_REG_CONF_MEAS4, // 0x1C00 Measurement 4 Configuration
	FDC1004_REG_FDC_CONF, // 0x0000 Capacitance to Digital Configuration
	FDC1004_REG_OFFSET_CAL_CIN1, // 0x0000 CIN1 Offset Calibration
	FDC1004_REG_OFFSET_CAL_CIN2, // 0x0000 CIN2 Offset Calibration
	FDC1004_REG_OFFSET_CAL_CIN3, // 0x0000 CIN3 Offset Calibration
	FDC1004_REG_OFFSET_CAL_CIN4, // 0x0000 CIN4 Offset Calibration
	FDC1004_REG_GAIN_CAL_CIN1, // 0x4000 CIN1 Gain Calibration
	FDC1004_REG_GAIN_CAL_CIN2, // 0x4000 CIN2 Gain Calibration
	FDC1004_REG_GAIN_CAL_CIN3, // 0x4000 CIN3 Gain Calibration
	FDC1004_REG_GAIN_CAL_CIN4, // 0x4000 CIN4 Gain Calibration
	FDC1004_REG_MANUFACTURER_ID = 0xFE, // 0x5449 ID of Texas Instruments
	FDC1004_REG_DEVICE_ID, // 0x1004 ID of FDC1004 device
};

bool fdc1004_set_enable(bool enable);
bool fdc1004_read_sensor_values(uint8_t values[4]);

static inline int fdc1004_read_data(uint8_t addr, uint8_t *data, int size)
{
	return i2c_read_data(FDC1004_I2C_ADDRESS, addr, data, size);
}

static inline int fdc1004_write_data(uint8_t addr, const uint8_t *data, int size)
{
	return i2c_write_data(FDC1004_I2C_ADDRESS, addr, data, size);
}
