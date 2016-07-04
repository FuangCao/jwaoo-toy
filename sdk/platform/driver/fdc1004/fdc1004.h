#pragma once

#define FDC1004_I2C_ADDRESS			0x50
#define FDC1004_MANUFACTURER_ID		0x5449
#define FDC1004_DEVICE_ID           0x1004

enum {
	REG_MEAS1_MSB, // 0x0000 MSB portion of Measurement 1
	REG_MEAS1_LSB, // 0x0000 LSB portion of Measurement 1
	REG_MEAS2_MSB, // 0x0000 MSB portion of Measurement 2
	REG_MEAS2_LSB, // 0x0000 LSB portion of Measurement 2
	REG_MEAS3_MSB, // 0x0000 MSB portion of Measurement 3
	REG_MEAS3_LSB, // 0x0000 LSB portion of Measurement 3
	REG_MEAS4_MSB, // 0x0000 MSB portion of Measurement 4
	REG_MEAS4_LSB, // 0x0000 LSB portion of Measurement 4
	REG_CONF_MEAS1, // 0x1C00 Measurement 1 Configuration
	REG_CONF_MEAS2, // 0x1C00 Measurement 2 Configuration
	REG_CONF_MEAS3, // 0x1C00 Measurement 3 Configuration
	REG_CONF_MEAS4, // 0x1C00 Measurement 4 Configuration
	REG_FDC_CONF, // 0x0000 Capacitance to Digital Configuration
	REG_OFFSET_CAL_CIN1, // 0x0000 CIN1 Offset Calibration
	REG_OFFSET_CAL_CIN2, // 0x0000 CIN2 Offset Calibration
	REG_OFFSET_CAL_CIN3, // 0x0000 CIN3 Offset Calibration
	REG_OFFSET_CAL_CIN4, // 0x0000 CIN4 Offset Calibration
	REG_GAIN_CAL_CIN1, // 0x4000 CIN1 Gain Calibration
	REG_GAIN_CAL_CIN2, // 0x4000 CIN2 Gain Calibration
	REG_GAIN_CAL_CIN3, // 0x4000 CIN3 Gain Calibration
	REG_GAIN_CAL_CIN4, // 0x4000 CIN4 Gain Calibration
	REG_MANUFACTURER_ID = 0xFE, // 0x5449 ID of Texas Instruments
	REG_DEVICE_ID, // 0x1004 ID of FDC1004 device
};

int fdc1004_read_u16(uint8_t addr, uint16_t *value);
int fdc1004_write_u16(uint8_t addr, uint16_t value);
int fdc1004_read_id(void);
int fdc1004_init(void);

static inline int fdc1004_read_data(uint8_t addr, uint8_t *data, int size)
{
	return i2c_read_data(FDC1004_I2C_ADDRESS, addr, data, size);
}

static inline int fdc1004_write_data(uint8_t addr, const uint8_t *data, int size)
{
	return i2c_write_data(FDC1004_I2C_ADDRESS, addr, data, size);
}
