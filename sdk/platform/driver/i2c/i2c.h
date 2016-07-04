/**
 ****************************************************************************************
 *
 * @file i2c.h
 *
 * @brief eeprom driver over i2c interface header file.
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#ifndef _I2C_H_
#define _I2C_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>

/*
 * DEFINES
 ****************************************************************************************
 */

#define NELEM(a) \
	(sizeof(a) / sizeof((a)[0]))

#define I2C_SEND_COMMAND(value) \
	SetWord16(I2C_DATA_CMD_REG, (value))

#define I2C_WAIT_WHILE_TX_FIFO_FULL() \
	while((GetWord16(I2C_STATUS_REG) & TFNF) == 0)

#define I2C_WAIT_UNTIL_TX_FIFO_EMPTY() \
	while((GetWord16(I2C_STATUS_REG) & TFE) == 0)

#define I2C_WAIT_UNTIL_NO_MASTER_ACTIVITY() \
	while((GetWord16(I2C_STATUS_REG) & MST_ACTIVITY) !=0)

#define I2C_WAIT_FOR_RECEIVED_BYTE() \
	while(GetWord16(I2C_RXFLR_REG) == 0)

enum
{
	I2C_SPEED_100K = 1,
	I2C_SPEED_400K,
};

enum
{
	I2C_ADDRESS_MODE_7BIT,
	I2C_ADDRESS_MODE_10BIT,
};

struct i2c_message
{
	uint8_t *data;
	uint8_t count;
	uint8_t read;
};

void i2c_init(uint8_t speed, uint8_t address_mode);
void i2c_release(void);

int i2c_transfer(uint8_t client, struct i2c_message *msgs, int count);
int i2c_read_data(uint8_t client, uint8_t addr, uint8_t *data, int size);
int i2c_write_data(uint8_t client, uint8_t addr, const uint8_t *data, int size);
int i2c_update_u8(uint8_t slave, uint8_t addr, uint8_t value, uint8_t mask);
int i2c_update_u16(uint8_t slave, uint8_t addr, uint16_t value, uint16_t mask);
int i2c_update_u32(uint8_t slave, uint8_t addr, uint32_t value, uint32_t mask);

static inline int i2c_read_u8(uint8_t slave, uint8_t addr, uint8_t *value)
{
	return i2c_read_data(slave, addr, value, sizeof(*value));
}

static inline int i2c_read_u16(uint8_t slave, uint8_t addr, uint16_t *value)
{
	return i2c_read_data(slave, addr, (uint8_t *) value, sizeof(*value));
}

static inline int i2c_read_u32(uint8_t slave, uint8_t addr, uint32_t *value)
{
	return i2c_read_data(slave, addr, (uint8_t *) value, sizeof(*value));
}

static inline int i2c_write_u8(uint8_t slave, uint8_t addr, uint8_t value)
{
	return i2c_write_data(slave, addr, &value, sizeof(value));
}

static inline int i2c_write_u16(uint8_t slave, uint8_t addr, uint16_t value)
{
	return i2c_write_data(slave, addr, (uint8_t *) &value, sizeof(value));
}

static inline int i2c_write_u32(uint8_t slave, uint8_t addr, uint32_t value)
{
	return i2c_write_data(slave, addr, (uint8_t *) &value, sizeof(value));
}

#endif // _I2C_H_
