/**
 ****************************************************************************************
 *
 * @file i2c.c
 *
 * @brief eeprom driver over i2c interface.
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

#include <stdint.h>
#include "global_io.h"
#include "gpio.h"
#include "user_periph_setup.h"
#include "i2c.h"
#include "uart.h"

#define I2C_AUTO_POWER_DOWN		1

static int i2c_wait_while_tx_fifo_full()
{
	int i;

	for (i = 0; i < 50000; i++) {
		if (GetWord16(I2C_STATUS_REG) & TFNF) {
			return 0;
		}
	}

	return -1;
}

static int i2c_wait_until_tx_fifo_empty()
{
	int i;

	for (i = 0; i < 50000; i++) {
		if (GetWord16(I2C_STATUS_REG) & TFE) {
			return 0;
		}
	}

	return -1;
}

static int i2c_send_command(uint16_t command)
{
	int ret;

	ret = i2c_wait_while_tx_fifo_full();
	if (ret < 0) {
		return ret;
	}

	SetWord16(I2C_DATA_CMD_REG, command);

	return 0;
}

static int i2c_get_rx_fifo_size()
{
	int i;

	for (i = 0; i < 50000; i++) {
		uint16_t size = GetWord16(I2C_RXFLR_REG);
		if (size > 0) {
			return size;
		}
	}

	return -1;
}

static int i2c_wait_until_idle()
{
	int i;

	for (i = 0; i < 500000; i++) {
		if ((GetWord16(I2C_STATUS_REG) & MST_ACTIVITY) == 0) {
			return 0;
		}
	}

	return -1;
}

void i2c_init(uint8_t speed, uint8_t address_mode)
{
	SetBits16(CLK_PER_REG, I2C_ENABLE, 1);                                        // enable  clock for I2C
	SetWord16(I2C_ENABLE_REG, 0x0);                                               // Disable the I2C controller
	SetWord16(I2C_CON_REG, I2C_MASTER_MODE | I2C_SLAVE_DISABLE | I2C_RESTART_EN); // Slave is disabled
	SetBits16(I2C_CON_REG, I2C_SPEED, speed);                                     // Set speed
	SetBits16(I2C_CON_REG, I2C_10BITADDR_MASTER, address_mode);                   // Set addressing mode

#if I2C_AUTO_POWER_DOWN == 0
	SetWord16(I2C_ENABLE_REG, 1);
	i2c_wait_until_idle();
#endif
}

void i2c_release(void)
{	
    SetWord16(I2C_ENABLE_REG, 0x0);                             // Disable the I2C controller	
    SetBits16(CLK_PER_REG, I2C_ENABLE, 0);                      // Disable clock for I2C
}

int i2c_transfer(uint8_t slave, struct i2c_message *msgs, int count)
{
	int ret;
	struct i2c_message *msg = msgs;
	struct i2c_message *msg_end = msg + count;

#if I2C_AUTO_POWER_DOWN
	SetWord16(I2C_TAR_REG, slave);
	SetWord16(I2C_ENABLE_REG, 1);
#else
	static uint8_t last_slave;

	if (slave != last_slave) {
		last_slave = slave;
		SetWord16(I2C_ENABLE_REG, 0);
		SetWord16(I2C_TAR_REG, slave);
		SetWord16(I2C_ENABLE_REG, 1);
	}
#endif

	ret = i2c_wait_until_idle();
	if (ret < 0) {
		goto out_i2c_disable;
	}

	GLOBAL_INT_DISABLE();

	GetWord16(I2C_CLR_TX_ABRT_REG);

	while (msg < msg_end) {
		uint8_t *data = msg->data;
		uint8_t *data_end = data + msg->count;

		if (msg->read) {
			int i;

			for (i = msg->count; i > 0; i--) {
				ret = i2c_send_command(0x0100);
				if (ret < 0) {
					goto out_enable_irq;
				}
			}

			while (data < data_end) {
				int fifo_size = i2c_get_rx_fifo_size();
				if (fifo_size < 0) {
					ret = fifo_size;
					println("Failed to i2c_read_byte");
					goto out_enable_irq;
				}

				while (fifo_size > 0 && data < data_end) {
					*data++ = GetWord16(I2C_DATA_CMD_REG) & 0xFF;
					fifo_size--;
				}
			}

			msg++;
		} else {
			while (data < data_end) {
				ret = i2c_send_command(*data++);
				if (ret < 0) {
					goto out_enable_irq;
				}
			}

			msg++;

			if (msg >= msg_end || msg->read) {
				uint16_t status;

				ret = i2c_wait_until_tx_fifo_empty();
				if (ret < 0) {
					goto out_enable_irq;
				}

				status = GetWord16(I2C_TX_ABRT_SOURCE_REG);
				if (status) {
					ret = -1;
					println("I2C_TX_ABRT_SOURCE_REG = 0x%04x", status);
					goto out_enable_irq;
				}
			}
		}
	}

	ret = count;

out_enable_irq:
	GLOBAL_INT_RESTORE();
out_i2c_disable:
#if I2C_AUTO_POWER_DOWN
	SetWord16(I2C_ENABLE_REG, 0);
#endif

	return ret;
}

int i2c_read_data(uint8_t slave, uint8_t addr, uint8_t *data, int size)
{
	struct i2c_message msgs[] = {
		{
			.data = &addr,
			.count = 1,
			.read = 0,
		}, {
			.data = data,
			.count = size,
			.read = 1,
		}
	};

	if (i2c_transfer(slave, msgs, NELEM(msgs)) == NELEM(msgs)) {
		return size;
	}

	return -1;
}

int i2c_write_data(uint8_t slave, uint8_t addr, const uint8_t *data, int size)
{
	struct i2c_message msgs[] = {
		{
			.data = &addr,
			.count = 1,
			.read = 0,
		}, {
			.data = (uint8_t *) data,
			.count = size,
			.read = 0,
		}
	};

	if (i2c_transfer(slave, msgs, NELEM(msgs)) == NELEM(msgs)) {
		return size;
	}

	return -1;
}

int i2c_update_u8(uint8_t slave, uint8_t addr, uint8_t value, uint8_t mask)
{
	int ret;
	uint8_t value_old;

	ret = i2c_read_u8(slave, addr, &value_old);
	if (ret < 0) {
		return ret;
	}

	if ((value_old & mask) == value) {
		return 0;
	}

	return i2c_write_u8(slave, addr, value | (value_old & (~mask)));
}

int i2c_update_u16(uint8_t slave, uint8_t addr, uint16_t value, uint16_t mask)
{
	int ret;
	uint16_t value_old;

	ret = i2c_read_u16(slave, addr, &value_old);
	if (ret < 0) {
		return ret;
	}

	if ((value_old & mask) == value) {
		return 0;
	}

	return i2c_write_u16(slave, addr, value | (value_old & (~mask)));
}

int i2c_update_u32(uint8_t slave, uint8_t addr, uint32_t value, uint32_t mask)
{
	int ret;
	uint32_t value_old;

	ret = i2c_read_u32(slave, addr, &value_old);
	if (ret < 0) {
		return ret;
	}

	if ((value_old & mask) == value) {
		return 0;
	}

	return i2c_write_u32(slave, addr, value | (value_old & (~mask)));
}
