/**
 ****************************************************************************************
 *
 * @file jwaoo_toy.c
 *
 * @brief Device Information Service Server Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup JWAOO_TOY
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_JWAOO_TOY_SERVER)
#include "app.h"
#include "pwm.h"
#include "uart.h"
#include "stdarg.h"
#include "spi_flash.h"
#include "attm_util.h"
#include "atts_util.h"
#include "jwaoo_toy.h"
#include "jwaoo_toy_task.h"
#include "prf_utils.h"
#include "app_easy_timer.h"
#include "user_periph_setup.h"
#include "bmi160.h"
#include "mpu6050.h"
#include "fdc1004.h"
#include "user_barebone.h"

/*
 * MACROS
 ****************************************************************************************
 */

#if SPI_CODE_SIZE > KB(32)
#define SPI_ERASE_MODE		BLOCK_ERASE_64
#else
#define SPI_ERASE_MODE		BLOCK_ERASE_32
#endif

/*
 * JWAOO_TOY ATTRIBUTES
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct jwaoo_toy_env_tag jwaoo_toy_env __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY
struct jwaoo_toy_device_data_tag jwaoo_toy_device_data __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY

struct jwaoo_toy_key jwaoo_toy_keys[JWAOO_TOY_KEY_COUNT] = {
	{
		.code = 0,
		.repeat_timer = JWAOO_TOY_KEY1_REPEAT_TIMER,
		.long_click_timer = JWAOO_TOY_KEY1_LONG_CLICK_TIMER,
		.multi_click_timer = JWAOO_TOY_KEY1_MULTI_CLICK_TIMER,
	}, {
		.code = 1,
		.repeat_timer = JWAOO_TOY_KEY2_REPEAT_TIMER,
		.long_click_timer = JWAOO_TOY_KEY2_LONG_CLICK_TIMER,
		.multi_click_timer = JWAOO_TOY_KEY2_MULTI_CLICK_TIMER,
	}, {
		.code = 2,
		.repeat_timer = JWAOO_TOY_KEY3_REPEAT_TIMER,
		.long_click_timer = JWAOO_TOY_KEY3_LONG_CLICK_TIMER,
		.multi_click_timer = JWAOO_TOY_KEY3_MULTI_CLICK_TIMER,
	}, {
		.code = 3,
		.repeat_timer = JWAOO_TOY_KEY4_REPEAT_TIMER,
		.long_click_timer = JWAOO_TOY_KEY4_LONG_CLICK_TIMER,
		.multi_click_timer = JWAOO_TOY_KEY4_MULTI_CLICK_TIMER,
	}
};

static const struct ke_task_desc TASK_DESC_JWAOO_TOY = {
	.state_handler = jwaoo_toy_state_handler,
	.default_handler = &jwaoo_toy_default_handler,
	.state = jwaoo_toy_state,
	.state_max = JWAOO_TOY_STATE_COUNT,
	.idx_max = JWAOO_TOY_TASK_COUNT
};

extern uint32_t spi_flash_jedec_id;
extern uint32_t spi_flash_size;
extern uint32_t spi_flash_page_size;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static bool jwaoo_toy_sensor_read_values_dummy(uint8_t values[3])
{
	return false;
}

static bool (*jwaoo_toy_accel_sensor_set_enable)(bool enable);
static bool (*jwaoo_toy_capacity_sensor_set_enable)(bool enable);
static bool (*jwaoo_toy_accel_sensor_read_values)(uint8_t values[3]) = jwaoo_toy_sensor_read_values_dummy;
static bool (*jwaoo_toy_capacity_sensor_read_values)(uint8_t values[4]) = jwaoo_toy_sensor_read_values_dummy;

uint8_t jwaoo_toy_sensor_poll(void)
{
	uint8_t buff[3 + FDC1004_DATA_BYTES + 1];

	if (jwaoo_toy_env.sensor_accel_dead < 10) {
		if (jwaoo_toy_accel_sensor_read_values(buff)) {
			jwaoo_toy_env.sensor_accel_dead = 0;
		} else {
			jwaoo_toy_env.sensor_accel_dead++;
			memset(buff, 0x00, 3);
		}
	} else {
		memset(buff, 0x00, 3);
	}

	if (jwaoo_toy_env.sensor_capacity_dead < 10) {
		if (jwaoo_toy_capacity_sensor_read_values(buff + 3)) {
			jwaoo_toy_env.sensor_capacity_dead = 0;
		} else {
			jwaoo_toy_env.sensor_capacity_dead++;
			memset(buff + 3, 0x00, FDC1004_DATA_BYTES);
		}
	} else {
		memset(buff + 3, 0x00, FDC1004_DATA_BYTES);
	}

	return jwaoo_toy_send_notify(JWAOO_TOY_ATTR_SENSOR_DATA, buff, 3 + FDC1004_DATA_BYTES);
}

bool jwaoo_toy_sensor_set_enable(bool enable)
{
	println("sensor_enable = %d, sensor_poll_delay = %d", enable, jwaoo_toy_env.sensor_poll_delay);

	if (enable) {
		if (jwaoo_toy_accel_sensor_set_enable) {
			jwaoo_toy_accel_sensor_set_enable(true);
		} else if (bmi160_set_enable(true)) {
			jwaoo_toy_accel_sensor_set_enable = bmi160_set_enable;
			jwaoo_toy_accel_sensor_read_values = bmi160_read_sensor_values;
		} else if (mpu6050_set_enable(true)) {
			jwaoo_toy_accel_sensor_set_enable = mpu6050_set_enable;
			jwaoo_toy_accel_sensor_read_values = mpu6050_read_sensor_values;
		} else {
			jwaoo_toy_accel_sensor_read_values = jwaoo_toy_sensor_read_values_dummy;
		}

		if (jwaoo_toy_capacity_sensor_set_enable) {
			jwaoo_toy_capacity_sensor_set_enable(true);
		} else if (fdc1004_set_enable(true)) {
			jwaoo_toy_capacity_sensor_set_enable = bmi160_set_enable;
			jwaoo_toy_capacity_sensor_read_values = fdc1004_read_sensor_values;
		} else {
			jwaoo_toy_capacity_sensor_read_values = jwaoo_toy_sensor_read_values_dummy;
		}

		jwaoo_toy_env.sensor_poll_enable = true;

		if (!ke_timer_active(JWAOO_TOY_SENSOR_POLL, TASK_JWAOO_TOY)) {
			ke_timer_set(JWAOO_TOY_SENSOR_POLL, TASK_JWAOO_TOY, 1);
		}
	} else {
		jwaoo_toy_env.sensor_poll_enable = false;
		ke_timer_clear(JWAOO_TOY_SENSOR_POLL, TASK_JWAOO_TOY);

		if (jwaoo_toy_capacity_sensor_set_enable) {
			jwaoo_toy_capacity_sensor_set_enable(false);
		}

		if (jwaoo_toy_accel_sensor_set_enable) {
			jwaoo_toy_accel_sensor_set_enable(false);
		}
	}

	return true;
}

bool jwaoo_toy_read_device_data(void)
{
	int32_t rdlen;
	const uint8_t *bd_addr = jwaoo_toy_device_data.bd_addr;

	rdlen = spi_flash_read_data((uint8_t *) &jwaoo_toy_device_data, SPI_PART_DEVICE_DATA, sizeof(jwaoo_toy_device_data));
	if (rdlen != sizeof(struct jwaoo_toy_device_data_tag)) {
		println("Failed to spi_flash_read_data: %d", rdlen);
		return false;
	}

	println("bd_addr: %02x:%02x:%02x:%02x:%02x:%02x",
		bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);

	return true;
}

bool jwaoo_toy_write_device_data(void)
{
	int8_t err;
	int32_t wrlen;

	err = spi_flash_block_erase(SPI_PART_DEVICE_DATA, SECTOR_ERASE);
	if (err != ERR_OK) {
		println("Failed to spi_flash_block_erase: %d", err);
		return false;
	}

	wrlen = spi_flash_write_data((uint8_t *) &jwaoo_toy_device_data, SPI_PART_DEVICE_DATA, sizeof(jwaoo_toy_device_data));
	if (wrlen != sizeof(jwaoo_toy_device_data)) {
		println("Failed to spi_flash_write_data: %d", err);
		return false;
	}

	return true;
}

bool jwaoo_toy_write_bd_addr(const uint8_t bd_addr[6])
{
	memcpy(jwaoo_toy_device_data.bd_addr, bd_addr, 6);

	return jwaoo_toy_write_device_data();
}

uint8_t jwaoo_toy_calculate_crc(const uint8_t *mem, uint32_t size, uint8_t crc)
{
	const uint8_t *mem_end;

	for (mem_end = mem + size; mem < mem_end; mem++) {
		crc ^= *mem;
	}

	return crc;
}

bool jwaoo_toy_flash_check_crc(uint32_t addr, uint32_t size, uint8_t crc_raw)
{
	uint8_t crc = 0xFF;

	while (size > 0) {
		int ret;
		int length;
		uint8_t buff[128];

		length = (size > sizeof(buff) ? sizeof(buff) : size);

		ret = spi_flash_read_data(buff, addr, length);
		if (ret != length) {
			println("Failed to spi_flash_read_data: %d", ret);
			return false;
		}

		crc = jwaoo_toy_calculate_crc(buff, length, crc);
		addr += length;
		size -= length;
	}

	if (crc != crc_raw) {
		println("crc not match: 0x%02x != 0x%02x", crc, crc_raw);
		return false;
	}

	return true;
}

bool jwaoo_toy_flash_copy(uint32_t rdaddr, uint32_t wraddr, uint32_t size, uint8_t crc_raw)
{
	uint8_t crc_read = 0xFF;
	uint8_t crc_write = 0xFF;

	println("spi_flash_block_erase: 0x%04x", wraddr);

	if (spi_flash_block_erase(wraddr, SPI_ERASE_MODE) != ERR_OK) {
		println("Failed to spi_flash_block_erase");
		return false;
	}

	println("%s: 0x%04x [%d]=> 0x%04x", __FUNCTION__, rdaddr, size, wraddr);

	while (size > 0) {
		int ret;
		int length;
		static uint8_t buff[128];

		length = (size > sizeof(buff) ? sizeof(buff) : size);

		ret = spi_flash_read_data(buff, rdaddr, length);
		if (ret != length) {
			println("Failed to spi_flash_read_data: %d", ret);
			return false;
		}

		crc_read = jwaoo_toy_calculate_crc(buff, length, crc_read);

		ret = spi_flash_write_data(buff, wraddr, length);
		if (ret != length) {
			println("Failed to spi_flash_write_data: %d", ret);
			return false;
		}

		ret = spi_flash_read_data(buff, wraddr, length);
		if (ret != length) {
			println("Failed to spi_flash_read_data: %d", ret);
			return false;
		}

		crc_write = jwaoo_toy_calculate_crc(buff, length, crc_write);

		rdaddr += length;
		wraddr += length;
		size -= length;
	}

	if (crc_read != crc_raw) {
		println("read crc not match: 0x%02x != 0x%02x", crc_read, crc_raw);
		return false;
	}

	if (crc_write != crc_raw) {
		println("write crc not match: 0x%02x != 0x%02x", crc_write, crc_raw);
		return false;
	}

	println("%s successfull", __FUNCTION__);

	return true;
}

bool jwaoo_toy_moto_set_mode(uint8_t mode, uint8_t speed)
{
	// println("Moto: mode = %d, level = %d", mode, level);

	switch (mode) {
	case 0:
		jwaoo_moto_set_speed(speed);
		break;

	case 1:
		jwaoo_moto_set_speed(10);
		break;

	case 2:
		jwaoo_moto_blink_sawtooth(4000);
		break;

	case 3:
		jwaoo_moto_blink_sawtooth(2000);
		break;

	case 4:
		jwaoo_moto_blink_square(4000);
		break;

	case 5:
		jwaoo_moto_blink_square(2000);
		break;

	case 6:
		ke_timer_clear(JWAOO_TOY_MOTO_BLINK, TASK_APP);
		ke_timer_set(JWAOO_TOY_MOTO_BLINK, TASK_APP, 1);
		break;

	default:
		return false;
	}

	jwaoo_toy_env.moto_mode = mode;

	return true;
}

void jwaoo_toy_set_factory_enable(bool enable)
{
	if (jwaoo_toy_env.factory_enable == enable) {
		return;
	}

	if (enable) {
		jwaoo_toy_env.factory_enable = true;

		jwaoo_toy_env.battery_led_locked = 3;
		jwaoo_led_close(&jwaoo_pwm_led1);
	} else {
		jwaoo_toy_env.factory_enable = false;

		jwaoo_toy_env.battery_led_locked = 0;
		jwaoo_toy_battery_led_update_state();
	}
}

// ===============================================================================

void jwaoo_toy_init(void)
{
    // Reset environment
    memset(&jwaoo_toy_env, 0, sizeof(jwaoo_toy_env));

	jwaoo_toy_env.sensor_poll_delay = 20;
	jwaoo_toy_env.key_long_click_delay = 200;
	jwaoo_toy_env.key_multi_click_delay = 30;

	jwaoo_toy_keys[JWAOO_TOY_KEYCODE_UP].lock_enable = true;
	jwaoo_toy_keys[JWAOO_TOY_KEYCODE_UP].repeat_enable= true;

	jwaoo_toy_keys[JWAOO_TOY_KEYCODE_DOWN].lock_enable = true;
	jwaoo_toy_keys[JWAOO_TOY_KEYCODE_DOWN].repeat_enable= true;

    // Create JWAOO_TOY task
    ke_task_create(TASK_JWAOO_TOY, &TASK_DESC_JWAOO_TOY);

    // Set task in disabled state
    ke_state_set(TASK_JWAOO_TOY, JWAOO_TOY_DISABLED);
}

void jwaoo_toy_enable(uint16_t conhdl) 
{
	jwaoo_toy_env.sensor_capacity_dead = 0;
	jwaoo_toy_env.sensor_accel_dead = 0;

#if 0
	if (jwaoo_toy_env.sensor_enable) {
		jwaoo_toy_sensor_set_enable(true);
	}
#endif
}

void jwaoo_toy_disable(uint16_t conhdl) 
{
    // Inform the application about the disconnection
    struct jwaoo_toy_disable_ind *ind = KE_MSG_ALLOC(JWAOO_TOY_DISABLE_IND,
		jwaoo_toy_env.con_info.appid, TASK_JWAOO_TOY, jwaoo_toy_disable_ind);

    ind->conhdl = conhdl;

    ke_msg_send(ind);

	jwaoo_toy_env.notify_busy_mask = 0;
	jwaoo_toy_env.notify_enable_mask = 0;
	jwaoo_toy_env.flash_upgrade = false;
	jwaoo_toy_env.flash_write_enable = false;
	jwaoo_toy_env.flash_write_success = false;
	jwaoo_toy_sensor_set_enable(false);
	jwaoo_toy_set_factory_enable(false);

    //Disable JWAOO_TOY in database
    attmdb_svc_set_permission(jwaoo_toy_env.handle, PERM(SVC, DISABLE));

    //Go to idle state
    ke_state_set(TASK_JWAOO_TOY, JWAOO_TOY_IDLE);
}

// ===============================================================================

uint8_t jwaoo_toy_write_data(uint16_t attr, const void *data, int size)
{
	return attmdb_att_set_value(jwaoo_toy_env.handle + attr, size, (void *) data);
}

uint8_t jwaoo_toy_send_notify(uint16_t attr, const void *data, int size)
{
	uint8_t ret;
	uint16_t handle;
	uint16_t mask = 1 << attr;

	if ((jwaoo_toy_env.notify_enable_mask & mask) == 0) {
		return ATT_ERR_WRITE_NOT_PERMITTED;
	}

	if (jwaoo_toy_env.notify_busy_mask & mask) {
		return ATT_ERR_PREPARE_QUEUE_FULL;
	}

	jwaoo_toy_env.notify_busy_mask |= mask;

	handle = jwaoo_toy_env.handle + attr;

	ret = attmdb_att_set_value(handle, size, (uint8_t *) data);
	if (ret != ATT_ERR_NO_ERROR) {
		println("Failed to attmdb_att_set_value: %d", ret);
		return ret;
	}

	prf_server_send_event((prf_env_struct *) &jwaoo_toy_env, false, handle);

	return ATT_ERR_NO_ERROR;
}

uint8_t jwaoo_toy_send_command_u8(uint8_t type, uint8_t value)
{
	struct jwaoo_toy_command command = {
		.type = type,
		.value8 = value
	};

	return jwaoo_toy_send_command((uint8_t *) &command, 2);
}

uint8_t jwaoo_toy_send_command_u16(uint8_t type, uint16_t value)
{
	struct jwaoo_toy_command command = {
		.type = type,
		.value16 = value
	};

	return jwaoo_toy_send_command((uint8_t *) &command, 3);
}

uint8_t jwaoo_toy_send_command_u32(uint8_t type, uint32_t value)
{
	struct jwaoo_toy_command command = {
		.type = type,
		.value32 = value
	};

	return jwaoo_toy_send_command((uint8_t *) &command, 5);
}

uint8_t jwaoo_toy_send_command_data(uint8_t type, const uint8_t *data, int size)
{
	uint8_t buff[size + 1];
	struct jwaoo_toy_command *command = (struct jwaoo_toy_command *) buff;

	command->type = type;
	memcpy(command->bytes, data, size);

	return jwaoo_toy_send_command(buff, sizeof(buff));
}

uint8_t jwaoo_toy_send_command_text(uint8_t type, const char *fmt, ...)
{
	va_list ap;
	int length;
	uint8_t buff[JWAOO_TOY_MAX_COMMAND_SIZE];

	buff[0] = type;

	va_start(ap, fmt);
	length = vsnprintf((char *) buff + 1, sizeof(buff) - 1, fmt, ap);
	va_end(ap);

	return jwaoo_toy_send_command(buff, length + 1);
}

// ===============================================================================

uint8_t jwaoo_toy_send_response_u8_typed(uint8_t type, uint8_t command, uint8_t value)
{
	struct jwaoo_toy_response response = {
		.command = command,
		.type = type,
		.value8 = value
	};

	return jwaoo_toy_send_command(&response, 3);
}

uint8_t jwaoo_toy_send_response_u8(uint8_t command, uint8_t value)
{
	return jwaoo_toy_send_response_u8_typed(JWAOO_TOY_RSP_U8, command, value);
}

uint8_t jwaoo_toy_send_response_bool(uint8_t command, bool value)
{
	return jwaoo_toy_send_response_u8_typed(JWAOO_TOY_RSP_BOOL, command, value);
}

uint8_t jwaoo_toy_send_response_u16(uint8_t command, uint16_t value)
{
	struct jwaoo_toy_response response = {
		.command = command,
		.type = JWAOO_TOY_RSP_U16,
		.value16 = value
	};

	return jwaoo_toy_send_command(&response, 4);
}

uint8_t jwaoo_toy_send_response_u32(uint8_t command, uint32_t value)
{
	struct jwaoo_toy_response response = {
		.command = command,
		.type = JWAOO_TOY_RSP_U32,
		.value32 = value
	};

	return jwaoo_toy_send_command(&response, 6);
}

uint8_t jwaoo_toy_send_response_data(uint8_t command, const uint8_t *data, uint16_t size)
{
	char buff[size + 2];
	struct jwaoo_toy_response *response = (void *) buff;

	response->command = command;
	response->type = JWAOO_TOY_RSP_DATA;
	memcpy(response->bytes, data, size);

	return jwaoo_toy_send_command(buff, sizeof(buff));
}

uint8_t jwaoo_toy_send_response_text(uint8_t command, const char *fmt, ...)
{
	va_list ap;
	int length;
	char buff[JWAOO_TOY_MAX_COMMAND_SIZE];
	struct jwaoo_toy_response *response = (void *) buff;

	response->command = command;
	response->type = JWAOO_TOY_RSP_TEXT;

	va_start(ap, fmt);
	length = vsnprintf(response->text, sizeof(buff) - 2, fmt, ap);
	va_end(ap);

	return jwaoo_toy_send_command(buff, length + 2);
}

// ===============================================================================

void jwaoo_toy_process_command(const struct jwaoo_toy_command *command, uint16_t length)
{
	bool success = false;

	if (length < 1) {
		println("Invalid command length: %d", length);
		return;
	}

	println("command = %d, length = %d", command->type, length);

	switch (command->type) {
	case JWAOO_TOY_CMD_NOOP:
		success = true;
		break;

	case JWAOO_TOY_CMD_IDENTIFY:
		jwaoo_toy_send_response_text(command->type, "%s", JWAOO_TOY_IDENTIFY);
		return;

	case JWAOO_TOY_CMD_VERSION:
		jwaoo_toy_send_response_u32(command->type, JWAOO_TOY_VERSION);
		return;

	case JWAOO_TOY_CMD_BUILD_DATE:
		jwaoo_toy_send_response_text(command->type, "%s %s", __DATE__, __TIME__);
		return;

	case JWAOO_TOY_CMD_REBOOT:
		if (jwaoo_toy_env.flash_upgrade) {
			break;
		}

		ke_timer_set(JWAOO_TOY_REBOOT, TASK_APP, 100);
		success = true;
		break;

	case JWAOO_TOY_CMD_SHUTDOWN:
		break;

	case JWAOO_TOY_CMD_I2C_RW:
		if (length >= 3) {
			int count = 0;
			uint8_t rdlen = command->i2c.rdlen;
			uint8_t wrlen = length - 3;
			struct i2c_message msgs[2];
			uint8_t response[rdlen + 2];

			if (wrlen > 0) {
				msgs[count].read = 0;
				msgs[count].count = wrlen;
				msgs[count].data = (uint8_t *) command->i2c.data;
				count++;
			}

			if (rdlen > 0) {
				msgs[count].read = 1;
				msgs[count].count = rdlen;
				msgs[count].data = response + 2;
				count++;
			}

			println("i2c: rdlen = %d, wrlen = %d, count = %d", rdlen, wrlen, count);

			if (count < 1 || i2c_transfer(command->i2c.slave, msgs, count) < 0) {
				break;
			}

			response[0] = command->type;
			response[1] = JWAOO_TOY_RSP_DATA;

			jwaoo_toy_send_command(response, sizeof(response));
			return;
		}
		break;

	case JWAOO_TOY_CMD_FACTORY_ENABLE:
		jwaoo_toy_set_factory_enable(length > 1 && command->enable.value);
		success = true;
		break;

	case JWAOO_TOY_CMD_LED_ENABLE:
		if (length >= 2) {
			struct jwaoo_pwm_device *device;

			if (command->led.index == 1) {
				device = &jwaoo_pwm_led1;
			} else if (command->led.index == 2) {
				device = &jwaoo_pwm_led2;
			} else {
				break;
			}

			if (length > 2 && command->led.enable > 0) {
				jwaoo_led_open(device);
			} else {
				jwaoo_led_close(device);
			}

			success = true;
		}
		break;

	case JWAOO_TOY_CMD_BATT_EVENT_ENABLE:
		jwaoo_toy_env.battery_report = (length > 1 && command->enable.value);
		success = true;
		break;

	// ================================================================================

	case JWAOO_TOY_CMD_BATT_INFO:
		break;

	case JWAOO_TOY_CMD_FLASH_ID:
		jwaoo_toy_send_response_u32(command->type, spi_flash_jedec_id);
		return;

	case JWAOO_TOY_CMD_FLASH_SIZE:
		jwaoo_toy_send_response_u32(command->type, spi_flash_size);
		return;

	case JWAOO_TOY_CMD_FLASH_PAGE_SIZE:
		jwaoo_toy_send_response_u32(command->type, spi_flash_page_size);
		return;

	case JWAOO_TOY_CMD_FLASH_WRITE_ENABLE:
		if (jwaoo_toy_env.flash_upgrade) {
			break;
		}

		jwaoo_toy_env.flash_write_enable = (length > 1 && command->enable.value);
		success = true;
		break;

	case JWAOO_TOY_CMD_FLASH_ERASE:
		if (jwaoo_toy_env.flash_write_enable) {
			println("spi_flash_block_erase SPI_PART_BACK_CODE");
			if (spi_flash_block_erase(SPI_PART_BACK_CODE, SPI_ERASE_MODE) != ERR_OK) {
				jwaoo_toy_env.flash_write_success = false;
				println("Failed to spi_flash_chip_erase SPI_PART_BACKUP_CODE");
				break;
			}

			success = true;
		}
		break;

	case JWAOO_TOY_CMD_FLASH_READ:
#if JWAOO_TOY_READ_FLASH_ENABLE
		if (length == 5) {
			uint32_t address;

			if (jwaoo_toy_env.flash_upgrade) {
				break;
			}

			address = command->value32;
			if (address < spi_flash_size) {
				uint8_t buff[JWAOO_TOY_MAX_FLASH_DATA_SIZE];
				int length = spi_flash_size - address;

				if (length > sizeof(buff)) {
					length = sizeof(buff);
				}

				length = spi_flash_read_data(buff, address, length);
				if (length > 0) {
					success = (jwaoo_toy_write_data(JWAOO_TOY_ATTR_FLASH_DATA, buff, length) == ATT_ERR_NO_ERROR);
				}
			}
		}
#endif
		break;

	case JWAOO_TOY_CMD_FLASH_SEEK:
		if (length == 5) {
			uint32_t offset = command->value32;
			if (offset < spi_flash_size) {
				jwaoo_toy_env.flash_write_offset = offset;
				success = true;
			}
		}
		break;

	case JWAOO_TOY_CMD_FLASH_WRITE_START:
		if (jwaoo_toy_env.flash_write_enable) {
			jwaoo_toy_moto_set_mode(0, 0);
			jwaoo_toy_sensor_set_enable(false);
			jwaoo_toy_env.flash_write_success = true;
			jwaoo_toy_env.flash_write_length = 0;
			jwaoo_toy_env.flash_write_offset = SPI_PART_BACK_CODE;
			jwaoo_toy_env.flash_write_crc = 0xFF;
			jwaoo_toy_env.flash_upgrade = true;
			success = true;
		}
		break;

	case JWAOO_TOY_CMD_FLASH_WRITE_FINISH:
		if (length != 4) {
			break;
		}

		if (jwaoo_toy_env.flash_upgrade && jwaoo_toy_env.flash_write_enable && jwaoo_toy_env.flash_write_success) {
			println("remote: crc = 0x%02x, length = %d", command->upgrade.crc, command->upgrade.length);
			println("local:  crc = 0x%02x, length = %d",
				jwaoo_toy_env.flash_write_crc, jwaoo_toy_env.flash_write_length);

			if (command->upgrade.crc != jwaoo_toy_env.flash_write_crc) {
				println("crc not match");
				break;
			}

			if (command->upgrade.length != jwaoo_toy_env.flash_write_length) {
				println("length not match");
				break;
			}

			ke_timer_set(JWAOO_TOY_UPGRADE_COMPLETE, TASK_JWAOO_TOY, 1);
			success = true;
		}
		break;

	case JWAOO_TOY_CMD_FLASH_READ_BD_ADDR:
		jwaoo_toy_send_response_data(command->type, jwaoo_toy_device_data.bd_addr, sizeof(jwaoo_toy_device_data.bd_addr));
		return;

	case JWAOO_TOY_CMD_FLASH_WRITE_BD_ADDR:
		if (length == 7 && jwaoo_toy_env.flash_write_enable) {
			success = jwaoo_toy_write_bd_addr(command->bytes);
		}
		break;

	// ================================================================================

	case JWAOO_TOY_CMD_SENSOR_ENABLE:
		if (jwaoo_toy_env.flash_upgrade) {
			break;
		}

		if (length > 1 && command->enable.value) {
			if (length > 5) {
				jwaoo_toy_env.sensor_poll_delay = command->enable.delay32 / 10;
			}

			success = jwaoo_toy_sensor_set_enable(true);
			jwaoo_toy_env.sensor_enable = success;
		} else {
			success = jwaoo_toy_sensor_set_enable(false);
			jwaoo_toy_env.sensor_enable = false;
		}
		break;

	// ================================================================================

	case JWAOO_TOY_CMD_MOTO_SET_MODE:
		if (jwaoo_toy_env.flash_upgrade || length != 3) {
			break;
		}

		success = jwaoo_toy_moto_set_mode(command->moto.mode, command->moto.level);
		break;

	// ================================================================================

	case JWAOO_TOY_CMD_KEY_CLICK_ENABLE:
		jwaoo_toy_env.key_click_enable = (length > 1 && command->enable.value);
		success = true;
		break;

	case JWAOO_TOY_CMD_KEY_LONG_CLICK_ENABLE:
		if (length > 1 && command->enable.value) {
			jwaoo_toy_env.key_long_click_enable = true;

			if (length > 3) {
				uint16_t delay = command->enable.delay16 / 10;

				if (delay > 0) {
					jwaoo_toy_env.key_long_click_delay = delay;
				}
			}
		} else {
			jwaoo_toy_env.key_long_click_enable = false;
		}

		success = true;
		break;

	case JWAOO_TOY_CMD_KEY_MULTI_CLICK_ENABLE:
		if (length > 1 && command->enable.value) {
			jwaoo_toy_env.key_multi_click_enable = true;

			if (length > 3) {
				uint16_t delay = command->enable.delay16 / 10;

				if (delay > 0) {
					jwaoo_toy_env.key_multi_click_delay = delay;
				}
			}
		} else {
			jwaoo_toy_env.key_multi_click_enable = false;
		}

		success = true;
		break;

	// ================================================================================

	case JWAOO_TOY_CMD_GPIO_GET:
		if (length == 3) {
			uint8_t value = GPIO_GetPinStatus((GPIO_PORT) command->gpio.port, (GPIO_PIN) command->gpio.pin);
			jwaoo_toy_send_response_u8(command->type, value);
			return;
		}
		break;

	case JWAOO_TOY_CMD_GPIO_SET:
		if (length == 4) {
			if (command->gpio.value) {
				GPIO_SetActive((GPIO_PORT) command->gpio.port, (GPIO_PIN) command->gpio.pin);
			} else {
				GPIO_SetInactive((GPIO_PORT) command->gpio.port, (GPIO_PIN) command->gpio.pin);
			}

			success = true;
		}
		break;

	case JWAOO_TOY_CMD_GPIO_CFG:
		if (length == 6) {
			GPIO_ConfigurePin((GPIO_PORT) command->gpio_config.port, (GPIO_PIN) command->gpio_config.pin,
				(GPIO_PUPD) command->gpio_config.mode, (GPIO_FUNCTION) command->gpio_config.function, command->gpio_config.high > 0);
			success = true;
		}
		break;

	// ================================================================================

	default:
		println("Invalid command: %d", command->type);
		break;
	}

	jwaoo_toy_send_response_bool(command->type, success);
}

static bool jwaoo_toy_process_flash_data_safe(const uint8_t *data, int length)
{
	int wrlen;

	wrlen = spi_flash_write_data((uint8_t *) data, jwaoo_toy_env.flash_write_offset, length);
	if (wrlen < 0) {
		println("Failed to spi_flash_write_data: %d", wrlen);
		return false;
	}

	jwaoo_toy_env.flash_write_crc = jwaoo_toy_calculate_crc(data, length, jwaoo_toy_env.flash_write_crc);
	jwaoo_toy_env.flash_write_offset += wrlen;
	jwaoo_toy_env.flash_write_length += wrlen;

	return true;
}

bool jwaoo_toy_process_flash_data(const uint8_t *data, uint16_t length)
{
	if (jwaoo_toy_env.flash_write_enable) {
		if (jwaoo_toy_process_flash_data_safe(data, length)) {
			return true;
		}

		println("Failed to jwaoo_toy_process_flash_data_safe");
		jwaoo_toy_env.flash_write_enable = false;
	} else {
		println("write flash is not enable");
	}

	jwaoo_toy_env.flash_write_success = false;

	return false;
}

void jwaoo_toy_battery_led_blink(void)
{
	if (jwaoo_toy_env.battery_led_locked < 2) {
		jwaoo_toy_env.battery_led_locked = 1;
		jwaoo_led_blink_square(&jwaoo_pwm_led1, 50, 2);
	}
}

void jwaoo_toy_battery_led_release(void)
{
	jwaoo_toy_env.battery_led_locked = 0;
	jwaoo_toy_battery_led_update_state();
}

void jwaoo_toy_battery_led_update_state(void)
{
	if (jwaoo_toy_env.battery_led_locked) {
		return;
	}

	switch (jwaoo_toy_env.battery_state) {
	case JWAOO_TOY_BATTERY_LOW:
		if (app_suspended) {
			jwaoo_led_close(&jwaoo_pwm_led1);
		} else {
			jwaoo_led_blink_square(&jwaoo_pwm_led1, 500, 0);
		}
		break;

	case JWAOO_TOY_BATTERY_FULL:
		jwaoo_led_open(&jwaoo_pwm_led1);
		break;

	case JWAOO_TOY_BATTERY_CHARGING:
		jwaoo_led_blink_sawtooth(&jwaoo_pwm_led1, 2000, 0);
		break;

	default:
		jwaoo_led_close(&jwaoo_pwm_led1);
	}
}

void jwaoo_toy_battery_set_state(uint8_t state)
{
	if (jwaoo_toy_env.battery_state != state) {
		jwaoo_toy_env.battery_state = state;
		jwaoo_toy_battery_led_update_state();
	}
}

// ================================================================================

static void jwaoo_toy_report_key_state(struct jwaoo_toy_key *key, uint8_t value)
{
	struct jwaoo_toy_key_message *msg = KE_MSG_ALLOC(JWAOO_TOY_KEY_REPORT_STATE, TASK_JWAOO_TOY, TASK_APP, jwaoo_toy_key_message);

	msg->key = key;
	msg->value = key->value;

	ke_msg_send(msg);
}

static void jwaoo_toy_report_key_click(struct jwaoo_toy_key *key, uint8_t count)
{
	struct jwaoo_toy_key_message *msg = KE_MSG_ALLOC(JWAOO_TOY_KEY_REPORT_CLICK, TASK_JWAOO_TOY, TASK_APP, jwaoo_toy_key_message);

	msg->key = key;
	msg->count = count;

	ke_msg_send(msg);
}

static void jwaoo_toy_report_key_long_click(struct jwaoo_toy_key *key)
{
	struct jwaoo_toy_key_message *msg = KE_MSG_ALLOC(JWAOO_TOY_KEY_REPORT_LONG_CLICK, TASK_JWAOO_TOY, TASK_APP, jwaoo_toy_key_message);

	msg->key = key;

	ke_msg_send(msg);
}

static uint8_t jwaoo_toy_moto_speed_add(void)
{
	uint8_t speed = jwaoo_pwm_moto.level + 1;

	if (speed > MOTO_SPEED_MAX) {
		speed = MOTO_SPEED_MAX;
	}

	jwaoo_toy_moto_set_mode(0, speed);

	return speed;
}

static uint8_t jwaoo_toy_moto_speed_sub(void)
{
	uint8_t speed;

	if (jwaoo_pwm_moto.level > 1) {
		speed = jwaoo_pwm_moto.level - 1;
	} else {
		speed = 0;
	}

	jwaoo_toy_moto_set_mode(0, speed);

	return speed;
}

static uint8_t jwaoo_toy_moto_mode_add(void)
{
	uint8_t mode = jwaoo_toy_env.moto_mode + 1;
	if (mode > JWAOO_TOY_MOTO_MODE_MAX) {
		mode = 1;
	}

	jwaoo_toy_moto_set_mode(mode, 0);

	return mode;
}

static void jwaoo_toy_on_key_clicked(struct jwaoo_toy_key *key, uint8_t count)
{
	println("clicked%d: value = %d, count = %d, repeat = %d", key->code, key->value, key->count, key->repeat);

	jwaoo_toy_battery_led_blink();

	switch (key->code) {
	case JWAOO_TOY_KEYCODE_UP:
		jwaoo_toy_moto_speed_add();
		break;

	case JWAOO_TOY_KEYCODE_DOWN:
		if (jwaoo_toy_moto_speed_sub() == 0 && key->repeat > 0) {
			user_app_set_suspend(true, true);
		}
		break;

	case JWAOO_TOY_KEYCODE_O:
		if (key->value == JWAOO_TOY_KEY_VALUE_DOWN) {
			jwaoo_toy_moto_mode_add();
		}
		break;
	}
}

static void jwaoo_toy_on_key_long_clicked(struct jwaoo_toy_key *key)
{
	println("long clicked: code = %d", key->code);
}

// ================================================================================

void jwaoo_toy_process_key_repeat(struct jwaoo_toy_key *key)
{
	if (key->value < JWAOO_TOY_KEY_VALUE_REPEAT) {
		key->value = JWAOO_TOY_KEY_VALUE_REPEAT;
	}

	if (key->repeat_enable) {
		jwaoo_toy_on_key_clicked(key, key->count);
	}
}

void jwaoo_toy_process_key_multi_click(struct jwaoo_toy_key *key)
{
	if (jwaoo_toy_env.key_long_click_enable == false || key->last_value != JWAOO_TOY_KEY_VALUE_LONG) {
		if (jwaoo_toy_env.key_multi_click_enable) {
			jwaoo_toy_report_key_click(key, key->count);
		}
	}

	if (key->long_click_enable == false || key->last_value != JWAOO_TOY_KEY_VALUE_LONG) {
		if (key->multi_click_enable) {
			jwaoo_toy_on_key_clicked(key, key->count);
		}
	}

	key->count = 0;
}

void jwaoo_toy_process_key_long_click(struct jwaoo_toy_key *key)
{
	key->value = JWAOO_TOY_KEY_VALUE_LONG;

	if (jwaoo_toy_env.key_long_click_enable) {
		jwaoo_toy_report_key_long_click(key);
	}

	if (key->long_click_enable) {
		jwaoo_toy_on_key_long_clicked(key);
	}
}

static void jwaoo_toy_key_timer_clear(struct jwaoo_toy_key *key) {
	ke_timer_clear(key->repeat_timer, TASK_APP);
	ke_timer_clear(key->long_click_timer, TASK_APP);
	ke_timer_clear(key->multi_click_timer, TASK_APP);
}

bool jwaoo_toy_process_key_lock(void)
{
	struct jwaoo_toy_key *key;
	struct jwaoo_toy_key *key_end = jwaoo_toy_keys + NELEM(jwaoo_toy_keys);

	for (key = jwaoo_toy_keys; key < key_end; key++) {
		if (key->lock_enable && key->value == 0) {
			if (jwaoo_toy_env.key_lock_pending) {
				ke_timer_clear(JWAOO_TOY_KEY_LOCK, TASK_APP);
				jwaoo_toy_env.key_lock_pending = false;
				jwaoo_toy_battery_led_release();
			}

			return false;
		}
	}

	if (jwaoo_toy_env.key_lock_pending) {
		return true;
	}

	jwaoo_toy_env.key_lock_pending = true;

	for (key = jwaoo_toy_keys; key < key_end; key++) {
		jwaoo_toy_key_timer_clear(key);

		if (key->lock_enable) {
			key->skip = 1;
		}
	}

	jwaoo_toy_env.battery_led_locked = 2;

	if (jwaoo_toy_env.key_locked) {
		jwaoo_led_close(&jwaoo_pwm_led1);
	} else {
		jwaoo_led_open(&jwaoo_pwm_led1);
	}

	ke_timer_set(JWAOO_TOY_KEY_LOCK, TASK_APP, 300);

	return true;
}

void jwaoo_toy_process_key(uint8_t index, uint8_t value)
{
	struct jwaoo_toy_key *key = jwaoo_toy_keys + index;

	// println("%d. code = %d, value = %d", index, key->code, value);

	jwaoo_toy_key_timer_clear(key);

	if (value == 0) {
		key->last_value = key->value;
	}

	key->value = value;

	if (jwaoo_toy_env.flash_upgrade) {
		return;
	}

	if (jwaoo_toy_env.factory_enable) {
		jwaoo_toy_report_key_state(key, value);
		return;
	}

	if (app_suspended) {
		if (key->code == JWAOO_TOY_KEYCODE_UP) {
			key->skip = 1;
			user_app_set_suspend(false, true);
		}

		return;
	}

	user_app_update_suspend_timer();

	if (jwaoo_toy_process_key_lock()) {
		return;
	}

	if (jwaoo_toy_env.key_locked) {
		return;
	}

	if (key->skip > 0) {
		if (value == 0) {
			key->skip--;
		}

		return;
	}

	if (value > 0) {
		key->count++;
		key->repeat = 0;
		ke_timer_set(key->repeat_timer, TASK_APP, JWAOO_TOY_KEY_REPEAT_LONG);
		ke_timer_set(key->long_click_timer, TASK_APP, jwaoo_toy_env.key_long_click_delay);
	} else {
		ke_timer_set(key->multi_click_timer, TASK_APP, jwaoo_toy_env.key_multi_click_delay);
	}

	if (key->multi_click_enable == false && (key->repeat_enable == false || key->repeat == 0)) {
		if (key->long_click_enable || key->lock_enable) {
			if (value == 0 && key->last_value != JWAOO_TOY_KEY_VALUE_LONG) {
				jwaoo_toy_on_key_clicked(key, 1);
			}
		} else if (value > 0) {
			jwaoo_toy_on_key_clicked(key, 1);
		}
	}

	if (!jwaoo_toy_env.key_multi_click_enable) {
		if (jwaoo_toy_env.key_long_click_enable) {
			if (value == 0 && key->last_value != JWAOO_TOY_KEY_VALUE_LONG) {
				jwaoo_toy_report_key_click(key, 1);
			}
		} else if (jwaoo_toy_env.key_click_enable) {
			if (value > 0) {
				jwaoo_toy_report_key_click(key, 1);
			}
		} else {
			jwaoo_toy_report_key_state(key, value);
		}
	}
}

#endif //BLE_JWAOO_TOY_SERVER

/// @} JWAOO_TOY
