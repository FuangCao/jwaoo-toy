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
#include "stdarg.h"
#include "uart.h"
#include "spi_flash.h"
#include "attm_util.h"
#include "atts_util.h"
#include "jwaoo_toy.h"
#include "jwaoo_toy_task.h"
#include "prf_utils.h"
#include "app_easy_timer.h"
#include "mpu6050.h"
#include "fdc1004.h"
#include "user_periph_setup.h"

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

extern struct jwaoo_toy_log_cache_tag uart2_log_cache;

struct jwaoo_toy_env_tag jwaoo_toy_env __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY
struct jwaoo_toy_device_data_tag jwaoo_toy_device_data __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY

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

uint8_t jwaoo_toy_sensor_poll(void)
{
	uint8_t buff[8];

	if (jwaoo_toy_env.mpu6050_dead < 10) {
		if (i2c_read_data(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, buff, 6) < 0) {
			jwaoo_toy_env.mpu6050_dead++;
			memset(buff, 0x00, 3);
		} else {
			jwaoo_toy_env.mpu6050_dead = 0;
			buff[1] = buff[2];
			buff[2] = buff[4];
		}
	} else {
		memset(buff, 0x00, 3);
	}

	if (jwaoo_toy_env.fdc1004_dead < 10) {
		if (fdc1004_read_capacity_simple(buff + 3) < 0) {
			jwaoo_toy_env.fdc1004_dead++;
			memset(buff + 3, 0x00, 4);
		} else {
			jwaoo_toy_env.fdc1004_dead = 0;
		}
	} else {
		memset(buff + 3, 0x00, 4);
	}

	return jwaoo_toy_send_notify(JWAOO_TOY_ATTR_SENSOR_DATA, buff, 7);
}

static bool jwaoo_toy_sensor_set_enable(bool enable)
{
	println("sensor_enable = %d, sensor_poll_delay = %d", enable, jwaoo_toy_env.sensor_poll_delay);

	if (enable) {
		if (jwaoo_toy_env.sensor_poll_delay > 0) {
			jwaoo_toy_env.sensor_poll_mode = JWAOO_SENSOR_POLL_MODE_SLOW;

			if (!ke_timer_active(JWAOO_TOY_SENSOR_POLL, TASK_JWAOO_TOY)) {
				ke_timer_set(JWAOO_TOY_SENSOR_POLL, TASK_JWAOO_TOY, 0);
			}
		} else {
			jwaoo_toy_env.sensor_poll_mode = JWAOO_SENSOR_POLL_MODE_FAST;

			ke_timer_clear(JWAOO_TOY_SENSOR_POLL, TASK_JWAOO_TOY);
			if (jwaoo_toy_sensor_notify_busy() == 0) {
				jwaoo_toy_sensor_poll();
			}
		}
	} else {
		jwaoo_toy_env.sensor_poll_mode = JWAOO_SENSOR_POLL_MODE_NONE;
		ke_timer_clear(JWAOO_TOY_SENSOR_POLL, TASK_JWAOO_TOY);
	}

	println("sensor_poll_mode = %d", jwaoo_toy_env.sensor_poll_mode);

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

bool jwaoo_toy_flash_copy_code(uint32_t rdaddr, uint32_t wraddr, uint32_t size, uint8_t crc_raw)
{
	uint8_t crc = 0xFF;

	println("%s: 0x%04x [%d]=> 0x%04x", __FUNCTION__, rdaddr, size, wraddr);

	while (size > 0) {
		int ret;
		int length;
		static uint8_t buff[64];

		length = (size > sizeof(buff) ? sizeof(buff) : size);

		ret = spi_flash_read_data(buff, rdaddr, length);
		if (ret != length) {
			println("Failed to spi_flash_read_data: %d", ret);
			return false;
		}

		ret = spi_flash_write_data(buff, wraddr, length);
		if (ret != length) {
			println("Failed to spi_flash_write_data: %d", ret);
			return false;
		}

		crc = jwaoo_toy_calculate_crc(buff, length, crc);
		rdaddr += length;
		wraddr += length;
		size -= length;
	}

	if (crc != crc_raw) {
		println("crc not match: 0x%02x != 0x%02x", crc, crc_raw);
		return false;
	}

	println("%s successfull", __FUNCTION__);

	return true;
}

void jwaoo_toy_init(void)
{
    // Reset environment
    memset(&jwaoo_toy_env, 0, sizeof(jwaoo_toy_env));

	jwaoo_toy_env.sensor_poll_delay = 20;
	jwaoo_toy_env.key_long_click_delay = 300;
	jwaoo_toy_env.key_multi_click_delay = 30;

    // Create JWAOO_TOY task
    ke_task_create(TASK_JWAOO_TOY, &TASK_DESC_JWAOO_TOY);

    // Set task in disabled state
    ke_state_set(TASK_JWAOO_TOY, JWAOO_TOY_DISABLED);
}

void jwaoo_toy_enable(uint16_t conhdl) 
{
	jwaoo_toy_env.fdc1004_dead = 0;
	jwaoo_toy_env.mpu6050_dead = 0;

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
	jwaoo_toy_env.flash_write_enable = false;
	jwaoo_toy_env.flash_write_success = false;
	jwaoo_toy_sensor_set_enable(false);

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

static void jwaoo_toy_reboot_timer(void)
{
	pr_pos_info();

	platform_reset(RESET_AFTER_SPOTA_UPDATE);
}

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

		success = (app_easy_timer(100, jwaoo_toy_reboot_timer) != EASY_TIMER_INVALID_TIMER);
		break;

	case JWAOO_TOY_CMD_SHUTDOWN:
		break;

	case JWAOO_TOY_CMD_I2C_RW:
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
		jwaoo_toy_env.flash_write_enable = (length > 1 && command->enable);
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

		if (jwaoo_toy_env.flash_write_enable && jwaoo_toy_env.flash_write_success) {
			println("remote: crc = 0x%02x, length = %d", command->crc, command->length);
			println("local:  crc = 0x%02x, length = %d",
				jwaoo_toy_env.flash_write_crc, jwaoo_toy_env.flash_write_length);

			if (command->crc != jwaoo_toy_env.flash_write_crc) {
				println("crc not match");
				break;
			}

			if (command->length != jwaoo_toy_env.flash_write_length) {
				println("length not match");
				break;
			}

			println("jwaoo_toy_flash_check_crc SPI_PART_BACK_CODE");

			if (!jwaoo_toy_flash_check_crc(SPI_PART_BACK_CODE,
				jwaoo_toy_env.flash_write_length, jwaoo_toy_env.flash_write_crc)) {
				println("Failed to jwaoo_toy_flash_check_crc");
				break;
			}

			println("spi_flash_block_erase SPI_PART_FRONT_CODE");

			if (spi_flash_block_erase(SPI_PART_FRONT_CODE, SPI_ERASE_MODE) != ERR_OK) {
				println("Failed to spi_flash_block_erase SPI_PART_FRONT_CODE");
				break;
			}

			if (!jwaoo_toy_flash_copy_code(SPI_PART_BACK_CODE, SPI_PART_FRONT_CODE,
				jwaoo_toy_env.flash_write_length, jwaoo_toy_env.flash_write_crc)) {
				println("Failed to jwaoo_toy_copy_code");
				break;
			}

			println("jwaoo_toy_flash_check_crc SPI_PART_FRONT_CODE");

			if (!jwaoo_toy_flash_check_crc(SPI_PART_FRONT_CODE,
				jwaoo_toy_env.flash_write_length, jwaoo_toy_env.flash_write_crc)) {
				println("Failed to jwaoo_toy_flash_check_crc");
				break;
			}

			jwaoo_toy_env.flash_write_enable = false;
			jwaoo_toy_env.flash_upgrade = false;
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

		if (length > 1 && command->enable) {
			if (length > 5) {
				jwaoo_toy_env.sensor_poll_delay = command->delay32 / 10;
			}

			success = jwaoo_toy_sensor_set_enable(true);
			jwaoo_toy_env.sensor_enable = success;
		} else {
			success = jwaoo_toy_sensor_set_enable(false);
			jwaoo_toy_env.sensor_enable = false;
		}
		break;

	// ================================================================================

	case JWAOO_TOY_CMD_KEY_CLICK_ENABLE:
		jwaoo_toy_env.key_click_enable = length > 1 && command->enable;
		success = true;
		break;

	case JWAOO_TOY_CMD_KEY_LONG_CLICK_ENABLE:
		if (length > 1 && command->enable) {
			jwaoo_toy_env.key_long_click_enable = true;

			if (length > 3) {
				uint16_t delay = command->delay16 / 10;

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
		if (length > 1 && command->enable) {
			jwaoo_toy_env.key_multi_click_enable = true;

			if (length > 3) {
				uint16_t delay = command->delay16 / 10;

				if (delay > 0) {
					jwaoo_toy_env.key_multi_click_delay = delay;
				}
			}
		} else {
			jwaoo_toy_env.key_multi_click_enable = false;
		}
		success = true;
		break;

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
		} else {
			println("Failed to jwaoo_toy_process_flash_data_safe");
		}

		jwaoo_toy_env.flash_write_enable = false;
	} else {
		println("write flash is not enable");
	}

	jwaoo_toy_env.flash_write_success = false;

	return false;
}

uint8_t jwaoo_toy_report_key_value(uint8_t code, uint8_t value)
{
	uint8_t event[] = { JWAOO_TOY_EVT_KEY_STATE, code, value };

	return jwaoo_toy_send_event(event, sizeof(event));
}

uint8_t jwaoo_toy_report_key_click(uint8_t code, uint8_t count)
{
	jwaoo_toy_env.key_click_count = 0;

	println("clicked: count = %d", count);

	if (count > 0) {
		uint8_t event[] = { JWAOO_TOY_EVT_KEY_CLICK, code, count };

		return jwaoo_toy_send_event(event, sizeof(event));
	}

	return ATT_ERR_NO_ERROR;
}

uint8_t jwaoo_toy_report_key_long_click(uint8_t code)
{
	uint8_t event[] = { JWAOO_TOY_EVT_KEY_LONG_CLICK, code };

	println("long clicked: code = %d", code);

	jwaoo_toy_env.key_click_count = 0;

	return jwaoo_toy_send_event(event, sizeof(event));

}

void jwaoo_toy_process_key(uint8_t code, uint8_t value)
{
	println("code = %d, value = %d", code, value);

	if (jwaoo_toy_env.key_long_click_enable || jwaoo_toy_env.key_multi_click_enable) {
		ke_timer_clear(JWAOO_TOY_KEY_LONG_CLICK, TASK_JWAOO_TOY);
		ke_timer_clear(JWAOO_TOY_KEY_MULTI_CLICK, TASK_JWAOO_TOY);

		if (value > 0) {
			if (code == jwaoo_toy_env.key_click_code) {
				jwaoo_toy_env.key_click_count++;
			} else {
				jwaoo_toy_env.key_click_code = code;
				jwaoo_toy_env.key_click_count = 1;
			}

			ke_timer_set(JWAOO_TOY_KEY_LONG_CLICK, TASK_JWAOO_TOY, jwaoo_toy_env.key_long_click_delay);
		} else if (jwaoo_toy_env.key_multi_click_enable) {
			ke_timer_set(JWAOO_TOY_KEY_MULTI_CLICK, TASK_JWAOO_TOY, jwaoo_toy_env.key_multi_click_delay);
		} else {
			jwaoo_toy_report_key_click(code, jwaoo_toy_env.key_click_count);
		}
	} else if (jwaoo_toy_env.key_click_enable) {
		if (value > 0) {
			jwaoo_toy_report_key_click(code, 1);
		}
	} else {
		jwaoo_toy_report_key_value(code, value);
	}
}

#endif //BLE_JWAOO_TOY_SERVER

/// @} JWAOO_TOY
