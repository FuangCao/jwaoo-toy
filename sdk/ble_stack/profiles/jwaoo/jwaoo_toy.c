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

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * JWAOO_TOY ATTRIBUTES
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct jwaoo_toy_env_tag jwaoo_toy_env __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY

static const struct ke_task_desc TASK_DESC_JWAOO_TOY = {
	.state_handler = jwaoo_toy_state_handler,
	.default_handler = &jwaoo_toy_default_handler,
	.state = jwaoo_toy_state,
	.state_max = JWAOO_TOY_STATE_COUNT,
	.idx_max = JWAOO_TOY_TASK_COUNT
};

extern uint32_t spi_flash_size;
extern uint32_t spi_flash_page_size;
extern const SPI_FLASH_DEVICE_PARAMETERS_BY_JEDEC_ID_t *spi_flash_detected_device;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t jwaoo_toy_sensor_poll(void)
{
	uint8_t buff[6];

	if (jwaoo_toy_env.mpu6050_dead < 10) {
		if (i2c_read_data(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, buff, sizeof(buff)) < 0) {
			jwaoo_toy_env.mpu6050_dead++;
			buff[0] = buff[1] = buff[2] = 0;
		} else {
			jwaoo_toy_env.mpu6050_dead = 0;
			buff[1] = buff[2];
			buff[2] = buff[4];
		}
	} else {
		buff[0] = buff[1] = buff[2] = 0;
	}

	if (jwaoo_toy_env.fdc1004_dead < 10) {
		int depth = fdc1004_get_depth();
		if (depth < 0) {
			jwaoo_toy_env.fdc1004_dead++;
			buff[3] = 0;
		} else {
			jwaoo_toy_env.fdc1004_dead = 0;
			buff[3] = depth;
		}
	} else {
		buff[3] = 0;
	}

	return jwaoo_toy_send_notify(JWAOO_TOY_ATTR_SENSOR_DATA, buff, 4);
}

static bool jwaoo_toy_sensor_set_enable(bool enable)
{
	println("sensor_enable = %d, sensor_poll_delay = %d", enable, jwaoo_toy_env.sensor_poll_delay);

	if (enable) {
		if (jwaoo_toy_env.sensor_poll_delay < 2) {
			jwaoo_toy_env.sensor_poll_mode = JWAOO_SENSOR_POLL_MODE_FAST;

			ke_timer_clear(JWAOO_TOY_SENSOR_POLL, TASK_JWAOO_TOY);
			if (!jwaoo_toy_env.notify_busy) {
				jwaoo_toy_sensor_poll();
			}
		} else {
			jwaoo_toy_env.sensor_poll_mode = JWAOO_SENSOR_POLL_MODE_SLOW;

			if (!ke_timer_active(JWAOO_TOY_SENSOR_POLL, TASK_JWAOO_TOY)) {
				ke_timer_set(JWAOO_TOY_SENSOR_POLL, TASK_JWAOO_TOY, 0);
			}
		}
	} else {
		jwaoo_toy_env.sensor_poll_mode = JWAOO_SENSOR_POLL_MODE_NONE;
		ke_timer_clear(JWAOO_TOY_SENSOR_POLL, TASK_JWAOO_TOY);
	}

	println("sensor_poll_mode = %d", jwaoo_toy_env.sensor_poll_mode);

	return true;
}

void jwaoo_toy_init(void)
{
    // Reset environment
    memset(&jwaoo_toy_env, 0, sizeof(jwaoo_toy_env));

	jwaoo_toy_env.sensor_poll_delay = 20;

    // Create JWAOO_TOY task
    ke_task_create(TASK_JWAOO_TOY, &TASK_DESC_JWAOO_TOY);

    // Set task in disabled state
    ke_state_set(TASK_JWAOO_TOY, JWAOO_TOY_DISABLED);
}

void jwaoo_toy_enable(uint16_t conhdl) 
{
	jwaoo_toy_env.notify_busy = false;
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

	jwaoo_toy_env.flash_write_enable = false;
	jwaoo_toy_env.flash_write_ok = false;
	jwaoo_toy_sensor_set_enable(false);

    //Disable JWAOO_TOY in database
    attmdb_svc_set_permission(jwaoo_toy_env.handle, PERM(SVC, DISABLE));

    //Go to idle state
    ke_state_set(TASK_JWAOO_TOY, JWAOO_TOY_IDLE);
}

// ===============================================================================

uint8_t jwaoo_toy_write_data(uint16_t attr, const uint8_t *data, int size)
{
	return attmdb_att_set_value(jwaoo_toy_env.handle + attr, size, (uint8_t *) data);
}

uint8_t jwaoo_toy_send_notify(uint16_t attr, const uint8_t *data, int size)
{
	uint8_t ret;
	uint16_t handle = jwaoo_toy_env.handle + attr;

	jwaoo_toy_env.notify_busy = true;

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
	struct jwaoo_toy_command_u8 command = { type, value };

	return jwaoo_toy_send_command((uint8_t *) &command, sizeof(command));
}

uint8_t jwaoo_toy_send_command_u16(uint8_t type, uint16_t value)
{
	struct jwaoo_toy_command_u16 command = { type, value };

	return jwaoo_toy_send_command((uint8_t *) &command, sizeof(command));
}

uint8_t jwaoo_toy_send_command_u32(uint8_t type, uint32_t value)
{
	struct jwaoo_toy_command_u32 command = { type, value };

	return jwaoo_toy_send_command((uint8_t *) &command, sizeof(command));
}

uint8_t jwaoo_toy_send_command_data(uint8_t type, const uint8_t *data, int size)
{
	uint8_t buff[sizeof(struct jwaoo_toy_command) + size];
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

static void jwaoo_toy_reboot_timer(void)
{
	pr_pos_info();

	platform_reset(RESET_AFTER_SPOTA_UPDATE);
}

void jwaoo_toy_process_command(const struct jwaoo_toy_command *command)
{
	bool success = false;

	switch (command->type) {
	case JWAOO_TOY_CMD_NOP:
		success = true;
		break;

	case JWAOO_TOY_CMD_IDENTIFY:
		jwaoo_toy_send_response_text("%s", JWAOO_TOY_IDENTIFY);
		return;

	case JWAOO_TOY_CMD_VERSION:
		jwaoo_toy_send_response_u32(JWAOO_TOY_VERSION);
		return;

	case JWAOO_TOY_CMD_BUILD_DATE:
		jwaoo_toy_send_response_text("%s - %s", __DATE__, __TIME__);
		return;

	case JWAOO_TOY_CMD_REBOOT:
		success = (app_easy_timer(100, jwaoo_toy_reboot_timer) != EASY_TIMER_INVALID_TIMER);
		break;

	case JWAOO_TOY_CMD_SHUTDOWN:
		break;

	case JWAOO_TOY_CMD_BATT_INFO:
		break;

	case JWAOO_TOY_CMD_FLASH_ID:
		if (spi_flash_detected_device == NULL) {
			break;
		}

		jwaoo_toy_send_response_u32(spi_flash_detected_device->jedec_id);
		return;

	case JWAOO_TOY_CMD_FLASH_SIZE:
		jwaoo_toy_send_response_u32(spi_flash_size);
		return;

	case JWAOO_TOY_CMD_FLASH_PAGE_SIZE:
		jwaoo_toy_send_response_u32(spi_flash_page_size);
		return;

	case JWAOO_TOY_CMD_FLASH_WRITE_ENABLE:
		if (command->bytes[0]) {
			success = (spi_flash_set_write_enable() == ERR_OK);
			jwaoo_toy_env.flash_write_enable = success;
		} else {
			jwaoo_toy_env.flash_write_enable = false;
			success = true;
		}
		break;

	case JWAOO_TOY_CMD_FLASH_ERASE:
		if (jwaoo_toy_env.flash_write_enable) {
			if (spi_flash_chip_erase() != ERR_OK) {
				break;
			}

			success = true;
		}
		break;

	case JWAOO_TOY_CMD_FLASH_READ: {
		uint32_t address = ((const struct jwaoo_toy_command_u32 *) command)->value;
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
		break;
	}

	case JWAOO_TOY_CMD_FLASH_SEEK: {
		uint32_t offset = ((const struct jwaoo_toy_command_u32 *) command)->value;
		if (offset < spi_flash_size) {
			jwaoo_toy_env.flash_write_address = offset;
			success = true;
		}
		break;
	}

	case JWAOO_TOY_CMD_FLASH_WRITE_START:
		if (jwaoo_toy_env.flash_write_enable) {
			jwaoo_toy_sensor_set_enable(false);
			jwaoo_toy_env.flash_write_address = 0;
			jwaoo_toy_env.flash_write_ok = true;
#if JWAOO_TOY_FLASH_CACHE_SIZE > 0
			jwaoo_toy_env.flash_cache_size = 0;
#endif
			success = true;
		}
		break;

	case JWAOO_TOY_CMD_FLASH_WRITE_FINISH:
		println("flash_write_ok = %d", jwaoo_toy_env.flash_write_ok);
		if (jwaoo_toy_env.flash_write_enable && jwaoo_toy_env.flash_write_ok) {
#if JWAOO_TOY_FLASH_CACHE_SIZE > 0
			if (jwaoo_toy_env.flash_cache_size > 0 && spi_flash_write_data(jwaoo_toy_env.flash_data_cache, jwaoo_toy_env.flash_write_address, jwaoo_toy_env.flash_cache_size) < 0) {
				break;
			}
#endif

			jwaoo_toy_env.flash_write_enable = false;
			success = true;
		}
		break;

	case JWAOO_TOY_CMD_SENSOR_ENABLE:
		if (command->bytes[0]) {
			success = jwaoo_toy_sensor_set_enable(true);
			jwaoo_toy_env.sensor_enable = success;
		} else {
			success = jwaoo_toy_sensor_set_enable(false);
			jwaoo_toy_env.sensor_enable = false;
		}
		break;

	case JWAOO_TOY_CMD_SENSOR_SET_DELAY:
		jwaoo_toy_env.sensor_poll_delay = ((const struct jwaoo_toy_command_u32 *) command)->value / 10;
		if (jwaoo_toy_env.sensor_enable) {
			jwaoo_toy_sensor_set_enable(true);
		}

		success = true;
		break;

	default:
		break;
	}

	jwaoo_toy_send_response_bool(success);
}

void jwaoo_toy_process_flash_data(const uint8_t *data, int length)
{
	if (jwaoo_toy_env.flash_write_enable) {
#if JWAOO_TOY_FLASH_CACHE_SIZE > 0
		int remain = sizeof(jwaoo_toy_env.flash_data_cache) - jwaoo_toy_env.flash_cache_size;

		if (length < remain) {
			memcpy(jwaoo_toy_env.flash_data_cache + jwaoo_toy_env.flash_cache_size, data, length);
			jwaoo_toy_env.flash_cache_size += length;
		} else {
			int wrlen;

			memcpy(jwaoo_toy_env.flash_data_cache + jwaoo_toy_env.flash_cache_size, data, remain);

			wrlen = spi_flash_write_data(jwaoo_toy_env.flash_data_cache, jwaoo_toy_env.flash_write_address, sizeof(jwaoo_toy_env.flash_data_cache));
			if (wrlen > 0) {
				jwaoo_toy_env.flash_write_address += wrlen;
				jwaoo_toy_env.flash_cache_size = length - remain;
				memcpy(jwaoo_toy_env.flash_data_cache, data + remain, jwaoo_toy_env.flash_cache_size);
			} else {
				jwaoo_toy_env.flash_write_enable = false;
				jwaoo_toy_env.flash_write_ok = false;
			}
		}
#else
	int wrlen = spi_flash_write_data((uint8_t *) data, jwaoo_toy_env.flash_write_address, length);
	if (wrlen < 0) {
		println("Failed to spi_flash_write_data: %d", wrlen);
		jwaoo_toy_env.flash_write_enable = false;
		jwaoo_toy_env.flash_write_ok = false;
	} else {
		jwaoo_toy_env.flash_write_address += wrlen;
	}
#endif
	} else {
		jwaoo_toy_env.flash_write_ok = false;
	}
}

#endif //BLE_JWAOO_TOY_SERVER

/// @} JWAOO_TOY
