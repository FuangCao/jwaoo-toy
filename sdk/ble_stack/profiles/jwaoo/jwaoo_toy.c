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
#include "uart.h"
#include "spi_flash.h"
#include "attm_util.h"
#include "atts_util.h"
#include "jwaoo_toy.h"
#include "jwaoo_toy_task.h"
#include "prf_utils.h"

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

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void jwaoo_toy_init(void)
{
    // Reset environment
    memset(&jwaoo_toy_env, 0, sizeof(jwaoo_toy_env));

    // Create JWAOO_TOY task
    ke_task_create(TASK_JWAOO_TOY, &TASK_DESC_JWAOO_TOY);

    // Set task in disabled state
    ke_state_set(TASK_JWAOO_TOY, JWAOO_TOY_DISABLED);
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

    //Disable JWAOO_TOY in database
    attmdb_svc_set_permission(jwaoo_toy_env.handle, PERM(SVC, DISABLE));

    //Go to idle state
    ke_state_set(TASK_JWAOO_TOY, JWAOO_TOY_IDLE);
}

uint8_t jwaoo_toy_write_data(uint16_t attr, const uint8_t *data, int size)
{
	return attmdb_att_set_value(jwaoo_toy_env.handle + attr, size, (uint8_t *) data);
}

uint8_t jwaoo_toy_send_notify(uint16_t attr, const uint8_t *data, int size)
{
	uint8_t ret;
	uint16_t handle = jwaoo_toy_env.handle + attr;
	static uint8_t buff[JWAOO_TOY_MAX_DATA_SIZE];

	memcpy(buff, data, size);

	ret = attmdb_att_set_value(handle, size, buff);
	if (ret != ATT_ERR_NO_ERROR) {
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

extern uint32_t spi_flash_size;

void jwaoo_toy_process_command(const struct jwaoo_toy_command *command)
{
	bool success = false;

	switch (command->type) {
	case JWAOO_TOY_CMD_FLASH_WEN:
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
			success = (spi_flash_chip_erase() == ERR_OK);
			jwaoo_toy_env.flash_write_ok = success;
			jwaoo_toy_env.flash_write_address = 0;
		}
		break;

	case JWAOO_TOY_CMD_FLASH_READ: {
		uint32_t address = ((const struct jwaoo_toy_command_u32 *) command)->value;
		if (address < spi_flash_size) {
			uint8_t buff[JWAOO_TOY_MAX_DATA_SIZE];
			int length = spi_flash_size - address;

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
			success = true;
			jwaoo_toy_env.flash_write_address = offset;
		}
		break;
	}

	case JWAOO_TOY_CMD_FLASH_WRITE_OK:
		success = jwaoo_toy_env.flash_write_ok;
		println("flash_write_ok = %d", success);
		break;

	case JWAOO_TOY_CMD_SENSOR_ENABLE:
		break;

	case JWAOO_TOY_CMD_SENSOR_SET_DELAY:
		break;
	}

	jwaoo_toy_send_response_bool(success);
}

void jwaoo_toy_process_flash_data(const uint8_t *data, int length)
{
	if (jwaoo_toy_env.flash_write_enable) {
		int wrlen = spi_flash_write_data((uint8_t *) data, jwaoo_toy_env.flash_write_address, length);
		if (wrlen > 0) {
			jwaoo_toy_env.flash_write_address += wrlen;
		} else {
			jwaoo_toy_env.flash_write_ok = false;
		}
	} else {
		jwaoo_toy_env.flash_write_ok = false;
	}
}

#endif //BLE_JWAOO_TOY_SERVER

/// @} JWAOO_TOY
