/**
 ****************************************************************************************
 *
 * @file jwaoo_toy.h
 *
 * @brief Header file - Device Information Service Server.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

#ifndef JWAOO_TOY_H_
#define JWAOO_TOY_H_

#pragma anon_unions

/**
 ****************************************************************************************
 * @addtogroup JWAOO_TOY Device Information Service Server
 * @ingroup JWAOO_TOY
 * @brief Device Information Service Server
 * @{
 ****************************************************************************************
 */
 
#define BLE_JWAOO_TOY_SERVER	1

#ifndef BLE_SERVER_PRF
#define BLE_SERVER_PRF			1
#endif 

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if (BLE_JWAOO_TOY_SERVER)
#include "prf_types.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define JWAOO_TOY_IDENTIFY				"JwaooToy"
#define JWAOO_TOY_VERSION				0x20160702

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

#define JWAOO_TOY_MAX_COMMAND_SIZE		32
#define JWAOO_TOY_MAX_EVENT_SIZE		20
#define JWAOO_TOY_MAX_FLASH_DATA_SIZE	20
#define JWAOO_TOY_MAX_SENSOR_DATA_SIZE	20
#define JWAOO_TOY_MAX_DEBUG_DATA_SIZE	20

enum
{
	JWAOO_TOY_UUID_SVC = 0x1888,
	JWAOO_TOY_UUID_COMMAND,
	JWAOO_TOY_UUID_EVENT,
	JWAOO_TOY_UUID_FLASH,
	JWAOO_TOY_UUID_SENSOR,
	JWAOO_TOY_UUID_DEBUG,
	JWAOO_TOY_UUID_MAX
};

enum
{
	JWAOO_TOY_ATTR_SVC,

	JWAOO_TOY_ATTR_COMMAND_CHAR,
	JWAOO_TOY_ATTR_COMMAND_DATA,

	JWAOO_TOY_ATTR_EVENT_CHAR,
	JWAOO_TOY_ATTR_EVENT_DATA,
	JWAOO_TOY_ATTR_EVENT_CFG,

	JWAOO_TOY_ATTR_FLASH_CHAR,
	JWAOO_TOY_ATTR_FLASH_DATA,

	JWAOO_TOY_ATTR_SENSOR_CHAR,
	JWAOO_TOY_ATTR_SENSOR_DATA,
	JWAOO_TOY_ATTR_SENSOR_CFG,

	JWAOO_TOY_ATTR_DEBUG_CHAR,
	JWAOO_TOY_ATTR_DEBUG_DATA,
	JWAOO_TOY_ATTR_DEBUG_CFG,

	JWAOO_TOY_ATTR_COUNT,
};

enum
{
	JWAOO_TOY_RSP_BOOL,
	JWAOO_TOY_RSP_U8,
	JWAOO_TOY_RSP_U16,
	JWAOO_TOY_RSP_U32,
	JWAOO_TOY_RSP_DATA,
	JWAOO_TOY_RSP_TEXT,
	JWAOO_TOY_CMD_NOOP = 20,
	JWAOO_TOY_CMD_IDENTIFY,
	JWAOO_TOY_CMD_VERSION,
	JWAOO_TOY_CMD_BUILD_DATE,
	JWAOO_TOY_CMD_REBOOT,
	JWAOO_TOY_CMD_SHUTDOWN,
	JWAOO_TOY_CMD_BATT_INFO,
	JWAOO_TOY_CMD_I2C_RW,
	JWAOO_TOY_CMD_FLASH_ID = 50,
	JWAOO_TOY_CMD_FLASH_SIZE,
	JWAOO_TOY_CMD_FLASH_PAGE_SIZE,
	JWAOO_TOY_CMD_FLASH_READ,
	JWAOO_TOY_CMD_FLASH_SEEK,
	JWAOO_TOY_CMD_FLASH_ERASE,
	JWAOO_TOY_CMD_FLASH_WRITE_ENABLE,
	JWAOO_TOY_CMD_FLASH_WRITE_START,
	JWAOO_TOY_CMD_FLASH_WRITE_FINISH,
	JWAOO_TOY_CMD_FLASH_READ_BD_ADDR,
	JWAOO_TOY_CMD_FLASH_WRITE_BD_ADDR,
	JWAOO_TOY_CMD_SENSOR_ENABLE = 70,
	JWAOO_TOY_CMD_MOTO_ENABLE = 80,
	JWAOO_TOY_CMD_KEY_CLICK_ENABLE = 90,
	JWAOO_TOY_CMD_KEY_LONG_CLICK_ENABLE,
	JWAOO_TOY_CMD_KEY_MULTI_CLICK_ENABLE,
};

enum
{
	JWAOO_TOY_EVT_NOOP,
	JWAOO_TOY_EVT_BATT_INFO,
	JWAOO_TOY_EVT_KEY_STATE,
	JWAOO_TOY_EVT_KEY_CLICK,
	JWAOO_TOY_EVT_KEY_LONG_CLICK,
};

enum
{
	JWAOO_SENSOR_POLL_MODE_NONE,
	JWAOO_SENSOR_POLL_MODE_SLOW,
	JWAOO_SENSOR_POLL_MODE_FAST,
};

#pragma pack(1)

struct jwaoo_toy_command
{
	uint8_t type;

	union {
		char text[1];
		uint8_t bytes[1];
		uint8_t value8;
		uint16_t value16;
		uint32_t value32;

		struct {
			uint8_t crc;
			uint16_t length;
		};

		struct {
			uint8_t enable;

			union {
				uint8_t delay8;
				uint16_t delay16;
				uint32_t delay32;
			};
		};
	};
};

struct jwaoo_toy_response
{
	uint8_t command;
	uint8_t type;

	union {
		char text[1];
		uint8_t bytes[1];
		uint8_t value8;
		uint16_t value16;
		uint32_t value32;
	};
};

struct jwaoo_toy_device_data_tag {
	uint8_t bd_addr[6];
};
#pragma pack()

///Attributes State Machine
///Device Information Service Server Environment Variable
struct jwaoo_toy_env_tag
{
    /// Connection Info
    struct prf_con_info con_info;

    /// Service Start HandleVAL
    uint16_t handle;
	uint16_t notify_busy_mask;
	uint16_t notify_enable_mask;

	bool flash_upgrade;
	bool flash_write_enable;
	bool flash_write_success;
	uint8_t flash_write_crc;
	uint16_t flash_write_length;
	uint32_t flash_write_offset;

	uint8_t fdc1004_dead;
	uint8_t mpu6050_dead;

	bool sensor_enable;
	uint8_t sensor_poll_mode;
	uint16_t sensor_poll_delay;

	bool key_click_enable;
	bool key_long_click_enable;
	bool key_multi_click_enable;
	uint8_t key_click_code;
	uint8_t key_click_count;
	uint16_t key_long_click_delay;
	uint16_t key_multi_click_delay;
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern struct jwaoo_toy_env_tag jwaoo_toy_env;
extern struct jwaoo_toy_device_data_tag jwaoo_toy_device_data;


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the JWAOO_TOY module.
 * This function performs all the initializations of the JWAOO_TOY module.
 ****************************************************************************************
 */
void jwaoo_toy_init(void);

/**
 ****************************************************************************************
 * @brief Check if the provided value length matches characteristic requirements
 * @param char_code Characteristic Code
 * @param val_len   Length of the Characteristic value
 ****************************************************************************************
 */
uint8_t jwaoo_toy_check_val_len(uint8_t char_code, uint8_t val_len);

/**
 ****************************************************************************************
 * @brief Disable actions grouped in getting back to IDLE and sending configuration to requester task
 ****************************************************************************************
 */
void jwaoo_toy_enable(uint16_t conhdl);
void jwaoo_toy_disable(uint16_t conhdl); 
uint8_t jwaoo_toy_sensor_poll(void);
uint8_t jwaoo_toy_report_key_value(uint8_t code, uint8_t value);
uint8_t jwaoo_toy_report_key_click(uint8_t code, uint8_t count);
uint8_t jwaoo_toy_report_key_long_click(uint8_t code);
bool jwaoo_toy_read_device_data(void);
uint8_t jwaoo_toy_write_data(uint16_t attr, const void *data, int size);
uint8_t jwaoo_toy_send_notify(uint16_t attr, const void *data, int size);

uint8_t jwaoo_toy_send_command_u8(uint8_t type, uint8_t value);
uint8_t jwaoo_toy_send_command_u16(uint8_t type, uint16_t value);
uint8_t jwaoo_toy_send_command_u32(uint8_t type, uint32_t value);
uint8_t jwaoo_toy_send_command_data(uint8_t type, const uint8_t *data, int size);
uint8_t jwaoo_toy_send_command_text(uint8_t type, const char *fmt, ...);

uint8_t jwaoo_toy_send_response_u8_typed(uint8_t type, uint8_t command, uint8_t value);
uint8_t jwaoo_toy_send_response_u8(uint8_t command, uint8_t value);
uint8_t jwaoo_toy_send_response_bool(uint8_t command, bool value);
uint8_t jwaoo_toy_send_response_u16(uint8_t command, uint16_t value);
uint8_t jwaoo_toy_send_response_u32(uint8_t command, uint32_t value);
uint8_t jwaoo_toy_send_response_data(uint8_t command, const uint8_t *data, uint16_t size);
uint8_t jwaoo_toy_send_response_text(uint8_t command, const char *fmt, ...);

void jwaoo_toy_process_command(const struct jwaoo_toy_command *command, uint16_t length);
bool jwaoo_toy_process_flash_data(const uint8_t *data, uint16_t length);
void jwaoo_toy_process_key(uint8_t code, uint8_t value);

static inline uint16_t jwaoo_toy_build_value16(const uint8_t *data)
{
	return data[0] | ((uint16_t) data[1]) << 8;
}

static inline uint32_t jwaoo_toy_build_value32(const uint8_t *data)
{
	return jwaoo_toy_build_value16(data) | ((uint32_t) jwaoo_toy_build_value16(data + 2)) << 16;
}

static inline uint8_t jwaoo_toy_send_command(const void *command, int size)
{
	return jwaoo_toy_write_data(JWAOO_TOY_ATTR_COMMAND_DATA, command, size);
}

static inline uint8_t jwaoo_toy_send_event(const uint8_t *event, int size)
{
	return jwaoo_toy_send_notify(JWAOO_TOY_ATTR_EVENT_DATA, event, size);
}

static inline uint8_t jwaoo_toy_send_empty_event(const uint8_t type)
{
	return jwaoo_toy_send_event(&type, 1);
}

static inline uint16_t jwaoo_toy_notify_busy(uint8_t attr)
{
	return jwaoo_toy_env.notify_busy_mask & (1 << attr);
}

static inline uint16_t jwaoo_toy_sensor_notify_busy(void)
{
	return jwaoo_toy_env.notify_busy_mask & (1 << JWAOO_TOY_ATTR_SENSOR_DATA);
}

#endif //BLE_JWAOO_TOY_SERVER

/// @} JWAOO_TOY

#endif // JWAOO_TOY_H_
