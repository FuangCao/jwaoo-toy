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

#define jwaoo_toy_send_response_bool(value) \
	jwaoo_toy_send_command_u8(JWAOO_TOY_RSP_BOOL, value);

#define jwaoo_toy_send_response_u8(value) \
	jwaoo_toy_send_command_u8(JWAOO_TOY_RSP_U8, value);

#define jwaoo_toy_send_response_u16(value) \
	jwaoo_toy_send_command_u16(JWAOO_TOY_RSP_U16, value);

#define jwaoo_toy_send_response_u32(value) \
	jwaoo_toy_send_command_u32(JWAOO_TOY_RSP_U32, value);

#define jwaoo_toy_send_response_data(data, length) \
	jwaoo_toy_send_command_data(JWAOO_TOY_RSP_DATA, data, length);

#define jwaoo_toy_send_response_text(fmt, args ...) \
	jwaoo_toy_send_command_text(JWAOO_TOY_RSP_TEXT, fmt, ##args);

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

enum
{
	JWAOO_TOY_ATTR_SVC,

	JWAOO_TOY_ATTR_COMMAND_CHAR,
	JWAOO_TOY_ATTR_COMMAND_DATA,
	JWAOO_TOY_ATTR_COMMAND_CFG,

	JWAOO_TOY_ATTR_EVENT_CHAR,
	JWAOO_TOY_ATTR_EVENT_DATA,
	JWAOO_TOY_ATTR_EVENT_CFG,

	JWAOO_TOY_ATTR_FLASH_CHAR,
	JWAOO_TOY_ATTR_FLASH_DATA,
	JWAOO_TOY_ATTR_FLASH_CFG,

	JWAOO_TOY_ATTR_SENSOR_CHAR,
	JWAOO_TOY_ATTR_SENSOR_DATA,
	JWAOO_TOY_ATTR_SENSOR_CFG,

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
	JWAOO_TOY_CMD_FIND,
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
	JWAOO_TOY_CMD_SENSOR_SET_DELAY,
	JWAOO_TOY_CMD_MOTO_ENABLE = 80,
	JWAOO_TOY_CMD_MOTO_SET_LEVEL,
};

enum
{
	JWAOO_TOY_EVT_NOOP,
	JWAOO_TOY_EVT_BATT_INFO,
	JWAOO_TOY_EVT_KEY_STATE,
	JWAOO_TOY_EVT_KEY_CLICK,
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
	uint8_t bytes[];
};

struct jwaoo_toy_command_u8
{
	uint8_t type;
	uint8_t value;
};

struct jwaoo_toy_command_u16
{
	uint8_t type;
	uint16_t value;
};

struct jwaoo_toy_command_u32
{
	uint8_t type;
	uint32_t value;
};

struct jwaoo_toy_command_flash_write_finish
{
	uint8_t type;
	uint8_t crc;
	uint16_t length;
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
	bool notify_busy;

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
bool jwaoo_toy_read_device_data(void);
uint8_t jwaoo_toy_write_data(uint16_t attr, const uint8_t *data, int size);
uint8_t jwaoo_toy_send_notify(uint16_t attr, const uint8_t *data, int size);
uint8_t jwaoo_toy_send_command_u8(uint8_t type, uint8_t value);
uint8_t jwaoo_toy_send_command_u16(uint8_t type, uint16_t value);
uint8_t jwaoo_toy_send_command_u32(uint8_t type, uint32_t value);
uint8_t jwaoo_toy_send_command_data(uint8_t type, const uint8_t *data, int size);
uint8_t jwaoo_toy_send_command_text(uint8_t type, const char *fmt, ...);

void jwaoo_toy_process_command(const struct jwaoo_toy_command *command, uint16_t length);
bool jwaoo_toy_process_flash_data(const uint8_t *data, uint16_t length);

static inline uint8_t jwaoo_toy_send_command(const uint8_t *command, int size)
{
	return jwaoo_toy_write_data(JWAOO_TOY_ATTR_COMMAND_DATA, command, size);
}

static inline uint8_t jwaoo_toy_send_event(const uint8_t *event, int size)
{
	return jwaoo_toy_send_notify(JWAOO_TOY_ATTR_EVENT_DATA, event, size);
}

static inline uint8_t jwaoo_toy_report_key(uint8_t keycode)
{
	uint8_t event[] = { JWAOO_TOY_EVT_KEY_CLICK, keycode };

	return jwaoo_toy_send_event(event, sizeof(event));
}

#endif //BLE_JWAOO_TOY_SERVER

/// @} JWAOO_TOY

#endif // JWAOO_TOY_H_
