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

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

enum
{
	JWAOO_TOY_ATTR_SVC,

	JWAOO_TOY_ATTR_COMMAND_CHAR,
	JWAOO_TOY_ATTR_COMMAND_DATA,

	JWAOO_TOY_ATTR_FLASH_CHAR,
	JWAOO_TOY_ATTR_FLASH_DATA,

	JWAOO_TOY_ATTR_SENSOR_CHAR,
	JWAOO_TOY_ATTR_SENSOR_DATA,

	JWAOO_TOY_ATTR_COUNT,
};

enum
{
	JWAOO_TOY_CMD_NONE,
	JWAOO_TOY_RSP_BOOL,
	JWAOO_TOY_RSP_U8,
	JWAOO_TOY_RSP_U16,
	JWAOO_TOY_RSP_U32,
	JWAOO_TOY_RSP_DATA,
	JWAOO_TOY_CMD_FLASH_WEN,
	JWAOO_TOY_CMD_FLASH_READ,
	JWAOO_TOY_CMD_FLASH_SEEK,
	JWAOO_TOY_CMD_FLASH_ERASE,
	JWAOO_TOY_CMD_FLASH_WRITE_OK,
	JWAOO_TOY_CMD_SENSOR_ENABLE,
	JWAOO_TOY_CMD_SENSOR_SET_DELAY,
	JWAOO_TOY_CMD_COUNT
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

#pragma pack()

///Attributes State Machine
///Device Information Service Server Environment Variable
struct jwaoo_toy_env_tag
{
    /// Connection Info
    struct prf_con_info con_info;

    /// Service Start HandleVAL
    uint16_t handle;

	bool flash_write_ok;
	bool flash_write_enable;
	uint32_t flash_write_address;
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern struct jwaoo_toy_env_tag jwaoo_toy_env;

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
void jwaoo_toy_disable(uint16_t conhdl); 
uint8_t jwaoo_toy_write_data(uint16_t attr, const uint8_t *data, int size);
uint8_t jwaoo_toy_send_notify(uint16_t attr, const uint8_t *data, int size);
uint8_t jwaoo_toy_send_command_u8(uint8_t type, uint8_t value);
uint8_t jwaoo_toy_send_command_u16(uint8_t type, uint16_t value);
uint8_t jwaoo_toy_send_command_u32(uint8_t type, uint32_t value);
uint8_t jwaoo_toy_send_command_data(uint8_t type, const uint8_t *data, int size);

void jwaoo_toy_process_command(const struct jwaoo_toy_command *command);
void jwaoo_toy_process_flash_data(const uint8_t *data, int length);

static inline uint8_t jwaoo_toy_send_command(const uint8_t *command, int size)
{
	return jwaoo_toy_write_data(JWAOO_TOY_ATTR_COMMAND_DATA, command, size);
}

static inline uint8_t jwaoo_toy_send_response_bool(bool value)
{
	return jwaoo_toy_send_command_u8(JWAOO_TOY_RSP_BOOL, value);
}

static inline uint8_t jwaoo_toy_send_response_u8(uint8_t value)
{
	return jwaoo_toy_send_command_u8(JWAOO_TOY_RSP_U8, value);
}

static inline uint16_t jwaoo_toy_send_response_u16(uint16_t value)
{
	return jwaoo_toy_send_command_u16(JWAOO_TOY_RSP_U16, value);
}

static inline uint32_t jwaoo_toy_send_response_u32(uint32_t value)
{
	return jwaoo_toy_send_command_u32(JWAOO_TOY_RSP_U32, value);
}

static inline uint32_t jwaoo_toy_send_response_data(const uint8_t *data, int length)
{
	return jwaoo_toy_send_command_data(JWAOO_TOY_RSP_DATA, data, length);
}

#endif //BLE_JWAOO_TOY_SERVER

/// @} JWAOO_TOY

#endif // JWAOO_TOY_H_
