/**
 ****************************************************************************************
 *
 * @file jwaoo_toy_task.h
 *
 * @brief Header file - JWAOO_TOY_TASK.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

#ifndef JWAOO_TOY_TASK_H_
#define JWAOO_TOY_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup JWAOO_TOY_TASK Task
 * @ingroup JWAOO_TOY
 * @brief Device Information Service Server Task
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if (BLE_JWAOO_TOY_SERVER)
#include <stdint.h>
#include "ke_task.h"
#include "jwaoo_toy.h"
#include "prf_types.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define JWAOO_TOY_TASK_COUNT			1
#define JWAOO_TOY_MAX_COMMAND_SIZE		32
#define JWAOO_TOY_MAX_EVENT_SIZE		20
#define JWAOO_TOY_MAX_FLASH_DATA_SIZE	64
#define JWAOO_TOY_MAX_SENSOR_DATA_SIZE	20

enum
{
	JWAOO_TOY_UUID_SVC = 0x1888,
	JWAOO_TOY_UUID_COMMAND,
	JWAOO_TOY_UUID_EVENT,
	JWAOO_TOY_UUID_FLASH,
	JWAOO_TOY_UUID_SENSOR,
	JWAOO_TOY_UUID_MAX
};

/// Possible states of the JWAOO_TOY task
enum
{
    ///Disabled state
    JWAOO_TOY_DISABLED,
    ///Idle state
    JWAOO_TOY_IDLE,
    ///Connected state
    JWAOO_TOY_CONNECTED,

    ///Number of defined states.
    JWAOO_TOY_STATE_COUNT
};

/// Messages for Device Information Service Server
enum
{
    ///Add a JWAOO_TOY instance into the database
    JWAOO_TOY_CREATE_DB_REQ = KE_FIRST_MSG(TASK_JWAOO_TOY),
    ///Inform APP of database creation status
    JWAOO_TOY_CREATE_DB_CFM,
    ///Set the value of an attribute
    JWAOO_TOY_SENSOR_POLL,

    ///Start the Device Information Service Task - at connection
    JWAOO_TOY_ENABLE_REQ,

    /// Inform the application that the profile service role task has been disabled after a disconnection
    JWAOO_TOY_DISABLE_IND,

    ///Error indication to Host
    JWAOO_TOY_ERROR_IND,
};

/*
 * API MESSAGES STRUCTURES
 ****************************************************************************************
 */

/// Parameters of the @ref JWAOO_TOY_CREATE_DB_REQ message
struct jwaoo_toy_create_db_req
{
    ///Database configuration
    uint16_t features;
};

/// Parameters of the @ref JWAOO_TOY_CREATE_DB_CFM message
struct jwaoo_toy_create_db_cfm
{
    ///Status
    uint8_t status;
};

/// Parameters of the @ref JWAOO_TOY_SET_CHAR_VAL_REQ message - shall be dynamically allocated
struct jwaoo_toy_set_char_val_req
{
    /// Characteristic Code
    uint8_t char_code;
    /// Value length
    uint8_t val_len;
    /// Value
    uint8_t val[1];
};

/// Parameters of the @ref JWAOO_TOY_ENABLE_REQ message
struct jwaoo_toy_enable_req
{
    ///Connection handle
    uint16_t conhdl;
    /// security level: b0= nothing, b1=unauthenticated, b2=authenticated, b3=authorized; b1 or b2 and b3 can go together
    uint8_t sec_lvl;
    ///Type of connection
    uint8_t con_type;
};

/// Parameters of the @ref JWAOO_TOY_DISABLE_IND message
struct jwaoo_toy_disable_ind
{
    ///Connection handle
    uint16_t conhdl;
};

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler jwaoo_toy_state_handler[JWAOO_TOY_STATE_COUNT];
extern const struct ke_state_handler jwaoo_toy_default_handler;
extern ke_state_t jwaoo_toy_state[JWAOO_TOY_TASK_COUNT];

#endif //BLE_JWAOO_TOY_SERVER

/// @} JWAOO_TOY_TASK
#endif // JWAOO_TOY_TASK_H_
