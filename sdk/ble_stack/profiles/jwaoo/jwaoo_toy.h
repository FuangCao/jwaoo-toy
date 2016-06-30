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

	JWAOO_TOY_ATTR_TX_CHAR,
	JWAOO_TOY_ATTR_TX_DATA,

	JWAOO_TOY_ATTR_RX_CHAR,
	JWAOO_TOY_ATTR_RX_DATA,

	JWAOO_TOY_ATTR_OTA_CHAR,
	JWAOO_TOY_ATTR_OTA_DATA,

	JWAOO_TOY_ATTR_COUNT,
};

enum
{
	JWAOO_TOY_CHAR_TX,
	JWAOO_TOY_CHAR_RX,
	JWAOO_TOY_CHAR_OTA,
	JWAOO_TOY_CHAR_COUNT,
};

///Attributes State Machine
///Device Information Service Server Environment Variable
struct jwaoo_toy_env_tag
{
    /// Connection Info
    struct prf_con_info con_info;

    /// Service Start HandleVAL
    uint16_t shdl;
    /// Attribute Table
    uint8_t att_tbl[JWAOO_TOY_CHAR_COUNT];
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

#endif //BLE_JWAOO_TOY_SERVER

/// @} JWAOO_TOY

#endif // JWAOO_TOY_H_
