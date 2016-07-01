/**
 ****************************************************************************************
 *
 * @file app_jwaoo_toy.h
 *
 * @brief Device Information Service Application entry point.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

#ifndef _APP_JWAOO_TOY_H_
#define _APP_JWAOO_TOY_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 *
 * @brief Device Information Service Application entry point.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW Configuration

#if (BLE_JWAOO_TOY_SERVER)

#include <stdint.h>          // Standard Integer Definition
#include <co_bt.h>
#include "ble_580_sw_version.h"
#include "user_config_sw_ver.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 *
 * Device Information Service Application Functions
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Device Information Service Application.
 * @return void
 ****************************************************************************************
 */
void app_jwaoo_toy_init(void);

/**
 ****************************************************************************************
 * @brief Add a Device Information Service instance in the DB.
 * @return void
 ****************************************************************************************
 */
void app_jwaoo_toy_create_db(void);

/**
 ****************************************************************************************
 * @brief Enable the Device Information Service.
 * @param[in] conhdl Connection handle
 * @return void
 ****************************************************************************************
 */
void app_jwaoo_toy_enable(uint16_t conhdl);
#endif // (BLE_JWAOO_TOY_SERVER)

/// @} APP

#endif // _APP_JWAOO_TOY_H_
