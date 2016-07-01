/**
 ****************************************************************************************
 *
 * @file app_jwaoo_toy.c
 *
 * @brief Device Information Service Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_JWAOO_TOY_SERVER)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_jwaoo_toy.h"                 // Device Information Service Application Definitions
#include "app_jwaoo_toy_task.h"            // Device Information Service Application Task API
#include "app.h"                     // Application Definitions
#include "app_task.h"                // Application Task Definitions
#include "jwaoo_toy_task.h"               // Health Thermometer Functions
#include "app_prf_perm_types.h"

/*
 * LOCAL VARIABLES DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_jwaoo_toy_init(void)
{
    return;
}

void app_jwaoo_toy_create_db(void)
{
    // Add JWAOO_TOY in the database
    struct jwaoo_toy_create_db_req *req = KE_MSG_ALLOC(JWAOO_TOY_CREATE_DB_REQ,
                                                  TASK_JWAOO_TOY, TASK_APP,
                                                  jwaoo_toy_create_db_req);

    req->features = 0;

    // Send the message
    ke_msg_send(req);
}

void app_jwaoo_toy_enable(uint16_t conhdl)
{
    // Allocate the message
    struct jwaoo_toy_enable_req *req = KE_MSG_ALLOC(JWAOO_TOY_ENABLE_REQ,
                                               TASK_JWAOO_TOY, TASK_APP,
                                               jwaoo_toy_enable_req);

    // Fill in the parameter structure
    req->conhdl             = conhdl;
    req->sec_lvl            = get_user_prf_srv_perm(TASK_JWAOO_TOY);
    req->con_type           = PRF_CON_DISCOVERY;

    // Send the message
    ke_msg_send(req);
}
#endif // (BLE_JWAOO_TOY_SERVER)

/// @} APP
