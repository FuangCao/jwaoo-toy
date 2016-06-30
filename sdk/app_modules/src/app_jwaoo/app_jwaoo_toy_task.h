/**
 ****************************************************************************************
 *
 * @file app_jwaoo_toy_task.h
 *
 * @brief Header file - APPDISTASK.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

#ifndef APP_JWAOO_TOY_TASK_H_
#define APP_JWAOO_TOY_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APPDISTASK Task
 * @ingroup APPDIS
 * @brief Device Information Service Application Task
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_JWAOO_TOY_SERVER)

#include "jwaoo_toy_task.h"
#include "ke_task.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximal number of APP JWAOO_TOY Task instances
#define APP_JWAOO_TOY_IDX_MAX        (1)

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles JWAOO_TOY Server profile database creation confirmation.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int jwaoo_toy_create_db_cfm_handler(ke_msg_id_t const msgid,
                                      struct jwaoo_toy_create_db_cfm const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Handles disable indication from the JWAOO_TOY Server profile.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int jwaoo_toy_disable_ind_handler(ke_msg_id_t const msgid,
                                    struct jwaoo_toy_disable_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);                                      


#endif //(BLE_JWAOO_TOY_SERVER)

/// @} APPDISTASK

#endif //APP_JWAOO_TOY_TASK_H_
