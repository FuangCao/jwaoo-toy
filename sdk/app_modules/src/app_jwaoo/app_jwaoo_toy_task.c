/**
 ****************************************************************************************
 *
 * @file app_jwaoo_toy_task.c
 *
 * @brief Device Information Service Application Task
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

#include "rwip_config.h"        // SW Configuration
#include <string.h>             // srtlen()

#if (BLE_JWAOO_TOY_SERVER)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "jwaoo_toy_task.h"          // Device Information Service Server Task API
#include "jwaoo_toy.h"               // Device Information Service Definitions
#include "app_jwaoo_toy.h"            // Device Information Service Application Definitions
#include "app_jwaoo_toy_task.h"       // Device Information Service Application Task API
#include "app_task.h"           // Application Task API



/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#include "app_entry_point.h"

static const struct ke_msg_handler app_jwaoo_toy_process_handlers[]=
{
    { JWAOO_TOY_CREATE_DB_CFM,		(ke_msg_func_t)jwaoo_toy_create_db_cfm_handler },
    { JWAOO_TOY_DISABLE_IND,		(ke_msg_func_t)jwaoo_toy_disable_ind_handler }, 
};

enum process_event_response app_jwaoo_toy_process_handler (ke_msg_id_t const msgid,
								 void const *param,
								 ke_task_id_t const dest_id,
								 ke_task_id_t const src_id, 
								 enum ke_msg_status_tag *msg_ret)
{
    return (app_std_process_event(msgid, param,src_id,dest_id,msg_ret, app_jwaoo_toy_process_handlers,
                                         sizeof(app_jwaoo_toy_process_handlers)/sizeof(struct ke_msg_handler)));
} 



/**
 ****************************************************************************************
 * @brief Handles JWAOO_TOY Server profile database creation confirmation.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int jwaoo_toy_create_db_cfm_handler(ke_msg_id_t const msgid,
                                      struct jwaoo_toy_create_db_cfm const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    if (ke_state_get(dest_id) == APP_DB_INIT)
    {
        // Inform the Application Manager
        struct app_module_init_cmp_evt *cfm = KE_MSG_ALLOC(APP_MODULE_INIT_CMP_EVT,
                                                           TASK_APP, TASK_APP,
                                                           app_module_init_cmp_evt);

        cfm->status = param->status;

        ke_msg_send(cfm);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles disable indication from the JWAOO_TOY Server profile.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int jwaoo_toy_disable_ind_handler(ke_msg_id_t const msgid,
                                    struct jwaoo_toy_disable_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

#endif //(BLE_JWAOO_TOY_SERVER)

/// @} APP
