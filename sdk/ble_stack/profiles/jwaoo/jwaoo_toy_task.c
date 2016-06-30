/**
 ****************************************************************************************
 *
 * @file jwaoo_toy_task.c
 *
 * @brief Device Information Service Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup JWAOO_TOY_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_JWAOO_TOY_SERVER)
#include "gap.h"
#include "uart.h"
#include "gattc_task.h"
#include "jwaoo_toy.h"
#include "jwaoo_toy_task.h"
#include "prf_utils.h"
#include "attm_util.h"
#include "atts_util.h"

static uint16_t jwaoo_toy_svc = JWAOO_TOY_UUID_SVC;
static struct att_char_desc jwaoo_toy_tx_char = ATT_CHAR(ATT_CHAR_PROP_NTF, JWAOO_TOY_CHAR_TX, JWAOO_TOY_UUID_TX);
static struct att_char_desc jwaoo_toy_rx_char = ATT_CHAR(ATT_CHAR_PROP_WR, JWAOO_TOY_CHAR_RX, JWAOO_TOY_UUID_RX);
static struct att_char_desc jwaoo_toy_ota_char = ATT_CHAR(ATT_CHAR_PROP_WR, JWAOO_TOY_CHAR_OTA, JWAOO_TOY_UUID_OTA);

const struct attm_desc jwaoo_toy_att_db[JWAOO_TOY_ATTR_COUNT] =
{
	[JWAOO_TOY_ATTR_SVC] = {
		.uuid = ATT_DECL_PRIMARY_SERVICE,
		.perm = PERM(RD, ENABLE),
		.max_length = sizeof(jwaoo_toy_svc),
		.length = sizeof(jwaoo_toy_svc),
		.value = (uint8_t *) &jwaoo_toy_svc
	},
	[JWAOO_TOY_ATTR_TX_CHAR] = {
		.uuid = ATT_DECL_CHARACTERISTIC,
		.perm = PERM(RD, ENABLE),
		.max_length = sizeof(jwaoo_toy_tx_char),
		.length = sizeof(jwaoo_toy_tx_char),
		.value = (uint8_t *)& jwaoo_toy_tx_char
	},
	[JWAOO_TOY_ATTR_TX_DATA] = {
		.uuid = JWAOO_TOY_UUID_TX,
		.perm = PERM(NTF, ENABLE),
		.max_length = JWAOO_TOY_MAX_DATA_SIZE,
		.length = 0,
		.value = NULL
	},
	[JWAOO_TOY_ATTR_RX_CHAR] = {
		.uuid = ATT_DECL_CHARACTERISTIC,
		.perm = PERM(RD, ENABLE),
		.max_length = sizeof(jwaoo_toy_rx_char),
		.length = sizeof(jwaoo_toy_rx_char),
		.value = (uint8_t *)& jwaoo_toy_rx_char
	},
	[JWAOO_TOY_ATTR_RX_DATA] = {
		.uuid = JWAOO_TOY_UUID_RX,
		.perm = PERM(WR, ENABLE),
		.max_length = JWAOO_TOY_MAX_DATA_SIZE,
		.length = 0,
		.value = NULL
	},
	[JWAOO_TOY_ATTR_OTA_CHAR] = {
		.uuid = ATT_DECL_CHARACTERISTIC,
		.perm = PERM(RD, ENABLE),
		.max_length = sizeof(jwaoo_toy_ota_char),
		.length = sizeof(jwaoo_toy_ota_char),
		.value = (uint8_t *)& jwaoo_toy_ota_char
	},
	[JWAOO_TOY_ATTR_OTA_DATA] = {
		.uuid = JWAOO_TOY_UUID_OTA,
		.perm = PERM(WR, ENABLE),
		.max_length = JWAOO_TOY_MAX_DATA_SIZE,
		.length = 0,
		.value = NULL
	},
};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static int jwaoo_toy_create_db_req_handler(ke_msg_id_t const msgid,
                                      struct jwaoo_toy_create_db_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    struct jwaoo_toy_create_db_cfm *cfm;
    //DB Creation Statis
    uint8_t status = ATT_ERR_NO_ERROR;
	//Service content flag
    uint32_t cfg_flag = (1 << JWAOO_TOY_ATTR_COUNT) - 1;

    //Save profile id
    jwaoo_toy_env.con_info.prf_id = TASK_JWAOO_TOY;

	status = attm_svc_create_db(&jwaoo_toy_env.shdl, (uint8_t *) &cfg_flag, JWAOO_TOY_ATTR_COUNT, jwaoo_toy_env.att_tbl, dest_id, jwaoo_toy_att_db);
	if (status == ATT_ERR_NO_ERROR)
    {
        //Disable service
        status = attmdb_svc_set_permission(jwaoo_toy_env.shdl, PERM(SVC, DISABLE));

        //If we are here, database has been fulfilled with success, go to idle test
        ke_state_set(TASK_JWAOO_TOY, JWAOO_TOY_IDLE);
    }

    //Send response to application
    cfm = KE_MSG_ALLOC(JWAOO_TOY_CREATE_DB_CFM, src_id, TASK_JWAOO_TOY, jwaoo_toy_create_db_cfm);
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref JWAOO_TOY_SET_CHAR_VAL_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int jwaoo_toy_set_char_val_req_handler(ke_msg_id_t const msgid,
                                         struct jwaoo_toy_set_char_val_req const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref JWAOO_TOY_ENABLE_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int jwaoo_toy_enable_req_handler(ke_msg_id_t const msgid,
                                   struct jwaoo_toy_enable_req const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    //Save the connection handle associated to the profile
    jwaoo_toy_env.con_info.conidx = gapc_get_conidx(param->conhdl);
    //Save the application id
    jwaoo_toy_env.con_info.appid = src_id;

    // Check if the provided connection exist
    if (jwaoo_toy_env.con_info.conidx == GAP_INVALID_CONIDX)
    {
        // The connection doesn't exist, request disallowed
        prf_server_error_ind_send((prf_env_struct *) &jwaoo_toy_env, PRF_ERR_REQ_DISALLOWED,
                                  JWAOO_TOY_ERROR_IND, JWAOO_TOY_ENABLE_REQ);
    }
    else
    {
        //Enable Attributes + Set Security Level
        attmdb_svc_set_permission(jwaoo_toy_env.shdl, param->sec_lvl);

        // Go to connected state
        ke_state_set(TASK_JWAOO_TOY, JWAOO_TOY_CONNECTED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Disconnection indication to HTPT.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                      struct gapc_disconnect_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    if (KE_IDX_GET(src_id) == jwaoo_toy_env.con_info.conidx)
    {
        jwaoo_toy_disable(param->conhdl);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref gattc_cmp_evt message.
 * The handler enables the Serial Port Service Device profile.
 * @param[in] msgid Id of the message received .
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance 
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gattc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	pr_pos_info();

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_WRITE_CMD_IND message.
 * Receive and proces incoming data 
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_write_cmd_ind_handler(ke_msg_id_t const msgid,
                                      struct gattc_write_cmd_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
	pr_pos_info();
	println("length = %d", param->length);
	uart2_nputs((const char *) param->value, param->length);
	uart2_nputs("\r\n", 2);

	if (param->response)
	{
		atts_write_rsp_send(jwaoo_toy_env.con_info.conidx, param->handle, PRF_ERR_OK);
	}

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///Disabled State handler definition.
const struct ke_msg_handler jwaoo_toy_disabled[] =
{
	{ JWAOO_TOY_CREATE_DB_REQ,		(ke_msg_func_t) jwaoo_toy_create_db_req_handler },
};

///Idle State handler definition.
const struct ke_msg_handler jwaoo_toy_idle[] =
{
    { JWAOO_TOY_SET_CHAR_VAL_REQ,	(ke_msg_func_t) jwaoo_toy_set_char_val_req_handler },
    { JWAOO_TOY_ENABLE_REQ,			(ke_msg_func_t) jwaoo_toy_enable_req_handler },
};

const struct ke_msg_handler jwaoo_toy_connected[] =
{
    {GAPC_DISCONNECT_IND,			(ke_msg_func_t) gapc_disconnect_ind_handler},
    {GATTC_CMP_EVT,					(ke_msg_func_t) gattc_cmp_evt_handler},
    {GATTC_WRITE_CMD_IND,			(ke_msg_func_t) gattc_write_cmd_ind_handler},
};

///Specifies the message handler structure for every input state.
const struct ke_state_handler jwaoo_toy_state_handler[JWAOO_TOY_STATE_COUNT] =
{
    [JWAOO_TOY_DISABLED]			= KE_STATE_HANDLER(jwaoo_toy_disabled),
    [JWAOO_TOY_IDLE]				= KE_STATE_HANDLER(jwaoo_toy_idle),
    [JWAOO_TOY_CONNECTED]			= KE_STATE_HANDLER(jwaoo_toy_connected),
};

///Specifies the message handlers that are common to all states.
const struct ke_state_handler jwaoo_toy_default_handler = KE_STATE_HANDLER_NONE;

///Defines the place holder for the states of all the task instances.
ke_state_t jwaoo_toy_state[JWAOO_TOY_TASK_COUNT] __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY

#endif //BLE_JWAOO_TOY_SERVER

/// @} JWAOO_TOY_TASK
