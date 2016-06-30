/**
 ****************************************************************************************
 *
 * @file jwaoo_toy.c
 *
 * @brief Device Information Service Server Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup JWAOO_TOY
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_JWAOO_TOY_SERVER)
#include "attm_util.h"
#include "atts_util.h"
#include "jwaoo_toy.h"
#include "jwaoo_toy_task.h"
#include "prf_utils.h"

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * JWAOO_TOY ATTRIBUTES
 ****************************************************************************************
 */

static const att_svc_desc_t jwaoo_toy_svc = JWAOO_TOY_UUID_SVC;
static const struct att_char_desc jwaoo_toy_rx_char = ATT_CHAR(ATT_CHAR_PROP_WR_NO_RESP, JWAOO_TOY_RX_CHAR, JWAOO_TOY_UUID_RX);
static const struct att_char_desc jwaoo_toy_tx_char = ATT_CHAR(ATT_CHAR_PROP_NTF, JWAOO_TOY_TX_CHAR, JWAOO_TOY_UUID_TX);
static const struct att_char_desc jwaoo_toy_ota_char = ATT_CHAR(ATT_CHAR_PROP_WR, JWAOO_TOY_OTA_CHAR, JWAOO_TOY_UUID_OTA);

/// Full JWAOO_TOY Database Description - Used to add attributes into the database
const struct attm_desc jwaoo_toy_att_db[JWAOO_TOY_IDX_NB] =
{
	[JWAOO_TOY_IDX_SVC] = {
		.uuid = ATT_DECL_PRIMARY_SERVICE,
		.perm = PERM(RD, ENABLE),
		.max_length = sizeof(jwaoo_toy_svc),
		.length = sizeof(jwaoo_toy_svc),
		.value = (uint8_t *) &jwaoo_toy_svc
	},
	[JWAOO_TOY_IDX_TX_CHAR] = {
		.uuid = ATT_DECL_CHARACTERISTIC,
		.perm = PERM(RD, ENABLE),
		.max_length = sizeof(jwaoo_toy_tx_char),
		.length = sizeof(jwaoo_toy_tx_char),
		.value = (uint8_t *)& jwaoo_toy_tx_char
	},
	[JWAOO_TOY_IDX_TX_VAL] = {
		.uuid = JWAOO_TOY_UUID_TX,
		.perm = PERM(NTF, ENABLE),
		.max_length = JWAOO_TOY_VAL_MAX_LEN,
		.length = 0,
		.value = NULL
	},
	[JWAOO_TOY_IDX_RX_CHAR] = {
		.uuid = ATT_DECL_CHARACTERISTIC,
		.perm = PERM(RD, ENABLE),
		.max_length = sizeof(jwaoo_toy_rx_char),
		.length = sizeof(jwaoo_toy_rx_char),
		.value = (uint8_t *)& jwaoo_toy_rx_char
	},
	[JWAOO_TOY_IDX_RX_VAL] = {
		.uuid = JWAOO_TOY_UUID_RX,
		.perm = PERM(WR, ENABLE),
		.max_length = JWAOO_TOY_VAL_MAX_LEN,
		.length = 0,
		.value = NULL
	},
	[JWAOO_TOY_IDX_OTA_CHAR] = {
		.uuid = ATT_DECL_CHARACTERISTIC,
		.perm = PERM(RD, ENABLE),
		.max_length = sizeof(jwaoo_toy_ota_char),
		.length = sizeof(jwaoo_toy_ota_char),
		.value = (uint8_t *)& jwaoo_toy_ota_char
	},
	[JWAOO_TOY_IDX_OTA_VAL] = {
		.uuid = JWAOO_TOY_UUID_OTA,
		.perm = PERM(WR, ENABLE),
		.max_length = JWAOO_TOY_VAL_MAX_LEN,
		.length = 0,
		.value = NULL
	},
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct jwaoo_toy_env_tag jwaoo_toy_env __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

static const struct ke_task_desc TASK_DESC_JWAOO_TOY = {
	.state_handler = jwaoo_toy_state_handler,
	.default_handler = &jwaoo_toy_default_handler,
	.state = jwaoo_toy_state,
	.state_max = JWAOO_TOY_STATE_MAX,
	.idx_max = JWAOO_TOY_IDX_MAX
};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void jwaoo_toy_init(void)
{
    // Reset environment
    memset(&jwaoo_toy_env, 0, sizeof(jwaoo_toy_env));

    // Create JWAOO_TOY task
    ke_task_create(TASK_JWAOO_TOY, &TASK_DESC_JWAOO_TOY);

    // Set task in disabled state
    ke_state_set(TASK_JWAOO_TOY, JWAOO_TOY_DISABLED);
}

void jwaoo_toy_disable(uint16_t conhdl) 
{
    // Inform the application about the disconnection
    struct jwaoo_toy_disable_ind *ind = KE_MSG_ALLOC(JWAOO_TOY_DISABLE_IND,
		jwaoo_toy_env.con_info.appid, TASK_JWAOO_TOY, jwaoo_toy_disable_ind);

    ind->conhdl = conhdl;

    ke_msg_send(ind);

    //Disable JWAOO_TOY in database
    attmdb_svc_set_permission(jwaoo_toy_env.shdl, PERM(SVC, DISABLE));

    //Go to idle state
    ke_state_set(TASK_JWAOO_TOY, JWAOO_TOY_IDLE);
}

#endif //BLE_JWAOO_TOY_SERVER

/// @} JWAOO_TOY
