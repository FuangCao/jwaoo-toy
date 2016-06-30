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
#include "attm_db_128.h"

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * JWAOO_TOY ATTRIBUTES
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

struct jwaoo_toy_env_tag jwaoo_toy_env __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY

static const struct ke_task_desc TASK_DESC_JWAOO_TOY = {
	.state_handler = jwaoo_toy_state_handler,
	.default_handler = &jwaoo_toy_default_handler,
	.state = jwaoo_toy_state,
	.state_max = JWAOO_TOY_STATE_COUNT,
	.idx_max = JWAOO_TOY_TASK_COUNT
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
