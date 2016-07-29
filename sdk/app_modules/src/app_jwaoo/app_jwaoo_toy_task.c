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

#include "pwm.h"
#include "co_math.h"			// Common Maths Definition
#include "jwaoo_toy_task.h"          // Device Information Service Server Task API
#include "jwaoo_toy.h"               // Device Information Service Definitions
#include "app_jwaoo_toy.h"            // Device Information Service Application Definitions
#include "app_jwaoo_toy_task.h"       // Device Information Service Application Task API
#include "app_task.h"           // Application Task API
#include "user_periph_setup.h"


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#include "app_entry_point.h"
#include "uart.h"

static struct jwaoo_toy_key *jwaoo_toy_key_get(ke_msg_id_t const msgid, ke_msg_id_t const first)
{
	int index = (msgid - first) / 3;

	return jwaoo_toy_keys + index;
}

static int jwaoo_toy_key_repeat_timer(ke_msg_id_t const msgid,
                                         uint8_t *code,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id) {
	struct jwaoo_toy_key *key = jwaoo_toy_key_get(msgid, JWAOO_TOY_KEY1_REPEAT_TIMER);

	if (key->value > 0) {
		ke_timer_set(key->repeat_timer, TASK_APP, JWAOO_TOY_KEY_REPEAT_SHORT);

		key->repeat++;
		jwaoo_toy_process_key_repeat(key);
	}

	return KE_MSG_CONSUMED;
}

static int jwaoo_toy_key_long_click_timer(ke_msg_id_t const msgid,
                                         uint8_t *code,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id) {
	struct jwaoo_toy_key *key = jwaoo_toy_key_get(msgid, JWAOO_TOY_KEY1_LONG_CLICK_TIMER);

	jwaoo_toy_process_key_long_click(key);

	return KE_MSG_CONSUMED;
}

static int jwaoo_toy_key_multi_click_timer(ke_msg_id_t const msgid,
                                         uint8_t *code,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id) {
	struct jwaoo_toy_key *key = jwaoo_toy_key_get(msgid, JWAOO_TOY_KEY1_MULTI_CLICK_TIMER);

	jwaoo_toy_process_key_multi_click(key);

	return KE_MSG_CONSUMED;
}

static int jwaoo_toy_reboot_handler(ke_msg_id_t const msgid,
                                         void *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
	platform_reset(RESET_AFTER_SPOTA_UPDATE);

    return (KE_MSG_CONSUMED);
}

static int jwaoo_toy_key_lock_handler(ke_msg_id_t const msgid,
                                         void *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
	if (jwaoo_toy_env.key_locked) {
		LED1_OPEN;
		jwaoo_toy_env.key_locked = false;
	} else {
		LED1_CLOSE;
		jwaoo_toy_env.key_locked = true;
	}

	println("key_locked = %d", jwaoo_toy_env.key_locked);

    return (KE_MSG_CONSUMED);
}

static int jwaoo_toy_led1_blink_handler(ke_msg_id_t const msgid,
                                         void *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
	LED1_BLINK_WALK;

    return (KE_MSG_CONSUMED);
}

static int jwaoo_toy_led2_blink_handler(ke_msg_id_t const msgid,
                                         void *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
	LED2_BLINK_WALK;

    return (KE_MSG_CONSUMED);
}

static int jwaoo_toy_moto_boost_handler(ke_msg_id_t const msgid,
                                         void *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
	if (jwaoo_toy_env.moto_level > 0) {
		MOTO_SET_LEVEL(jwaoo_toy_env.moto_level);
	}

    return (KE_MSG_CONSUMED);
}

static int jwaoo_toy_moto_blink_handler(ke_msg_id_t const msgid,
                                         void *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
	if (jwaoo_toy_env.moto_mode == 6) {
		MOTO_SET_LEVEL((rand() % PWM_LEVEL_MAX) + 1);
		ke_timer_set(msgid, dest_id, (rand() & 0x3F) + 1);
	} else {
		MOTO_BLINK_WALK;
	}

    return (KE_MSG_CONSUMED);
}


static const struct ke_msg_handler app_jwaoo_toy_process_handlers[]=
{
    { JWAOO_TOY_CREATE_DB_CFM,				(ke_msg_func_t) jwaoo_toy_create_db_cfm_handler },
    { JWAOO_TOY_DISABLE_IND,				(ke_msg_func_t) jwaoo_toy_disable_ind_handler }, 

	{ JWAOO_TOY_REBOOT, 					(ke_msg_func_t) jwaoo_toy_reboot_handler },
	{ JWAOO_TOY_KEY_LOCK, 					(ke_msg_func_t) jwaoo_toy_key_lock_handler },
	{ JWAOO_TOY_LED1_BLINK, 				(ke_msg_func_t) jwaoo_toy_led1_blink_handler },
	{ JWAOO_TOY_LED2_BLINK, 				(ke_msg_func_t) jwaoo_toy_led2_blink_handler },

	{ JWAOO_TOY_KEY1_REPEAT_TIMER,			(ke_msg_func_t) jwaoo_toy_key_repeat_timer },
	{ JWAOO_TOY_KEY2_REPEAT_TIMER,			(ke_msg_func_t) jwaoo_toy_key_repeat_timer },
	{ JWAOO_TOY_KEY3_REPEAT_TIMER,			(ke_msg_func_t) jwaoo_toy_key_repeat_timer },
	{ JWAOO_TOY_KEY4_REPEAT_TIMER,			(ke_msg_func_t) jwaoo_toy_key_repeat_timer },

	{ JWAOO_TOY_KEY1_LONG_CLICK_TIMER,		(ke_msg_func_t) jwaoo_toy_key_long_click_timer },
	{ JWAOO_TOY_KEY2_LONG_CLICK_TIMER,		(ke_msg_func_t) jwaoo_toy_key_long_click_timer },
	{ JWAOO_TOY_KEY3_LONG_CLICK_TIMER,		(ke_msg_func_t) jwaoo_toy_key_long_click_timer },
	{ JWAOO_TOY_KEY4_LONG_CLICK_TIMER,		(ke_msg_func_t) jwaoo_toy_key_long_click_timer },

	{ JWAOO_TOY_KEY1_MULTI_CLICK_TIMER,		(ke_msg_func_t) jwaoo_toy_key_multi_click_timer },
	{ JWAOO_TOY_KEY2_MULTI_CLICK_TIMER, 	(ke_msg_func_t) jwaoo_toy_key_multi_click_timer },
	{ JWAOO_TOY_KEY3_MULTI_CLICK_TIMER, 	(ke_msg_func_t) jwaoo_toy_key_multi_click_timer },
	{ JWAOO_TOY_KEY4_MULTI_CLICK_TIMER, 	(ke_msg_func_t) jwaoo_toy_key_multi_click_timer },

	{ JWAOO_TOY_MOTO_BOOST, 				(ke_msg_func_t) jwaoo_toy_moto_boost_handler },
	{ JWAOO_TOY_MOTO_BLINK, 				(ke_msg_func_t) jwaoo_toy_moto_blink_handler },
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
