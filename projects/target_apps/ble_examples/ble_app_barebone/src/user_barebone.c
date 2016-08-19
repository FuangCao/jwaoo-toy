/**
 ****************************************************************************************
 *
 * @file user_barebone.c
 *
 * @brief Barebone project source code.
 *
 * Copyright (C) 2015. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "jwaoo_toy.h"
#include "rwip_config.h"             // SW configuration
#include "user_barebone.h"
#include "arch_api.h"
#include "gap.h"
#include "user_periph_setup.h"       // peripheral configuration

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

// Manufacturer Specific Data ADV structure type
struct mnf_specific_data_ad_structure
{
    uint8_t ad_structure_size;
    uint8_t ad_structure_type;
    uint8_t company_id[APP_AD_MSD_COMPANY_ID_LEN];
    uint8_t proprietary_data[APP_AD_MSD_DATA_LEN];
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

uint16_t app_suspend_remain;
uint8_t app_connection_idx;
ke_msg_id_t app_suspend_timer_used;
ke_msg_id_t app_param_update_request_timer_used;

// Manufacturer Specific Data
struct mnf_specific_data_ad_structure mnf_data __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

static void jwaoo_app_set_blink_enable(bool enable, bool connected)
{
	if (connected) {
		jwaoo_led_open(&jwaoo_pwm_led2);
	} else if (enable) {
		jwaoo_led_blink_square(&jwaoo_pwm_led2, 1000, 0);
	} else {
		jwaoo_led_close(&jwaoo_pwm_led2);
	}
}

static void user_app_suspend_timer_cb()
{
	if (app_suspend_remain > 0) {
		app_suspend_timer_used = app_easy_timer(100, user_app_suspend_timer_cb);
		app_suspend_remain--;
	} else {
		app_suspend_timer_used = EASY_TIMER_INVALID_TIMER;
		user_app_set_suspend(true, false);
	}
}

void user_app_update_suspend_timer(void)
{
	app_suspend_remain = APP_SUSPEND_OVER_TIME_SECOND;

	if (app_suspend_timer_used == EASY_TIMER_INVALID_TIMER) {
		user_app_suspend_timer_cb();
	}
}

bool user_app_set_suspend(bool enable, bool force)
{
	if (app_suspended == enable) {
		return true;
	}

	if (jwaoo_toy_env.flash_upgrade || jwaoo_toy_env.factory_enable) {
		return false;
	}

	app_suspended = enable;

	if (enable) {
		if (jwaoo_pwm_moto.level > 0) {
			if (force) {
				jwaoo_moto_close();
			} else {
				app_suspended = false;
				return false;
			}
		}

		if (ke_state_get(TASK_APP) == APP_CONNECTED) {
			if (force) {
				app_easy_gap_disconnect(app_connection_idx);
			} else {
				app_suspended = false;
				return false;
			}
		}

		jwaoo_app_set_blink_enable(false, false);
		app_easy_gap_advertise_stop();

		if (app_suspend_timer_used != EASY_TIMER_INVALID_TIMER) {
			app_easy_timer_cancel(app_suspend_timer_used);
			app_suspend_timer_used = EASY_TIMER_INVALID_TIMER;
		}

		LDO_P3V3_CLOSE;
		arch_set_sleep_mode(ARCH_DEEP_SLEEP_ON);
	} else {
		arch_set_sleep_mode(ARCH_SLEEP_OFF);

		// jwaoo_toy_env.battery_voltage_head = 0;
		// jwaoo_toy_env.battery_voltage_count = 0;

		user_app_adv_start();
	}

	jwaoo_toy_battery_led_update_state();

	return true;
}

/**
 ****************************************************************************************
 * @brief Initialize Manufacturer Specific Data
 * @return void
 ****************************************************************************************
 */
static void mnf_data_init()
{
    mnf_data.ad_structure_size = sizeof(struct mnf_specific_data_ad_structure ) - sizeof(uint8_t); // minus the size of the ad_structure_size field
    mnf_data.ad_structure_type = GAP_AD_TYPE_MANU_SPECIFIC_DATA;
    mnf_data.company_id[0] = APP_AD_MSD_COMPANY_ID & 0xFF; // LSB
    mnf_data.company_id[1] = (APP_AD_MSD_COMPANY_ID >> 8 )& 0xFF; // MSB
    mnf_data.proprietary_data[0] = 0;
    mnf_data.proprietary_data[1] = 0;
}

/**
 ****************************************************************************************
 * @brief Update Manufacturer Specific Data
 * @return void
 ****************************************************************************************
 */
static void mnf_data_update()
{
    uint16_t data;

    data = mnf_data.proprietary_data[0] | (mnf_data.proprietary_data[1] << 8);
    data += 1;
    mnf_data.proprietary_data[0] = data & 0xFF;
    mnf_data.proprietary_data[1] = (data >> 8) & 0xFF;

    if (data == 0xFFFF) {
         mnf_data.proprietary_data[0] = 0;
         mnf_data.proprietary_data[1] = 0;
    }
}

/**
 ****************************************************************************************
 * @brief Parameter update request timer callback function.
 * @return void
 ****************************************************************************************
*/
static void param_update_request_timer_cb()
{
    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;

    app_easy_gap_param_update_start(app_connection_idx);
}

void user_app_init(void)
{
    // Initialize Manufacturer Specific Data
    mnf_data_init();

    default_app_on_init();
}

/**
 * @brief Add an AD structure in the Advertising or Scan Response Data of the GAPM_START_ADVERTISE_CMD parameter struct
 * @param[in] cmd               GAPM_START_ADVERTISE_CMD parameter struct.
 * @param[in] ad_struct_data    AD structure buffer.
 * @param[in] ad_struct_len     AD structure length.
 * @return void.
 */
static void app_add_ad_struct(struct gapm_start_advertise_cmd *cmd, void *ad_struct_data, uint8_t ad_struct_len)
{
    if ( (APP_ADV_DATA_MAX_SIZE - cmd->info.host.adv_data_len) >= ad_struct_len)
    {
        // Copy data
        memcpy(&cmd->info.host.adv_data[cmd->info.host.adv_data_len], ad_struct_data, ad_struct_len);

        // Update Advertising Data Length
        cmd->info.host.adv_data_len += ad_struct_len;
    }
    else if ( (APP_SCAN_RESP_DATA_MAX_SIZE - cmd->info.host.scan_rsp_data_len) >= ad_struct_len)
    {
        // Copy data
        memcpy(&cmd->info.host.scan_rsp_data[cmd->info.host.scan_rsp_data_len], ad_struct_data, ad_struct_len);

        // Update Scan Responce Data Length
        cmd->info.host.scan_rsp_data_len += ad_struct_len;
    }
    else
    {
        // Manufacturer Specific Data do not fit in either Advertising Data or Scan Response Data
        ASSERT_ERROR(0);
    }
}

void user_app_adv_start(void)
{
	if (app_suspended) {
		return;
	}

	if (!ke_timer_active(JWAOO_TOY_BATT_POLL, TASK_APP)) {
		ke_timer_set(JWAOO_TOY_BATT_POLL, TASK_APP, 1);
	}

	jwaoo_app_set_blink_enable(true, false);

    // Schedule the next advertising data update
    user_app_update_suspend_timer();

    struct gapm_start_advertise_cmd* cmd;
    cmd = app_easy_gap_undirected_advertise_get_active();

    // add manufacturer specific data dynamically
    mnf_data_update();
    app_add_ad_struct(cmd, &mnf_data, sizeof(struct mnf_specific_data_ad_structure));

    app_easy_gap_undirected_advertise_start();
}

void user_app_connection(uint8_t connection_idx, struct gapc_connection_req_ind const *param)
{
    if (app_env[connection_idx].conidx != GAP_INVALID_CONIDX)
    {
		jwaoo_app_set_blink_enable(false, true);

        app_connection_idx = connection_idx;

        // Stop the advertising data update timer
        app_easy_timer_cancel(app_suspend_timer_used);
		app_suspend_timer_used = EASY_TIMER_INVALID_TIMER;

        // Check if the parameters of the established connection are the preferred ones.
        // If not then schedule a connection parameter update request.
        if ((param->con_interval < user_connection_param_conf.intv_min) ||
            (param->con_interval > user_connection_param_conf.intv_max) ||
            (param->con_latency != user_connection_param_conf.latency) ||
            (param->sup_to != user_connection_param_conf.time_out))
        {
            // Connection params are not these that we expect
            app_param_update_request_timer_used = app_easy_timer(APP_PARAM_UPDATE_REQUEST_TO, param_update_request_timer_cb);
        }
    }
    else
    {
        // No connection has been established, restart advertising
        user_app_adv_start();
    }

    default_app_on_connection(connection_idx, param);
}

void user_app_adv_undirect_complete(uint8_t status)
{
    // If advertising was canceled then update advertising data and start advertising again
    if (status == GAP_ERR_CANCELED)
    {
        user_app_adv_start();
    }
}

void user_app_disconnect(struct gapc_disconnect_ind const *param)
{
     uint8_t state = ke_state_get(TASK_APP);

	 jwaoo_app_set_blink_enable(false, false);

    if ((state == APP_SECURITY) ||
        (state == APP_CONNECTED) ||
        (state == APP_PARAM_UPD))
    {
        // Restart Advertising
        user_app_adv_start();
    }
    else
    {
        // We are not in a Connected State
        ASSERT_ERR(0);
    }
}

void user_catch_rest_hndl(ke_msg_id_t const msgid,
                          void const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case GAPC_PARAM_UPDATED_IND:
        {
            // Cast the "param" pointer to the appropriate message structure
            struct gapc_param_updated_ind const *msg_param = (struct gapc_param_updated_ind const *)(param);

            // Check if updated Conn Params filled to preffered ones
            if ((msg_param->con_interval >= user_connection_param_conf.intv_min) &&
                (msg_param->con_interval <= user_connection_param_conf.intv_max) &&
                (msg_param->con_latency == user_connection_param_conf.latency) &&
                (msg_param->sup_to == user_connection_param_conf.time_out))
            {
            }
        } break;
        
        default:
            break;
    }
}


/// @} APP
