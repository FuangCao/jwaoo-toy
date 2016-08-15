/**
 ****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief Peripherals setup and initialization.
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

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration
#include "user_periph_setup.h"       // peripheral configuration
#include "global_io.h"
#include "gpio.h"
#include "uart.h"                    // UART initialization
#include "spi_flash.h"
#include "i2c.h"
#include "pwm.h"
#include "adc.h"
#include "app.h"
#include "mpu6050.h"
#include "fdc1004.h"
#include "jwaoo_toy.h"
#include "jwaoo_toy_task.h"

static bool jwaoo_pwm_set_level_base(struct jwaoo_pwm_device *device, uint8_t level)
{
#if PWM_AUTO_ALLOC
	static uint8_t busy_mask;
#endif

	if (app_suspended) {
		level = 0;
	}

	if (device->active_low) {
		level = PWM_LEVEL_MAX - level;
	}

	if (level > 0 && level < PWM_LEVEL_MAX) {
		uint8_t pwm = device->pwm;

#if PWM_AUTO_ALLOC
		if (pwm == 0xFF) {
			for (pwm = 0; (busy_mask & (1 << pwm)); pwm++) {
				if (pwm >= 2) {
					return false;
				}
			}

			busy_mask |= (1 << pwm);
			device->pwm = pwm;
		}
#endif

		timer2_set_sw_pause(PWM_2_3_4_SW_PAUSE_ENABLED);
		SetWord16(PWM2_DUTY_CYCLE + (pwm * 2), level);
		timer2_set_sw_pause(PWM_2_3_4_SW_PAUSE_DISABLED);
		GPIO_ConfigurePin(device->port, device->pin, OUTPUT, (GPIO_FUNCTION) (PID_PWM2 + pwm), device->active_low);
	} else {
		GPIO_ConfigurePin(device->port, device->pin, OUTPUT, PID_GPIO, level > 0);

#if PWM_AUTO_ALLOC
		if (device->pwm != 0xFF) {
			busy_mask &= ~(1 << device->pwm);
			device->pwm = 0xFF;
		}
#endif
	}

	// println("pwm = %d, level = %d, busy = 0x%02x", device->pwm, device->level, busy_mask);

	return true;
}

static void jwaoo_led_set_level_handler(struct jwaoo_pwm_device *device, uint8_t level)
{
	jwaoo_pwm_set_level_base(device, level);
	device->level = level;
}

static void jwaoo_moto_set_level_handler(struct jwaoo_pwm_device *device, uint8_t level)
{
	uint8_t pwm_level;

	if (level > 0 && device->level == 0) {
		pwm_level = MOTO_LEVEL_BOOST;
		ke_timer_set(JWAOO_TOY_MOTO_BOOST, TASK_APP, MOTO_BOOST_TIME);
	} else if (level < MOTO_SPEED_MAX) {
		if (level > 0) {
			pwm_level = level * MOTO_LEVEL_RANGE / MOTO_SPEED_MAX + MOTO_LEVEL_MIN;
		} else {
			pwm_level = 0;
		}
	} else {
		pwm_level = PWM_LEVEL_MAX;
	}

	jwaoo_pwm_set_level_base(device, pwm_level);
	device->level = level;
}

struct jwaoo_pwm_device jwaoo_pwm_moto = {
#if PWM_AUTO_ALLOC
	.pwm = 0xFF,
#else
	.pwm = 0,
#endif
	.port = MOTO_GPIO_PORT,
	.pin = MOTO_GPIO_PIN,
	.blink_timer = JWAOO_TOY_MOTO_BLINK,
	.set_level = jwaoo_moto_set_level_handler
};

struct jwaoo_pwm_device jwaoo_pwm_led1 = {
#if PWM_AUTO_ALLOC
	.pwm = 0xFF,
#else
	.pwm = 1,
#endif
	.port = LED1_GPIO_PORT,
	.pin = LED1_GPIO_PIN,
	.blink_timer = JWAOO_TOY_LED1_BLINK,
	.set_level = jwaoo_led_set_level_handler
};

struct jwaoo_pwm_device jwaoo_pwm_led2 = {
#if PWM_AUTO_ALLOC
	.pwm = 0xFF,
#else
	.pwm = 2,
#endif
	.port = LED2_GPIO_PORT,
	.pin = LED2_GPIO_PIN,
	.blink_timer = JWAOO_TOY_LED2_BLINK,
	.set_level = jwaoo_led_set_level_handler
};

static bool jwaoo_pwm_set_blink_direction(struct jwaoo_pwm_device *device, bool add)
{
	device->blink_add = add;

	if (device->blink_count == 0) {
		return false;
	}

	if (--device->blink_count) {
		return false;
	}

	device->blink_delay = 0;

	return true;
 }

bool jwaoo_pwm_blink_walk(struct jwaoo_pwm_device *device)
{
	uint8_t level;
	bool complete;

	if (app_suspended) {
		level = 0;
		complete = true;
	} else {
		complete = (device->blink_delay == 0);
		if (complete) {
			level = device->blink_min;
		} else {
			if (device->blink_add) {
				level = device->level + device->blink_step;
				if (level > device->blink_max) {
					level = device->blink_max - device->blink_step;
					complete = jwaoo_pwm_set_blink_direction(device, false);
				}
			} else {
				level = device->level - device->blink_step;
				if (level < device->blink_min || level > device->blink_max) {
					level = device->blink_min + device->blink_step;
					complete = jwaoo_pwm_set_blink_direction(device, true);
				}
			}

			if (complete) {
				level = device->blink_min;
			} else {
				ke_timer_set(device->blink_timer, TASK_APP, device->blink_delay);
			}
		}
	}

	device->set_level(device, level);

	return complete;
}

void jwaoo_pwm_blink_set(struct jwaoo_pwm_device *device, uint8_t min, uint8_t max, uint8_t step, uint8_t delay, uint8_t count)
{
	ke_timer_clear(device->blink_timer, TASK_APP);
	device->set_level(device, min);

	if (min < max && step > 0) {
		device->blink_add = true;
		device->blink_min = min;
		device->blink_max = max;
		device->blink_step = step;
		device->blink_delay = delay;
		device->blink_count = count;

		ke_timer_set(device->blink_timer, TASK_APP, 1);
	} else {
		device->blink_min = device->blink_max = min;
		device->blink_delay = 0;
	}
}

void jwaoo_pwm_blink_sawtooth(struct jwaoo_pwm_device *device, uint8_t min, uint8_t max, uint8_t step, uint32_t cycle, uint8_t count)
{
	uint8_t delay = cycle * step / (max - min) / 20;
	if (delay < 1) {
		delay = 1;
	}

	jwaoo_pwm_blink_set(device, min, max, step, delay, count);
}

void jwaoo_pwm_blink_square(struct jwaoo_pwm_device *device, uint8_t min, uint8_t max, uint32_t cycle, uint8_t count)
{
	uint8_t delay = cycle / 20;
	if (delay < 1) {
		delay = 1;
	}

	jwaoo_pwm_blink_set(device, min, max, max - min, delay, count);
}

#if DEVELOPMENT_DEBUG

void GPIO_reservations(void)
{
/*
* Globally reserved GPIOs reservation
*/

/*
* Application specific GPIOs reservation. Used only in Development mode (#if DEVELOPMENT_DEBUG)

i.e.
    RESERVE_GPIO(DESCRIPTIVE_NAME, GPIO_PORT_0, GPIO_PIN_1, PID_GPIO);    //Reserve P_01 as Generic Purpose I/O
*/

#ifdef CFG_PRINTF_UART2
	RESERVE_GPIO(UART2_TX, UART1_TX_GPIO_PORT, UART1_TX_GPIO_PIN, PID_UART2_TX);
#ifdef UART1_RX_GPIO_PORT
	RESERVE_GPIO(UART2_RX, UART1_RX_GPIO_PORT, UART1_RX_GPIO_PIN, PID_UART2_RX);
#endif
#endif

	LED1_RESERVE;
	LED2_RESERVE;

#ifdef BLUZZ_RESERVE
	BLUZZ_RESERVE;
#endif

#ifdef RELAY_RESERVE
	RELAY_RESERVE;
#endif

	KEY_GPIO_RESERVE(1);
	KEY_GPIO_RESERVE(2);
#ifdef KEY3_GPIO_PORT
	KEY_GPIO_RESERVE(3);
#endif
	KEY_GPIO_RESERVE(4);

#ifdef LEDR_RESERVE
	LEDR_RESERVE;
#endif

#ifdef LEDG_RESERVE
	LEDG_RESERVE;
#endif

#ifdef LEDB_RESERVE
	LEDB_RESERVE;
#endif

#ifdef MOTO_RESERVE
	MOTO_RESERVE;
#endif

#ifdef BATT_ADC_RESERVE
	BATT_ADC_RESERVE;
#endif

#ifdef CHG_DET_RESERVE
	CHG_DET_RESERVE;
#endif

#ifdef CHG_STAT_RESERVE
	CHG_STAT_RESERVE;
#endif

	RESERVE_GPIO(SPI_CLK, SPI_CLK_GPIO_PORT, SPI_CLK_GPIO_PIN, PID_SPI_CLK);
	RESERVE_GPIO(SPI_DO, SPI_DO_GPIO_PORT, SPI_DO_GPIO_PIN, PID_SPI_DO);
	RESERVE_GPIO(SPI_DI, SPI_DI_GPIO_PORT, SPI_DI_GPIO_PIN, PID_SPI_DI);
	RESERVE_GPIO(SPI_EN, SPI_CS_GPIO_PORT, SPI_CS_GPIO_PIN, PID_SPI_EN);

	RESERVE_GPIO(I2C_SCL, I2C1_GPIO_PORT, I2C1_SCL_GPIO_PIN, PID_I2C_SCL);
	RESERVE_GPIO(I2C_SDA, I2C1_GPIO_PORT, I2C1_SDA_GPIO_PIN, PID_I2C_SDA);
}
#endif //DEVELOPMENT_DEBUG

void set_pad_functions(void)        // set gpio port function mode
{
#ifdef CFG_PRINTF_UART2
	GPIO_ConfigurePin(UART1_TX_GPIO_PORT, UART1_TX_GPIO_PIN, OUTPUT, PID_UART2_TX, false);
#ifdef UART1_RX_GPIO_PORT
	GPIO_ConfigurePin(UART1_RX_GPIO_PORT, UART1_RX_GPIO_PIN, INPUT, PID_UART2_RX, false);
#endif
#endif

#ifdef LED1_CONFIG
	LED1_CONFIG;
#endif

#ifdef LED2_CONFIG
	LED2_CONFIG;
#endif

#ifdef BLUZZ_CONFIG
	BLUZZ_CONFIG;
#endif

#ifdef RELAY_CONFIG
	RELAY_CONFIG;
#endif

#ifdef BATT_ADC_CONFIG
	BATT_ADC_CONFIG;
#endif

	KEY_GPIO_CONFIG(1);
	KEY_GPIO_CONFIG(2);
#ifdef KEY3_GPIO_PORT
	KEY_GPIO_CONFIG(3);
#endif
	KEY_GPIO_CONFIG(4);

#ifdef CHG_DET_CONFIG
	CHG_DET_CONFIG;
#endif

#ifdef CHG_STAT_CONFIG
	CHG_STAT_CONFIG;
#endif

	GPIO_ConfigurePin(SPI_CS_GPIO_PORT, SPI_CS_GPIO_PIN, OUTPUT, PID_SPI_EN, true);
	GPIO_ConfigurePin(SPI_CLK_GPIO_PORT, SPI_CLK_GPIO_PIN, OUTPUT, PID_SPI_CLK, false);
	GPIO_ConfigurePin(SPI_DO_GPIO_PORT, SPI_DO_GPIO_PIN, OUTPUT, PID_SPI_DO, false);
	GPIO_ConfigurePin(SPI_DI_GPIO_PORT, SPI_DI_GPIO_PIN, INPUT, PID_SPI_DI, false);

	GPIO_ConfigurePin(I2C1_GPIO_PORT, I2C1_SCL_GPIO_PIN, OUTPUT, PID_I2C_SCL, false);
	GPIO_ConfigurePin(I2C1_GPIO_PORT, I2C1_SDA_GPIO_PIN, OUTPUT, PID_I2C_SDA, false);

/*
* Configure application ports.
i.e.
    GPIO_ConfigurePin( GPIO_PORT_0, GPIO_PIN_1, OUTPUT, PID_GPIO, false ); // Set P_01 as Generic purpose Output
*/
}

static void app_spi_flash_init(void)
{
	if (spi_flash_enable(SPI_CS_GPIO_PORT, SPI_CS_GPIO_PIN) < 0)
	{
		spi_flash_init(SPI_FLASH_DEFAULT_SIZE, SPI_FLASH_DEFAULT_PAGE);
	}
}

static uint8_t app_process_key(IRQn_Type irq, uint8_t code, GPIO_PORT port, GPIO_PIN pin)
{
	uint8_t value;
	GPIO_IRQ_INPUT_LEVEL level;

	if (GPIO_GetPinStatus(port, pin)) {
		value = !KEY_ACTIVE_LOW;
		level = GPIO_IRQ_INPUT_LEVEL_LOW;
	} else {
		value = KEY_ACTIVE_LOW;
		level = GPIO_IRQ_INPUT_LEVEL_HIGH;
	}

	GPIO_SetIRQInputLevel(irq, level);
	jwaoo_toy_process_key(code, value);

	return value;
}

static void app_config_key(IRQn_Type irq, GPIO_handler_function_t isr, GPIO_PORT port, GPIO_PIN pin)
{
	GPIO_RegisterCallback(irq, isr);
	GPIO_EnableIRQ(port, pin, irq, GPIO_GetPinStatus(port, pin), KEY_ACTIVE_LOW, 60);
}

static void app_key1_isr(void)
{
	app_process_key(KEY1_GPIO_IRQ, 0, KEY1_GPIO_PORT, KEY1_GPIO_PIN);
}

static void app_key2_isr(void)
{
	app_process_key(KEY2_GPIO_IRQ, 1, KEY2_GPIO_PORT, KEY2_GPIO_PIN);
}

#ifdef KEY3_GPIO_IRQ
static void app_key3_isr(void)
{
	app_process_key(KEY3_GPIO_IRQ, 2, KEY3_GPIO_PORT, KEY3_GPIO_PIN);
}
#endif

static void app_key4_isr(void)
{
	app_process_key(KEY4_GPIO_IRQ, 3, KEY4_GPIO_PORT, KEY4_GPIO_PIN);
}

void periph_init(void)
{
    // Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP));

    SetBits16(CLK_16M_REG, XTAL16_BIAS_SH_ENABLE, 1);

    //rom patch
    patch_func();

    //Init pads
    set_pad_functions();

    // (Re)Initialize peripherals
    // i.e.
    //  uart_init(UART_BAUDRATE_115K2, 3);

#ifdef CFG_PRINTF_UART2
    SetBits16(CLK_PER_REG, UART2_ENABLE, 1);
    uart2_init(UART_BAUDRATE_115K2, 3);
#endif

	app_config_key(KEY1_GPIO_IRQ, app_key1_isr, KEY1_GPIO_PORT, KEY1_GPIO_PIN);
	app_config_key(KEY2_GPIO_IRQ, app_key2_isr, KEY2_GPIO_PORT, KEY2_GPIO_PIN);
#ifdef KEY3_GPIO_IRQ
	app_config_key(KEY3_GPIO_IRQ, app_key3_isr, KEY3_GPIO_PORT, KEY3_GPIO_PIN);
#endif
	app_config_key(KEY4_GPIO_IRQ, app_key4_isr, KEY4_GPIO_PORT, KEY4_GPIO_PIN);

	set_tmr_enable(CLK_PER_REG_TMR_ENABLED);
	// set_tmr_div(CLK_PER_REG_TMR_DIV_8);
	set_tmr_div(CLK_PER_REG_TMR_DIV_1);
	timer2_init(HW_CAN_NOT_PAUSE_PWM_2_3_4, PWM_2_3_4_SW_PAUSE_ENABLED, PWM_LEVEL_MAX);

#ifdef LEDR_SET_LEVEL
	LEDR_SET_LEVEL(0);
#endif

#ifdef LEDG_SET_LEVEL
	LEDG_SET_LEVEL(0);
#endif

#ifdef LEDB_SET_LEVEL
	LEDB_SET_LEVEL(0);
#endif

#ifdef MOTO_SET_LEVEL
	MOTO_SET_LEVEL(0);
#endif

#ifdef LED1_SET_LEVEL
	LED1_SET_LEVEL(0);
#endif

#ifdef LED2_SET_LEVEL
	LED2_SET_LEVEL(0);
#endif

	app_spi_flash_init();
	jwaoo_toy_read_device_data();

	i2c_init(I2C_SPEED_400K, I2C_ADDRESS_MODE_7BIT);

   // Enable the pads
    SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
}
