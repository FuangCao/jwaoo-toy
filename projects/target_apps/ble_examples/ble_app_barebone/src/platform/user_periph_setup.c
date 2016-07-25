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
#include "mpu6050.h"
#include "fdc1004.h"
#include "jwaoo_toy.h"

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
	// RESERVE_GPIO(UART2_RX, UART1_RX_GPIO_PORT, UART1_RX_GPIO_PIN, PID_UART2_RX);
#endif

	LED1_RESERVE;
	LED2_RESERVE;

#if 0
	BLUZZ_RESERVE;
	RELAY_RESERVE;
#endif

	KEY_GPIO_RESERVE(1);
	KEY_GPIO_RESERVE(2);
#ifdef KEY3_GPIO_PORT
	KEY_GPIO_RESERVE(3);
#endif
	KEY_GPIO_RESERVE(4);

	LEDR_RESERVE;
	LEDG_RESERVE;
	LEDB_RESERVE;

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
	// GPIO_ConfigurePin(UART1_RX_GPIO_PORT, UART1_RX_GPIO_PIN, INPUT, PID_UART2_RX, false);
#endif

	LED1_CONFIG;
	LED2_CONFIG;

#if 0
	BLUZZ_CONFIG;
	RELAY_CONFIG;
#endif

	KEY_GPIO_CONFIG(1);
	KEY_GPIO_CONFIG(2);
#ifdef KEY3_GPIO_PORT
	KEY_GPIO_CONFIG(3);
#endif
	KEY_GPIO_CONFIG(4);

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
		value = 0;
		level = GPIO_IRQ_INPUT_LEVEL_LOW;
	} else {
		value = 1;
		level = GPIO_IRQ_INPUT_LEVEL_HIGH;
	}

	GPIO_SetIRQInputLevel(irq, level);
	jwaoo_toy_process_key(code, value);

	return value;
}

static void app_config_key(IRQn_Type irq, GPIO_handler_function_t isr, GPIO_PORT port, GPIO_PIN pin)
{
	GPIO_RegisterCallback(irq, isr);
	GPIO_EnableIRQ(port, pin, irq, GPIO_GetPinStatus(port, pin), true, 60);
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
	set_tmr_div(CLK_PER_REG_TMR_DIV_8);
	timer2_init(HW_CAN_NOT_PAUSE_PWM_2_3_4, PWM_2_3_4_SW_PAUSE_ENABLED, LED_LEVEL_MAX);
	LEDR_LEVEL(0);
	LEDG_LEVEL(0);
	LEDB_LEVEL(0);

	app_spi_flash_init();
	jwaoo_toy_read_device_data();

	i2c_init(I2C_SPEED_400K, I2C_ADDRESS_MODE_7BIT);
	MPU6050_Initialize();
	MPU6050_GetDeviceID();

	fdc1004_init();

   // Enable the pads
    SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
}
