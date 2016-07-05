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
#include "mpu6050.h"
#include "fdc1004.h"

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
	RESERVE_GPIO(UART2_RX, UART1_RX_GPIO_PORT, UART1_RX_GPIO_PIN, PID_UART2_RX);
#endif

	LED1_RESERVE;
	LED2_RESERVE;
	BLUZZ_RESERVE;
	RELAY_RESERVE;

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
	GPIO_ConfigurePin(UART1_RX_GPIO_PORT, UART1_RX_GPIO_PIN, INPUT, PID_UART2_RX, false);
#endif

	LED1_CONFIG;
	LED2_CONFIG;
	BLUZZ_CONFIG;
	RELAY_CONFIG;

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
	int ret;
	SPI_Pad_t flash_cs_pad = { SPI_CS_GPIO_PORT, SPI_CS_GPIO_PIN };

	spi_init(&flash_cs_pad, SPI_MODE_8BIT, SPI_ROLE_MASTER, SPI_CLK_IDLE_POL_LOW, SPI_PHA_MODE_0, SPI_MINT_DISABLE, SPI_XTAL_DIV_8);

	spi_flash_release_from_power_down();

	ret = spi_flash_auto_detect();
	if (ret == SPI_FLASH_AUTO_DETECT_NOT_DETECTED)
	{
		spi_flash_init(SPI_FLASH_DEFAULT_SIZE, SPI_FLASH_DEFAULT_PAGE);
	}
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

	app_spi_flash_init();

	i2c_init(I2C_SPEED_400K, I2C_ADDRESS_MODE_7BIT);
	MPU6050_Initialize();
	MPU6050_GetDeviceID();

	fdc1004_init();

   // Enable the pads
    SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
}
