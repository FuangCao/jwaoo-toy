/**
 ****************************************************************************************
 *
 * @file user_periph_setup.h
 *
 * @brief Peripherals setup header file.
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

#ifndef _USER_PERIPH_SETUP_H_
#define _USER_PERIPH_SETUP_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "global_io.h"
#include "arch.h"
#include "da1458x_periph_setup.h"
#include "uart.h"
#include "gpio.h"

/*
 * DEFINES
 ****************************************************************************************
 */

//*** <<< Use Configuration Wizard in Context Menu >>> ***

// <o> DK selection <0=> As in da1458x_periph_setup.h <1=> Basic <2=> Pro <3=> Expert
#define HW_CONFIG (0)

#define HW_CONFIG_BASIC_DK  ((HW_CONFIG==0 && SDK_CONFIG==1) || HW_CONFIG==1)
#define HW_CONFIG_PRO_DK    ((HW_CONFIG==0 && SDK_CONFIG==2) || HW_CONFIG==2)
#define HW_CONFIG_EXPERT_DK ((HW_CONFIG==0 && SDK_CONFIG==3) || HW_CONFIG==3)

//*** <<< end of configuration section >>>    ***

#define KB(a)				((a) << 10)
#define LED_LEVEL_MAX		10

#define LED1_GPIO_PORT		GPIO_PORT_1
#define LED1_GPIO_PIN		GPIO_PIN_0
#define LED1_OPEN			GPIO_SetActive(LED1_GPIO_PORT, LED1_GPIO_PIN)
#define LED1_CLOSE			GPIO_SetInactive(LED1_GPIO_PORT, LED1_GPIO_PIN)
#define LED1_STATUS			GPIO_GetPinStatus(LED1_GPIO_PORT, LED1_GPIO_PIN)
#define LED1_BLINK			(LED1_STATUS ? LED1_CLOSE : LED1_OPEN)
#define LED1_RESERVE		RESERVE_GPIO(LED1, LED1_GPIO_PORT, LED1_GPIO_PIN, PID_GPIO)
#define LED1_CONFIG			GPIO_ConfigurePin(LED1_GPIO_PORT, LED1_GPIO_PIN, OUTPUT, PID_GPIO, false)

#define LED2_GPIO_PORT		GPIO_PORT_1
#define LED2_GPIO_PIN		GPIO_PIN_1
#define LED2_OPEN			GPIO_SetActive(LED2_GPIO_PORT, LED2_GPIO_PIN)
#define LED2_CLOSE			GPIO_SetInactive(LED2_GPIO_PORT, LED2_GPIO_PIN)
#define LED2_STATUS			GPIO_GetPinStatus(LED2_GPIO_PORT, LED2_GPIO_PIN)
#define LED2_BLINK			(LED2_STATUS ? LED2_CLOSE : LED2_OPEN)
#define LED2_RESERVE		RESERVE_GPIO(LED2, LED2_GPIO_PORT, LED2_GPIO_PIN, PID_GPIO)
#define LED2_CONFIG			GPIO_ConfigurePin(LED2_GPIO_PORT, LED2_GPIO_PIN, OUTPUT, PID_GPIO, false)

#define BLUZZ_GPIO_PORT		GPIO_PORT_2
#define BLUZZ_GPIO_PIN		GPIO_PIN_0
#define BLUZZ_OPEN			GPIO_SetActive(BLUZZ_GPIO_PORT, BLUZZ_GPIO_PIN)
#define BLUZZ_CLOSE			GPIO_SetInactive(BLUZZ_GPIO_PORT, BLUZZ_GPIO_PIN)
#define BLUZZ_RESERVE		RESERVE_GPIO(BLUZZ, BLUZZ_GPIO_PORT, BLUZZ_GPIO_PIN, PID_GPIO)
#define BLUZZ_CONFIG		GPIO_ConfigurePin(BLUZZ_GPIO_PORT, BLUZZ_GPIO_PIN, OUTPUT, PID_GPIO, false)

#define RELAY_GPIO_PORT		GPIO_PORT_2
#define RELAY_GPIO_PIN		GPIO_PIN_3
#define RELAY_OPEN			GPIO_SetInactive(RELAY_GPIO_PORT, RELAY_GPIO_PIN)
#define RELAY_CLOSE			GPIO_SetActive(RELAY_GPIO_PORT, RELAY_GPIO_PIN)
#define RELAY_RESERVE		RESERVE_GPIO(RELAY, RELAY_GPIO_PORT, RELAY_GPIO_PIN, PID_GPIO)
#define RELAY_CONFIG		GPIO_ConfigurePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, OUTPUT, PID_GPIO, true)

#define KEY_GPIO_RESERVE(index) \
	RESERVE_GPIO(KEY##index, KEY##index##_GPIO_PORT, KEY##index##_GPIO_PIN, PID_GPIO)

#define KEY_GPIO_CONFIG(index) \
	GPIO_ConfigurePin(KEY##index##_GPIO_PORT, KEY##index##_GPIO_PIN, INPUT_PULLUP, PID_GPIO, true)

#define KEY1_GPIO_PORT		GPIO_PORT_2
#define KEY1_GPIO_PIN		GPIO_PIN_7
#define KEY1_GPIO_IRQ		GPIO0_IRQn

#define KEY2_GPIO_PORT		GPIO_PORT_0
#define KEY2_GPIO_PIN		GPIO_PIN_7
#define KEY2_GPIO_IRQ		GPIO1_IRQn

#if 0
#define KEY3_GPIO_PORT		GPIO_PORT_2
#define KEY3_GPIO_PIN		GPIO_PIN_3
#define KEY3_GPIO_IRQ		GPIO2_IRQn
#endif

#define KEY4_GPIO_PORT		GPIO_PORT_2
#define KEY4_GPIO_PIN		GPIO_PIN_4
#define KEY4_GPIO_IRQ		GPIO3_IRQn

#define LEDR_GPIO_PORT		GPIO_PORT_2
#define LEDR_GPIO_PIN		GPIO_PIN_6
#define LEDR_RESERVE		RESERVE_GPIO(LEDR, LEDR_GPIO_PORT, LEDR_GPIO_PIN, PID_PWM2)
#define LEDR_LEVEL(level)	LED_SET_LEVEL(R, 2, level)

#define LEDG_GPIO_PORT		GPIO_PORT_2
#define LEDG_GPIO_PIN		GPIO_PIN_8
#define LEDG_RESERVE		RESERVE_GPIO(LEDG, LEDG_GPIO_PORT, LEDG_GPIO_PIN, PID_PWM3)
#define LEDG_LEVEL(level)	LED_SET_LEVEL(G, 3, level)

#define LEDB_GPIO_PORT		GPIO_PORT_2
#define LEDB_GPIO_PIN		GPIO_PIN_9
#define LEDB_RESERVE		RESERVE_GPIO(LEDB, LEDB_GPIO_PORT, LEDB_GPIO_PIN, PID_PWM4)
#define LEDB_LEVEL(level)	LED_SET_LEVEL(B, 4, level)

#define LED_SET_LEVEL(color, pwm, level) \
	do { \
		if (level > 0) { \
			timer2_set_sw_pause(PWM_2_3_4_SW_PAUSE_ENABLED); \
			timer2_set_pwm##pwm##_duty_cycle(LED_LEVEL_MAX - (level)); \
			timer2_set_sw_pause(PWM_2_3_4_SW_PAUSE_DISABLED); \
			GPIO_ConfigurePin(LED##color##_GPIO_PORT, LED##color##_GPIO_PIN, OUTPUT, PID_PWM##pwm, true); \
		} else { \
			GPIO_ConfigurePin(LED##color##_GPIO_PORT, LED##color##_GPIO_PIN, OUTPUT, PID_GPIO, true); \
		} \
	} while (0)

/****************************************************************************************/
/* i2c eeprom configuration                                                             */
/****************************************************************************************/

#define I2C1_GPIO_PORT		GPIO_PORT_1
#define I2C1_SDA_GPIO_PIN	GPIO_PIN_2
#define I2C1_SCL_GPIO_PIN	GPIO_PIN_3

#define I2C2_GPIO_PORT		GPIO_PORT_0
#define I2C2_SDA_GPIO_PIN	GPIO_PIN_2
#define I2C2_SCL_GPIO_PIN	GPIO_PIN_7

#define I2C_EEPROM_SIZE		0x20000         // EEPROM size in bytes
#define I2C_EEPROM_PAGE		256             // EEPROM's page size in bytes
#define I2C_SPEED_MODE		I2C_FAST        // 1: standard mode (100 kbits/s), 2: fast mode (400 kbits/s)
#define I2C_ADDRESS_MODE	I2C_7BIT_ADDR   // 0: 7-bit addressing, 1: 10-bit addressing
#define I2C_ADDRESS_SIZE	I2C_2BYTES_ADDR // 0: 8-bit memory address, 1: 16-bit memory address, 3: 24-bit memory address


/****************************************************************************************/
/* SPI FLASH configuration                                                              */
/****************************************************************************************/

#define SPI_FLASH_DEFAULT_SIZE	0x40000    // SPI Flash memory size in bytes
#define SPI_FLASH_DEFAULT_PAGE	0x100
#define SPI_SECTOR_SIZE			4096
#define SPI_SECTOR_SIZE_MASK	((SPI_SECTOR_SIZE) - 1)

#define SPI_CODE_SIZE			KB(32)
#define SPI_PART_FRONT_CODE		0
#define SPI_PART_BACK_CODE		(SPI_PART_FRONT_CODE + SPI_CODE_SIZE)
#define SPI_PART_DEVICE_DATA	(SPI_PART_BACK_CODE + SPI_CODE_SIZE)

#define SPI_CS_GPIO_PORT		GPIO_PORT_0
#define SPI_CS_GPIO_PIN			GPIO_PIN_3

#define SPI_CLK_GPIO_PORT		GPIO_PORT_0
#define SPI_CLK_GPIO_PIN		GPIO_PIN_0

#define SPI_DO_GPIO_PORT		GPIO_PORT_0
#define SPI_DO_GPIO_PIN			GPIO_PIN_6

#define SPI_DI_GPIO_PORT		GPIO_PORT_0
#define SPI_DI_GPIO_PIN			GPIO_PIN_5

/* Enable WKUPCT. Required by wkupct_quadec driver. */
#define WKUP_ENABLED

/****************************************************************************************/
/* uart pin configuration                                                               */
/****************************************************************************************/

#define UART1_TX_GPIO_PORT		GPIO_PORT_0
#define UART1_TX_GPIO_PIN		GPIO_PIN_4

#define UART1_RX_GPIO_PORT		GPIO_PORT_0
#define UART1_RX_GPIO_PIN		GPIO_PIN_1

#define UART1_RTSN_GPIO_PORT	GPIO_PORT_0
#define UART1_RTSN_GPIO_PIN		GPIO_PIN_6

#define UART1_CTSN_GPIO_PORT	GPIO_PORT_0
#define UART1_CTSN_GPIO_PIN		GPIO_PIN_7

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable pad's and peripheral clocks assuming that peripherals' power domain
 * is down. The Uart and SPI clocks are set.
 * @return void
 ****************************************************************************************
 */
void periph_init(void);

/**
 ****************************************************************************************
 * @brief Map port pins. The Uart and SPI port pins and GPIO ports are mapped.
 * @return void
 ****************************************************************************************
 */
void set_pad_functions(void);

/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 * @return void
 ****************************************************************************************
 */
void GPIO_reservations(void);

#endif // _USER_PERIPH_SETUP_H_
