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
#define PWM_AUTO_ALLOC		0
#define PWM_LEVEL_MAX		100
#define MOTO_LEVEL_MIN		28
#define MOTO_LEVEL_BOOST	PWM_LEVEL_MAX
#define MOTO_LEVEL_RANGE	(PWM_LEVEL_MAX - MOTO_LEVEL_MIN)
#define MOTO_SPEED_MAX		18
#define MOTO_BOOST_TIME		4
#define BATT_VOLTAGE_MIN	3000
#define BATT_VOLTAGE_MAX	4200
#define BATT_LEVEL_LOW		8

#define MOTO_GPIO_PORT			GPIO_PORT_2
#define MOTO_GPIO_PIN			GPIO_PIN_0
#define MOTO_RESERVE			RESERVE_GPIO(MOTO, MOTO_GPIO_PORT, MOTO_GPIO_PIN, PID_GPIO)

#if 0
#define LEDR_GPIO_PORT			GPIO_PORT_2
#define LEDR_GPIO_PIN			GPIO_PIN_6
#define LEDR_RESERVE			RESERVE_GPIO(LEDR, LEDR_GPIO_PORT, LEDR_GPIO_PIN, PID_PWM2)
#define LEDR_SET_LEVEL(level)	LED_SET_LEVEL(LEDR, 2, level)

#define LEDG_GPIO_PORT			GPIO_PORT_2
#define LEDG_GPIO_PIN			GPIO_PIN_8
#define LEDG_RESERVE			RESERVE_GPIO(LEDG, LEDG_GPIO_PORT, LEDG_GPIO_PIN, PID_PWM3)
#define LEDG_SET_LEVEL(level)	LED_SET_LEVEL(LEDG, 3, level)

#define LEDB_GPIO_PORT			GPIO_PORT_2
#define LEDB_GPIO_PIN			GPIO_PIN_9
#define LEDB_RESERVE			RESERVE_GPIO(LEDB, LEDB_GPIO_PORT, LEDB_GPIO_PIN, PID_PWM4)
#define LEDB_SET_LEVEL(level)	LED_SET_LEVEL(LEDB, 4, level)
#endif

#define LED1_GPIO_PORT			GPIO_PORT_0
#define LED1_GPIO_PIN			GPIO_PIN_7
#define LED1_RESERVE			RESERVE_GPIO(LED1, LED1_GPIO_PORT, LED1_GPIO_PIN, PID_GPIO)

#define LED2_GPIO_PORT			GPIO_PORT_2
#define LED2_GPIO_PIN			GPIO_PIN_9
#define LED2_RESERVE			RESERVE_GPIO(LED2, LED2_GPIO_PORT, LED2_GPIO_PIN, PID_GPIO)

#if 0
#define BLUZZ_GPIO_PORT			GPIO_PORT_2
#define BLUZZ_GPIO_PIN			GPIO_PIN_0
#define BLUZZ_OPEN				GPIO_SetActive(BLUZZ_GPIO_PORT, BLUZZ_GPIO_PIN)
#define BLUZZ_CLOSE				GPIO_SetInactive(BLUZZ_GPIO_PORT, BLUZZ_GPIO_PIN)
#define BLUZZ_RESERVE			RESERVE_GPIO(BLUZZ, BLUZZ_GPIO_PORT, BLUZZ_GPIO_PIN, PID_GPIO)
#define BLUZZ_CONFIG			GPIO_ConfigurePin(BLUZZ_GPIO_PORT, BLUZZ_GPIO_PIN, OUTPUT, PID_GPIO, false)

#define RELAY_GPIO_PORT			GPIO_PORT_2
#define RELAY_GPIO_PIN			GPIO_PIN_3
#define RELAY_OPEN				GPIO_SetInactive(RELAY_GPIO_PORT, RELAY_GPIO_PIN)
#define RELAY_CLOSE				GPIO_SetActive(RELAY_GPIO_PORT, RELAY_GPIO_PIN)
#define RELAY_RESERVE			RESERVE_GPIO(RELAY, RELAY_GPIO_PORT, RELAY_GPIO_PIN, PID_GPIO)
#define RELAY_CONFIG			GPIO_ConfigurePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, OUTPUT, PID_GPIO, true)
#endif

#define KEY_ACTIVE_LOW			0

#define KEY_GPIO_RESERVE(index) \
	RESERVE_GPIO(KEY##index, KEY##index##_GPIO_PORT, KEY##index##_GPIO_PIN, PID_GPIO)

#if KEY_ACTIVE_LOW
#define KEY_GPIO_CONFIG(index) \
	GPIO_ConfigurePin(KEY##index##_GPIO_PORT, KEY##index##_GPIO_PIN, INPUT_PULLUP, PID_GPIO, true)
#else
#define KEY_GPIO_CONFIG(index) \
	GPIO_ConfigurePin(KEY##index##_GPIO_PORT, KEY##index##_GPIO_PIN, INPUT_PULLDOWN, PID_GPIO, true)
#endif

#define KEY1_GPIO_PORT			GPIO_PORT_2
#define KEY1_GPIO_PIN			GPIO_PIN_1
#define KEY1_GPIO_IRQ			GPIO0_IRQn

#define KEY2_GPIO_PORT			GPIO_PORT_2
#define KEY2_GPIO_PIN			GPIO_PIN_2
#define KEY2_GPIO_IRQ			GPIO1_IRQn

#if 1
#define KEY3_GPIO_PORT			GPIO_PORT_2
#define KEY3_GPIO_PIN			GPIO_PIN_3
#define KEY3_GPIO_IRQ			GPIO2_IRQn
#endif

#define KEY4_GPIO_PORT			GPIO_PORT_2
#define KEY4_GPIO_PIN			GPIO_PIN_4
#define KEY4_GPIO_IRQ			GPIO3_IRQn

#define BATT_ADC_GPIO_PORT		GPIO_PORT_0
#define BATT_ADC_GPIO_PIN		GPIO_PIN_1
#define BATT_ADC_RESERVE		RESERVE_GPIO(BATT_ADC, BATT_ADC_GPIO_PORT, BATT_ADC_GPIO_PIN, PID_ADC)
#define BATT_ADC_CONFIG 		GPIO_ConfigurePin(BATT_ADC_GPIO_PORT, BATT_ADC_GPIO_PIN, INPUT, PID_ADC, true)

#if 0
#define CHG_DET_GPIO_PORT
#define CHG_DET_GPIO_PIN
#define CHG_DET_RESERVE			// RESERVE_GPIO(CHG_DET, CHG_DET_GPIO_PORT, CHG_DET_GPIO_PIN, PID_GPIO)
#define CHG_DET_CONFIG 			// GPIO_ConfigurePin(CHG_DET_GPIO_PORT, CHG_DET_GPIO_PIN, INPUT, PID_GPIO, true)
#define CHG_DET_GPIO_GET		0
#endif

#define CHG_STAT_GPIO_PORT		GPIO_PORT_1
#define CHG_STAT_GPIO_PIN		GPIO_PIN_1
#define CHG_STAT_GPIO_IRQ		GPIO4_IRQn
#define CHG_STAT_RESERVE		RESERVE_GPIO(CHG_STAT, CHG_STAT_GPIO_PORT, CHG_STAT_GPIO_PIN, PID_GPIO)
#define CHG_STAT_CONFIG 		GPIO_ConfigurePin(CHG_STAT_GPIO_PORT, CHG_STAT_GPIO_PIN, INPUT, PID_GPIO, true)
#define CHG_STAT_GPIO_GET		GPIO_GetPinStatus(CHG_STAT_GPIO_PORT, CHG_STAT_GPIO_PIN)

#define LDO_P3V3_GPIO_PORT		GPIO_PORT_0
#define LDO_P3V3_GPIO_PIN		GPIO_PIN_2
#define LDO_P3V3_RESERVE		RESERVE_GPIO(LDO_P3V3, LDO_P3V3_GPIO_PORT, LDO_P3V3_GPIO_PIN, PID_GPIO)
#define LDO_P3V3_CONFIG 		GPIO_ConfigurePin(LDO_P3V3_GPIO_PORT, LDO_P3V3_GPIO_PIN, OUTPUT, PID_GPIO, true)
#define LDO_P3V3_OPEN			GPIO_SetActive(LDO_P3V3_GPIO_PORT, LDO_P3V3_GPIO_PIN)
#define LDO_P3V3_CLOSE			GPIO_SetInactive(LDO_P3V3_GPIO_PORT, LDO_P3V3_GPIO_PIN)

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

#if 0
#define UART1_RX_GPIO_PORT		GPIO_PORT_0
#define UART1_RX_GPIO_PIN		GPIO_PIN_1

#define UART1_RTSN_GPIO_PORT	GPIO_PORT_0
#define UART1_RTSN_GPIO_PIN		GPIO_PIN_6

#define UART1_CTSN_GPIO_PORT	GPIO_PORT_0
#define UART1_CTSN_GPIO_PIN		GPIO_PIN_7
#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

struct jwaoo_pwm_device
{
	uint8_t pwm;
	GPIO_PORT port;
	GPIO_PIN pin;
	bool active_low;
	uint16_t blink_timer;

	uint8_t level;
	bool blink_add;
	uint8_t blink_step;
	uint8_t blink_min;
	uint8_t blink_max;
	uint8_t blink_delay;
	uint8_t blink_count;

	void (*set_level)(struct jwaoo_pwm_device *device, uint8_t level);
};

extern struct jwaoo_pwm_device jwaoo_pwm_moto;
extern struct jwaoo_pwm_device jwaoo_pwm_led1;
extern struct jwaoo_pwm_device jwaoo_pwm_led2;

bool jwaoo_pwm_blink_walk(struct jwaoo_pwm_device *device);
void jwaoo_pwm_blink_set(struct jwaoo_pwm_device *device, uint8_t min, uint8_t max, uint8_t step, uint8_t delay, uint8_t count);
void jwaoo_pwm_blink_sawtooth(struct jwaoo_pwm_device *device, uint8_t min, uint8_t max, uint8_t step, uint32_t cycle, uint8_t count);
void jwaoo_pwm_blink_square(struct jwaoo_pwm_device *device, uint8_t min, uint8_t max, uint32_t cycle, uint8_t count);

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

// ================================================================================

static inline void jwaoo_led_blink_sawtooth(struct jwaoo_pwm_device *device, uint32_t cycle, uint8_t count)
{
	jwaoo_pwm_blink_sawtooth(device, 0, PWM_LEVEL_MAX, PWM_LEVEL_MAX / 10, cycle, count);
}

static inline void jwaoo_led_blink_square(struct jwaoo_pwm_device *device, uint32_t cycle, uint8_t count)
{
	jwaoo_pwm_blink_square(device, 0, PWM_LEVEL_MAX, cycle, count);
}

static inline void jwaoo_led_set_brightness(struct jwaoo_pwm_device *device, uint8_t brightness)
{
	jwaoo_pwm_blink_set(device, brightness, brightness, 0, 0, 0);
}

static inline void jwaoo_led_open(struct jwaoo_pwm_device *device)
{
	jwaoo_led_set_brightness(device, PWM_LEVEL_MAX);
}

static inline void jwaoo_led_close(struct jwaoo_pwm_device *device)
{
	jwaoo_led_set_brightness(device, 0);
}

// ================================================================================

static inline void jwaoo_moto_blink_sawtooth(uint16_t cycle)
{
	jwaoo_pwm_blink_sawtooth(&jwaoo_pwm_moto, 1, MOTO_SPEED_MAX, 1, cycle, 0);
}

static inline void jwaoo_moto_blink_square(uint16_t cycle)
{
	jwaoo_pwm_blink_square(&jwaoo_pwm_moto, 0, MOTO_SPEED_MAX, cycle, 0);
}

static inline void jwaoo_moto_set_speed(uint8_t speed)
{
	jwaoo_pwm_blink_set(&jwaoo_pwm_moto, speed, speed, 0, 0, 0);
}

static inline void jwaoo_moto_open(void)
{
	jwaoo_moto_set_speed(MOTO_SPEED_MAX);
}

static inline void jwaoo_moto_close(void)
{
	jwaoo_moto_set_speed(0);
}

#endif // _USER_PERIPH_SETUP_H_
