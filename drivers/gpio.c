/****************************************************************************
	*  drivers/gpio.c
	*
	* Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
	*
	* This program is free software: you can redistribute it and/or modify
	* it under the terms of the GNU General Public License as published by
	* the Free Software Foundation, either version 2 of the License, or
	* (at your option) any later version.
	*
	* This program is distributed in the hope that it will be useful,
	* but WITHOUT ANY WARRANTY; without even the implied warranty of
	* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	* GNU General Public License for more details.
	*
	* You should have received a copy of the GNU General Public License
	* along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*
	*************************************************************************** */

/***************************************************************************** */
/*                GPIOs                                                        */
/***************************************************************************** */



#include <stdint.h>
#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"
#include "core/system.h"
#include "drivers/gpio.h"


/***************************************************************************** */
/*   Public access to GPIO setup   */
#define PORT0_NB_PINS 32
#define PORT1_NB_PINS 7
#define PORT2_NB_PINS 16
static volatile uint32_t* gpio_regs_handles_port0[PORT0_NB_PINS] = {
	&(LPC_IO_CONTROL->pio0_0),
	&(LPC_IO_CONTROL->pio0_1),
	&(LPC_IO_CONTROL->pio0_2),
	&(LPC_IO_CONTROL->pio0_3),
	&(LPC_IO_CONTROL->pio0_4),
	&(LPC_IO_CONTROL->pio0_5),
	&(LPC_IO_CONTROL->pio0_6),
	&(LPC_IO_CONTROL->pio0_7),
	&(LPC_IO_CONTROL->pio0_8),
	&(LPC_IO_CONTROL->pio0_9),
	&(LPC_IO_CONTROL->pio0_10),
	&(LPC_IO_CONTROL->pio0_11),
	&(LPC_IO_CONTROL->pio0_12),
	&(LPC_IO_CONTROL->pio0_13),
	&(LPC_IO_CONTROL->pio0_14),
	&(LPC_IO_CONTROL->pio0_15),
	&(LPC_IO_CONTROL->pio0_16),
	&(LPC_IO_CONTROL->pio0_17),
	&(LPC_IO_CONTROL->pio0_18),
	&(LPC_IO_CONTROL->pio0_19),
	&(LPC_IO_CONTROL->pio0_20),
	&(LPC_IO_CONTROL->pio0_21),
	&(LPC_IO_CONTROL->pio0_22),
	&(LPC_IO_CONTROL->pio0_23),
	&(LPC_IO_CONTROL->pio0_24),
	&(LPC_IO_CONTROL->pio0_25),
	&(LPC_IO_CONTROL->pio0_26),
	&(LPC_IO_CONTROL->pio0_27),
	&(LPC_IO_CONTROL->pio0_28),
	&(LPC_IO_CONTROL->pio0_29),
	&(LPC_IO_CONTROL->pio0_30),
	&(LPC_IO_CONTROL->pio0_31),
};
static volatile uint32_t* gpio_regs_handles_port1[PORT1_NB_PINS] = {
	&(LPC_IO_CONTROL->pio1_0),
	&(LPC_IO_CONTROL->pio1_1),
	&(LPC_IO_CONTROL->pio1_2),
	&(LPC_IO_CONTROL->pio1_3),
	&(LPC_IO_CONTROL->pio1_4),
	&(LPC_IO_CONTROL->pio1_5),
	&(LPC_IO_CONTROL->pio1_6),
};
static volatile uint32_t* gpio_regs_handles_port2[PORT2_NB_PINS] = {
	&(LPC_IO_CONTROL->pio2_0),
	&(LPC_IO_CONTROL->pio2_1),
	&(LPC_IO_CONTROL->pio2_2),
	&(LPC_IO_CONTROL->pio2_3),
	&(LPC_IO_CONTROL->pio2_4),
	&(LPC_IO_CONTROL->pio2_5),
	&(LPC_IO_CONTROL->pio2_6),
	&(LPC_IO_CONTROL->pio2_7),
	&(LPC_IO_CONTROL->pio2_8),
	&(LPC_IO_CONTROL->pio2_9),
	&(LPC_IO_CONTROL->pio2_10),
	&(LPC_IO_CONTROL->pio2_11),
	&(LPC_IO_CONTROL->pio2_12),
	&(LPC_IO_CONTROL->pio2_13),
	&(LPC_IO_CONTROL->pio2_14),
	&(LPC_IO_CONTROL->pio2_15),
};

void config_gpio(uint8_t port, uint8_t pin, uint32_t mode)
{
	volatile uint32_t* handle = NULL;

	switch (port) {
		case 0:
			if (pin >= PORT0_NB_PINS)
				return;
			handle = gpio_regs_handles_port0[pin];
			break;
		case 1:
			if (pin >= PORT1_NB_PINS)
				return;
			handle = gpio_regs_handles_port1[pin];
			break;
		case 2:
			if (pin >= PORT2_NB_PINS)
				return;
			handle = gpio_regs_handles_port2[pin];
			break;
		default:
			return;
	}
	/* Make sure IO_Config is clocked */
	io_config_clk_on();
	*handle = mode;
	/* Config done, power off IO_CONFIG block */
	io_config_clk_off();
}

void gpio_on(void)
{
	/* Provide power to GPIO control blocks */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_GPIO0, 1);
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_GPIO1, 1);
}
void gpio_off(void)
{
	/* Remove power from GPIO control blocks */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_GPIO0, 0);
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_GPIO1, 0);
}


/* Set all GPIO used on the GPIO_Demo module in a default state */
void set_gpio_pins(void)
{
	struct lpc_io_control* ioctrl = LPC_IO_CONTROL;

	/* Make sure IO_Config is clocked */
	io_config_clk_on();

	/* Configure GPIO pins */
	ioctrl->pio0_0 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_3 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_4 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_5 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_6 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_19 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_20 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_21 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_22 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_23 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_24 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_25 = LPC_IO_FUNC_ALT(6) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_26 = LPC_IO_FUNC_ALT(6) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_27 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_28 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	ioctrl->pio0_29 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP;
	/* ADC pins configured in ADC driver. Kept for info, for those not using them as ADC */
	ioctrl->pio0_30 = LPC_IO_FUNC_ALT(1) | LPC_IO_MODE_PULL_UP; /* AD0 */
	ioctrl->pio0_31 = LPC_IO_FUNC_ALT(1) | LPC_IO_MODE_PULL_UP; /* AD1 */
	ioctrl->pio1_0 = LPC_IO_FUNC_ALT(1) | LPC_IO_MODE_PULL_UP; /* AD2 */
	ioctrl->pio1_1 = LPC_IO_FUNC_ALT(1) | LPC_IO_MODE_PULL_UP; /* AD3 */
	ioctrl->pio1_2 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* AD4 */
	ioctrl->pio1_3 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* AD5 */
	/* Bicolor led */
	ioctrl->pio1_4 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* Green LED */
	ioctrl->pio1_5 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* Red LED */
	/* SPI pins */
	ioctrl->pio0_14 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* SPI Clock */
	ioctrl->pio0_15 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* SPI Chip Select */
	ioctrl->pio0_16 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* SPI MISO */
	ioctrl->pio0_17 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* SPI MOSI */
	/* UART pins */
	ioctrl->pio0_1 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* UART0 RxD */
	ioctrl->pio0_2 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* UART0 TxD */
	ioctrl->pio0_8 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* UART1 RxD */
	ioctrl->pio0_9 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* UART1 TxD */
	/* I2C pins */
	ioctrl->pio0_10 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* I2C SCL */
	ioctrl->pio0_11 = LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP; /* I2C SDA */

	/* Config done, power off IO_CONFIG block */
	io_config_clk_off();
}

/* Handlers */
void PIO_0_Handler(void)
{
}
void PIO_1_Handler(void)
{
}
void PIO_2_Handler(void)
{
}


/***************************************************************************** */
/* Status LED */

#define LED_RED 5
#define LED_GREEN 4

/* The status LED is on GPIO Port 1, pin 4 (PIO1_4) and Port 1, pin 5 (PIO1_5) */
void status_led_config(void)
{
	struct lpc_gpio* gpio1 = LPC_GPIO_1;
	uint32_t mode = (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL | LPC_IO_DRIVE_HIGHCURENT);
	/* Status Led GPIO */
	config_gpio(1, LED_GREEN, (LPC_IO_FUNC_ALT(0) | mode));
	config_gpio(1, LED_RED, (LPC_IO_FUNC_ALT(0) | mode));
	/* Configure both as output */
	gpio1->data_dir = (1 << LED_GREEN) | (1 << LED_RED);
	/* Turn both LEDs on */
	//gpio1->set = (1 << LED_GREEN) | (1 << LED_RED);
	gpio1->set = (1 << LED_GREEN);
}

void status_led(uint32_t val)
{
	struct lpc_gpio* gpio1 = LPC_GPIO_1;

	switch (val) {
		case red_only:
			gpio1->set = (1 << LED_RED);
			gpio1->clear = (1 << LED_GREEN);
			break;
		case red_on:
			gpio1->set = (1 << LED_RED);
			break;
		case red_off:
			gpio1->clear = (1 << LED_RED);
			break;
		case red_toggle:
			gpio1->invert = (1 << LED_RED);
			break;
		case green_only:
			gpio1->set = (1 << LED_GREEN);
			gpio1->clear = (1 << LED_RED);
			break;
		case green_on:
			gpio1->set = (1 << LED_GREEN);
			break;
		case green_off:
			gpio1->clear = (1 << LED_GREEN);
			break;
		case green_toggle:
			gpio1->invert = (1 << LED_GREEN);
			break;
		case both:
			gpio1->set = (1 << LED_GREEN) | (1 << LED_RED);
			break;
		case toggle:
			gpio1->invert = (1 << LED_GREEN) | (1 << LED_RED);
			break;
		case none:
		default:
			gpio1->clear = (1 << LED_GREEN) | (1 << LED_RED);
			break;
	}
}

void chenillard(uint32_t ms)
{
    status_led(red_only);
    msleep(ms);
    status_led(green_only);
    msleep(ms);
    status_led(none);
    msleep(ms);
    status_led(both);
    msleep(ms);
    status_led(none);
    msleep(ms);
    status_led(red_only);
    msleep(ms);
    status_led(red_only);
    msleep(ms);
    status_led(none);
    msleep(ms);
    status_led(green_only);
    msleep(ms);
    status_led(green_only);
    msleep(ms);
    status_led(none);
    msleep(ms);
}
