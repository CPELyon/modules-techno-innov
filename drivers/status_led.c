/****************************************************************************
 *  drivers/status_led.c
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
/*                Status Led                                                   */
/***************************************************************************** */


#include <stdint.h>
#include "core/lpc_regs_12xx.h"
#include "core/system.h"
#include "core/pio.h"
#include "drivers/status_led.h"


/***************************************************************************** */
/* Status LED is the bicolors red/green led on the GPIO Demo module */

/* The status LED is on GPIO Port 1, pin 4 (PIO1_4) and Port 1, pin 5 (PIO1_5) */
#define LED_RED    5
#define LED_GREEN  4

void status_led_config(void)
{
	struct lpc_gpio* gpio1 = LPC_GPIO_1;
	uint32_t mode = (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL | LPC_IO_DRIVE_HIGHCURENT);
	struct pio red_led = LPC_GPIO_1_4;
	struct pio green_led = LPC_GPIO_1_5;
	/* Status Led GPIO */
	config_pio(&green_led, mode);
	config_pio(&red_led, mode);
	/* Configure both as output */
	gpio1->data_dir |= (1 << LED_GREEN) | (1 << LED_RED);
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
			gpio1->toggle = (1 << LED_RED);
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
			gpio1->toggle = (1 << LED_GREEN);
			break;
		case both:
			gpio1->set = (1 << LED_GREEN) | (1 << LED_RED);
			break;
		case toggle:
			gpio1->toggle = (1 << LED_GREEN) | (1 << LED_RED);
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