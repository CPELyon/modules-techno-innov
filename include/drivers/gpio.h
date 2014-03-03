/****************************************************************************
 *  drivers/gpio.h
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

#ifndef DRIVERS_GPIO_H
#define DRIVERS_GPIO_H


#include <stdint.h>


/***************************************************************************** */
/*   Public access to GPIO setup   */


void config_gpio(uint8_t port, uint8_t pin, uint32_t mode);

enum gpio_interrupt_senses {
	EDGES_BOTH = 0,
	EDGE_FALLING,
	EDGE_RISING,
	LEVEL_HIGH,
	LEVEL_LOW,
};

#define LPC_IO_FUNC_GPIO_0_0    0
#define LPC_IO_FUNC_GPIO_0_1    0
#define LPC_IO_FUNC_GPIO_0_2    0
#define LPC_IO_FUNC_GPIO_0_3    0
#define LPC_IO_FUNC_GPIO_0_4    0
#define LPC_IO_FUNC_GPIO_0_5    0
#define LPC_IO_FUNC_GPIO_0_6    0
#define LPC_IO_FUNC_GPIO_0_7    0
#define LPC_IO_FUNC_GPIO_0_8    0
#define LPC_IO_FUNC_GPIO_0_9    0
#define LPC_IO_FUNC_GPIO_0_10   0
#define LPC_IO_FUNC_GPIO_0_11   0
#define LPC_IO_FUNC_GPIO_0_12   0
#define LPC_IO_FUNC_GPIO_0_13   1
#define LPC_IO_FUNC_GPIO_0_14   0
#define LPC_IO_FUNC_GPIO_0_15   0
#define LPC_IO_FUNC_GPIO_0_16   0
#define LPC_IO_FUNC_GPIO_0_17   0
#define LPC_IO_FUNC_GPIO_0_18   0
#define LPC_IO_FUNC_GPIO_0_19   0
#define LPC_IO_FUNC_GPIO_0_20   0
#define LPC_IO_FUNC_GPIO_0_21   0
#define LPC_IO_FUNC_GPIO_0_22   0
#define LPC_IO_FUNC_GPIO_0_23   0
#define LPC_IO_FUNC_GPIO_0_24   0
#define LPC_IO_FUNC_GPIO_0_25   6
#define LPC_IO_FUNC_GPIO_0_26   6
#define LPC_IO_FUNC_GPIO_0_27   0
#define LPC_IO_FUNC_GPIO_0_28   0
#define LPC_IO_FUNC_GPIO_0_29   0
#define LPC_IO_FUNC_GPIO_0_30   1
#define LPC_IO_FUNC_GPIO_0_31   1

#define LPC_IO_FUNC_GPIO_1_0   1
#define LPC_IO_FUNC_GPIO_1_1   1
#define LPC_IO_FUNC_GPIO_1_2   0
#define LPC_IO_FUNC_GPIO_1_3   0
#define LPC_IO_FUNC_GPIO_1_4   0
#define LPC_IO_FUNC_GPIO_1_5   0
#define LPC_IO_FUNC_GPIO_1_6   0

#define LPC_IO_FUNC_GPIO_2_0    0
#define LPC_IO_FUNC_GPIO_2_1    0
#define LPC_IO_FUNC_GPIO_2_2    0
#define LPC_IO_FUNC_GPIO_2_3    0
#define LPC_IO_FUNC_GPIO_2_4    0
#define LPC_IO_FUNC_GPIO_2_5    0
#define LPC_IO_FUNC_GPIO_2_6    0
#define LPC_IO_FUNC_GPIO_2_7    0
#define LPC_IO_FUNC_GPIO_2_8    0
#define LPC_IO_FUNC_GPIO_2_9    0
#define LPC_IO_FUNC_GPIO_2_10   0
#define LPC_IO_FUNC_GPIO_2_11   0
#define LPC_IO_FUNC_GPIO_2_12   0
#define LPC_IO_FUNC_GPIO_2_13   0
#define LPC_IO_FUNC_GPIO_2_14   0
#define LPC_IO_FUNC_GPIO_2_15   0

#define LPC_IO_FUNC_GPIO(port, pin)  LPC_IO_FUNC_GPIO_ ## port ## _ ## pin


int set_gpio_callback(void (*callback) (uint32_t), uint8_t port, uint8_t pin, uint8_t sense);
void remove_gpio_callback(uint8_t port, uint8_t pin);

void gpio_on(void);
void gpio_off(void);

/***************************************************************************** */
/* Status LED */
/* The status LED is on GPIO Port 1, pin 4 (PIO1_4) and Port 1, pin 5 (PIO1_5) */
void status_led_config(void);

void status_led(uint32_t val);

void chenillard(uint32_t ms);

enum led_status {
	none = 0,
	red_only,
	red_on,
	red_off,
	red_toggle,
	green_only,
	green_on,
	green_off,
	green_toggle,
	both,
	toggle,
};


#endif /* DRIVERS_GPIO_H */
