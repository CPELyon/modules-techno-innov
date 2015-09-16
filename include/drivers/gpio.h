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

/***************************************************************************** */
/*                GPIOs and GPIO Interrupts                                    */
/***************************************************************************** */

/* Driver for GPIO configuration and access (including GPIO interrupts) on the LPC1224.
 * Refer to LPC1224 documentation (UM10441.pdf) for more information.
 */

#ifndef DRIVERS_GPIO_H
#define DRIVERS_GPIO_H


#include <stdint.h>
#include "core/pio.h"


/***************************************************************************** */
/*   Public access to GPIO setup   */


enum gpio_interrupt_senses {
	EDGES_BOTH = 0,
	EDGE_FALLING,
	EDGE_RISING,
	LEVEL_HIGH,
	LEVEL_LOW,
};

/* GPIO Interrupts */
/* Add a callback on a GPIO interrupt.
 * This call will configure the GPIO (call to config_pio()), set it as input and
 * activate the interrupt on the given 'sense' event. use the gpio_interrupt_senses
 * enum for 'sense' values.
 * The callback will receive the pin number as argument (but not the port number).
 * Note :
 *   The interrupt hanlers are not efficient if the pin is not a low numbered one (require
 *   32 shifts + test for pin number 31).
 *   Use them if you place the signals generating interrupts on low numbered pins.
 *   When possible, get in touch with the people doing the electronic design, or change
 *   the handlers in drivers/gpio.c
 */
int set_gpio_callback(void (*callback) (uint32_t), const struct pio* gpio, uint8_t sense);
void remove_gpio_callback(const struct pio* gpio);



/* GPIO Activation */
void gpio_on(void);
void gpio_off(void);

#define gpio_dir_in(gpio) \
{ \
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio.port); \
	gpio_port->data_dir &= ~(1 << gpio.pin);\
}

#define gpio_dir_out(gpio) \
{ \
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio.port); \
	gpio_port->data_dir |= (1 << gpio.pin);\
}


#define gpio_set(gpio) \
{ \
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio.port); \
	gpio_port->set = (1 << gpio.pin);\
}

#define gpio_clear(gpio) \
{ \
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio.port); \
	gpio_port->clear = (1 << gpio.pin);\
}

#define gpio_toggle(gpio) \
{ \
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio.port); \
	gpio_port->toggle = (1 << gpio.pin);\
}


/* GPIO Configuration
 * This function calls the config_pio() function for the gpio with the given
 * mode, configures the direction of the pin and sets the initial state.
 * Use GPIO_DIR_IN or GPIO_DIR_OUT for the direction "dir", and 0 or 1 for the initial
 * value "ini_val".
 */
void config_gpio(const struct pio* gpio, uint32_t mode, uint8_t dir, uint8_t ini_val);


#endif /* DRIVERS_GPIO_H */
