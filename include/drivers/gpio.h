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


int set_gpio_callback(void (*callback) (uint32_t), struct pio* gpio, uint8_t sense);
void remove_gpio_callback(struct pio* gpio);

void gpio_on(void);
void gpio_off(void);


#endif /* DRIVERS_GPIO_H */
