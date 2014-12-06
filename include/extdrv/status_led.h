/****************************************************************************
 *  extdrv/status_led.h
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

#ifndef EXTDRV_STATUS_LED_H
#define EXTDRV_STATUS_LED_H


#include <stdint.h>


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


#endif /* EXTDRV_STATUS_LED_H */
