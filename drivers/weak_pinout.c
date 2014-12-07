/****************************************************************************
 *   driver/weak_pinout.c
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
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


#include "core/pio.h"

/***************************************************************************** */
/* Pins configuration */
/* List of available pin blocks :
 *  clkout_pin, uart0_pins, uart1_pins, i2c0_pins, ssp0_pins,
 *  timer0_pins, timer1_pins, timer2_pins, timer3_pins,
 *  adc_pins, gpio_pins
 *
 * This files defines "weak" "empty" pio structures for all of the drivers so
 *   that the drivers can compile even if the user does not define empty pio
 *   structures for unused drivers.
 *
 * These will be overridden by the user definitions.
 *
 */
const struct pio uart0_pins[] __attribute__ ((weak)) = {
	ARRAY_LAST_PIN,
};
const struct pio uart1_pins[] __attribute__ ((weak)) = {
	ARRAY_LAST_PIN,
};
const struct pio i2c0_pins[] __attribute__ ((weak)) = {
	ARRAY_LAST_PIN,
};
const struct pio ssp0_pins[] __attribute__ ((weak)) = {
	ARRAY_LAST_PIN,
};
const struct pio timer0_pins[] __attribute__ ((weak)) = {
	ARRAY_LAST_PIN,
};
const struct pio timer1_pins[] __attribute__ ((weak)) = {
	ARRAY_LAST_PIN,
};
const struct pio timer2_pins[] __attribute__ ((weak)) = {
	ARRAY_LAST_PIN,
};
const struct pio timer3_pins[] __attribute__ ((weak)) = {
	ARRAY_LAST_PIN,
};
const struct pio adc_pins[] __attribute__ ((weak)) = {
	ARRAY_LAST_PIN,
};
const struct pio gpio_pins[] __attribute__ ((weak)) = {
	ARRAY_LAST_PIN,
};
