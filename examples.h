/****************************************************************************
 *   examples.h
 *
 *
 *
 * Copyright 2013 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef EXAMPLE_H
#define EXAMPLE_H

/* This file holds all the functions specific to external devices used during tests
 * of the GPIO Demo module.
 */

#include <stdint.h>

#ifdef DEBUG
#define debug(uart_num, c) \
	do { \
		struct lpc_uart* uart = LPC_UART_ ## uart_num; \
		uart->func.buffer = c; \
	} while (0);
#else
#define debug(uart, c)
#endif


/***************************************************************************** */
/* Temperature */
/* The Temperature Alert pin is on GPIO Port 0, pin 7 (PIO0_7) */
/* The I2C Temperature sensor is at address */
void temp_config(void);

void temp_display(void);



/***************************************************************************** */
/* DHT11 Humidity and temp sensor */
void TH_config(uint8_t pin_num);

unsigned char read_dht11_dat();

void TH_display(void);



/***************************************************************************** */
/* EEPROM */
void module_desc_dump(uint8_t serial_num);

int module_desc_set(char* name, uint8_t module_version, uint16_t serial_num);



/***************************************************************************** */
void luminosity_display(int adc_num);

void TMP36_display(int adc_num);


/***************************************************************************** */
/* RGB Led */
void RGB_Led_config(uint8_t red_pin, uint8_t green_pin, uint8_t blue_pin);


/***************************************************************************** */
/* Maxim's MAX31855 themocouple to digital converter */
uint16_t Thermocouple_Read(void);

#endif /* EXAMPLE_H */

