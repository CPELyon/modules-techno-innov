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
/* ADC Examples */

/* This will display the integer value read on the ADC, between 0 and 1024.
 * ADC must be initialised prior to calls to voltage_to_position() (it means that
 *    adc_on() must be called before using this function.
 * adc_num is an ADC channel number (integer between 0 and 7)
 *  use LPC_ADC_NUM(x) for channel selection.
 * returns ADC convertion value or negative value on error.
 */
int adc_display(int adc_num);

/* Display the temperature computed from adc convertion of the voltage output of
 * a TMP36 analog temperature sensor
 * ADC must be initialised prior to calls to voltage_to_position() (it means that
 *    adc_on() must be called before using this function.
 * adc_num is an ADC channel number (integer between 0 and 7)
 *  use LPC_ADC_NUM(x) for channel selection.
 */
void TMP36_display(int adc_num);


/***************************************************************************** */
/* RGB Led */
void RGB_Led_config(uint8_t red_pin, uint8_t green_pin, uint8_t blue_pin);


/***************************************************************************** */
/* Maxim's MAX31855 themocouple to digital converter */
uint16_t Thermocouple_Read(uint8_t slave_sel_pin);



/***************************************************************************** */
/* Simple GPIO interrupt example: toggle a led when you push the switch.
 * Connect a switch and a pull down resistor to "INT_PIN" (connect the other end of
 *   the switch to +Vcc), and a Led to "LED_PIN" (You can add a resistor to limit the
 *   current in the Led).
 * Add a 10uF capacitor between "INT_PIN" and ground to prevent rebounds and thus
 *   multiple interrupts for each push of the switch.
 * Note: should also be used without the capacitor to test the input filter ?
 */
void gpio_intr_toggle_config(void);

#endif /* EXAMPLE_H */

