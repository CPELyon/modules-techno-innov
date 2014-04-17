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
void dth11_config(struct pio* gpio);

unsigned char dht11_read_dat();

void dth11_display(void);



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
void RGB_Led_config(uint8_t timer);


/***************************************************************************** */
/* Maxim's MAX31855 themocouple to digital converter */
uint16_t Thermocouple_Read(struct pio* spi_cs);



/***************************************************************************** */
/* Simple GPIO interrupt example: toggle a led when you push the switch.
 * Connect a switch and a pull down resistor to "INT_PIN" (connect the other end of
 *   the switch to +Vcc), and a Led to "LED_PIN" (You can add a resistor to limit the
 *   current in the Led).
 * Add a 10uF capacitor between "INT_PIN" and ground to prevent rebounds and thus
 *   multiple interrupts for each push of the switch.
 * Note: should also be used without the capacitor to test the input filter ?
 */
void gpio_intr_toggle_config(struct pio* irq_gpio, struct pio* led);



/***************************************************************************** */
/* Servo motor position control.
 * Actually only one channel is supported.
 * Timer must be initialised prior to calls to voltage_to_position() (it means that
 *    timer_on(timer_num, 0) must be called before using this function.
 * Parameters :
 *  - timer : one of LPC_TIMER_32B0, LPC_TIMER_32B1, LPC_TIMER_16B0 or LPC_TIMER_16B1
 *  - pwm_pin : pin number for PWM output. It must be the match 0 pin, and must be located
 *     on port 0.
 * The channel for the timer channel must be configured in the appropriate timer pins
 *    declaration table.
 */
void voltage_to_position_config(uint8_t timer, uint8_t channel);

/* Change the angle of the servo on the selected channel.
 *  - timer : the timer used for voltage_to_position_config()
 *  - channel : channel used for voltage_to_position_config()
 *  - angle : between 0 and 180 : servo angle in degrees.
 */
void pwm_update(uint8_t timer, uint8_t channel, uint8_t val);

#endif /* EXAMPLE_H */

