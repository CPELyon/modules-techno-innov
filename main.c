/****************************************************************************
 *   main.c
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


#include <stdint.h>
#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"
#include "core/system.h"
#include "lib/stdio.h"
#include "drivers/i2c.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/temp.h"
#include "drivers/adc.h"
#include "drivers/timers.h"
#include "drivers/ssp.h"

#include "examples.h"


#ifndef MODULE_SERIAL_NUM
#define MODULE_SERIAL_NUM 10
#endif
#define MODULE_VERSION    0x04
#define MODULE_NAME "GPIO Demo Module"


#define SELECTED_FREQ  FREQ_SEL_24MHz

void system_init()
{
	/* Stop the watchdog */
	stop_watchdog(); /* Do it right now, before it gets a chance to break in */

	system_brown_out_detection_config(0);
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	gpio_on();
	status_led_config();
	system_set_default_pins();
	systick_timer_on(1);
}


#define LED_RGB_RED   23
#define LED_RGB_GREEN 24
#define LED_RGB_BLUE  25


/***************************************************************************** */
int main(void) {
	system_init();
	uart_on(0, 115200);
	uart_on(1, 115200);
	adc_on();
	timer_on(LPC_TIMER_32B1, 0);
	ssp_master_on(LPC_SSP_FRAME_SPI, 8, 8*1000*1000); /* frame_type, data_width, rate */

	i2c_on(I2C_CLK_100KHz);

	/* Set or read Module identification header in EEPROM */
#ifdef EEPROM_WRITE
	if (module_desc_set(MODULE_NAME, MODULE_VERSION, MODULE_SERIAL_NUM) <= 0) {
		debug(0, 'E');
	} else {
		debug(0, 'O');
	}
#else
	module_desc_dump(1);
#endif

	/* Configure the DHT11 and the onboard temp sensor */
	TH_config(0);
	temp_config();

	/* GPIO interrupt test */
	gpio_intr_toggle_config();

	RGB_Led_config(LED_RGB_RED, LED_RGB_GREEN, LED_RGB_BLUE);

	while (1) {
		chenillard(250);
		luminosity_display(1);
		/* TH_display(); */
		TMP36_display(0);
		Thermocouple_Read(SPI_CS_PIN); /* SPI_CS_PIN is defined in spi.h (required for SPI */
		temp_display();
	}
	return 0;
}




