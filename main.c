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
#include "drivers/i2c.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"

#define SELECTED_FREQ  FREQ_SEL_36MHz

#if 0
char* UARTBUFFER = (char*)0x10000000;
volatile int UARTPTR = 0;
volatile int UARTSENDPTR = 0;
volatile int UARTSENDING = 0;
#endif

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
}

/***************************************************************************** */
/* Temperature */
/* The Temperature Alert pin is on GPIO Port 0, pin 7 (PIO0_7) */
/* The I2C Temperature sensor is at address */
void WAKEUP_Handler(void)
{
}
void Temp_config(void)
{
	struct lpc_io_control* ioctrl = LPC_IO_CONTROL;
	/* Temp Alert */
	config_gpio(&ioctrl->pio0_7, (LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP));
}


/***************************************************************************** */
/* EEPROM */



/***************************************************************************** */
/* GPIO */




/***************************************************************************** */
int main(void) {
	struct lpc_uart* uart0 = LPC_UART_0;
	struct lpc_uart* uart1 = LPC_UART_1;

	system_init();
	uart_on(0, 115200);
	uart_on(1, 115200);

	uart0->func.buffer = 'A';
	uart1->func.buffer = 'Z';

	status_led(green_on);
	while (1) {
		chenillard(250);
	}
	return 0;
}




