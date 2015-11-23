/****************************************************************************
 *   apps/adc/main.c
 *
 * ADC example
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


#include <stdint.h>
#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"
#include "core/pio.h"
#include "core/system.h"
#include "core/systick.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "extdrv/status_led.h"
#include "drivers/adc.h"


#define MODULE_VERSION    0x04
#define MODULE_NAME "GPIO Demo Module"


#define SELECTED_FREQ  FREQ_SEL_48MHz

/***************************************************************************** */
/* Pins configuration */
/* pins blocks are passed to set_pins() for pins configuration.
 * Unused pin blocks can be removed safely with the corresponding set_pins() call
 * All pins blocks may be safelly merged in a single block for single set_pins() call..
 */
const struct pio_config common_pins[] = {
	/* UART 0 */
	{ LPC_UART0_RX_PIO_0_1,  LPC_IO_DIGITAL },
	{ LPC_UART0_TX_PIO_0_2,  LPC_IO_DIGITAL },
	ARRAY_LAST_PIO,
};

const struct pio_config adc_pins[] = {
	{ LPC_ADC_AD0_PIO_0_30, LPC_IO_ANALOG },
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },
	{ LPC_ADC_AD2_PIO_1_0,  LPC_IO_ANALOG },
	{ LPC_ADC_AD3_PIO_1_1,  LPC_IO_ANALOG },
	{ LPC_ADC_AD4_PIO_1_2,  LPC_IO_ANALOG },
	{ LPC_ADC_AD5_PIO_1_3,  LPC_IO_ANALOG },
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

/***************************************************************************** */
/* This will display the integer value read on the ADC, between 0 and 1024.
 * ADC must be initialised prior to calls to adc_display() (it means that adc_on()
 *    must be called before using this function.
 * adc_num is an ADC channel number (integer between 0 and 7)
 *    use LPC_ADC_NUM(x) for channel selection.
 * returns ADC convertion value or negative value on error.
 */
int adc_display(int adc_num, int uart_num)
{
	uint16_t val = 0;
	int ret = 0;

	adc_start_convertion_once(adc_num, 0);
	msleep(10);
	ret = adc_get_value(&val, adc_num);
	if (ret < 0) {
		return ret;
	} else {
		uprintf(uart_num, "ADC(%d): %d (raw: 0x%04x)\r\n", adc_num, val, val);
	}
	return val;
}

/* Display the temperature computed from adc convertion of the voltage output of
 * a TMP36 analog temperature sensor
 * ADC must be initialised prior to calls to TMP36_display() (it means that adc_on()
 *    must be called before using this function.
 * adc_num is an ADC channel number (integer between 0 and 7)
 *    use LPC_ADC_NUM(x) for channel selection.
 */
void TMP36_display(int adc_num, int uart_num)
{
	uint16_t val = 0;
	int ret = 0;

	adc_start_convertion_once(adc_num, 0);
	msleep(8);
	ret = adc_get_value(&val, adc_num);
	if (ret == 0) {
		int micro_volts = 0;
		/* depends on vref, should use a precise 3.0V Vref and multiply by 3000 */
		micro_volts = (val * 3200);
		int converted = ((micro_volts / 100) - 5000);
		uprintf(uart_num, "TMP36: %d,%d (orig: %d, raw: %04x)\r\n",
						(converted / 100), (converted % 100), val, val);
	}
}


/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */

	/* Note: Brown-Out detection must be powered to operate the ADC. adc_on() will power
	 *  it back on if called after system_init() */
	system_brown_out_detection_config(0);
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	set_pins(adc_pins);
	gpio_on();
	status_led_config(&status_led_green, &status_led_red);
	/* System tick timer MUST be configured and running in order to use the sleeping
	 * functions */
	systick_timer_on(1); /* 1ms */
	systick_start();
}

/* Define our fault handler. This one is not mandatory, the dummy fault handler
 * will be used when it's not overridden here.
 * Note : The default one does a simple infinite loop. If the watchdog is deactivated
 * the system will hang.
 */
void fault_info(const char* name, uint32_t len)
{
	serial_write(0, name, len);
	/* Wait for end of Tx */
	serial_flush(0);
	/* FIXME : Perform soft reset of the micro-controller ! */
	while (1);
}



/***************************************************************************** */
int main(void) {
	system_init();
	uart_on(0, 115200, NULL);
	adc_on();

	while (1) {
		chenillard(500);
		/* ADC Test */
		adc_display(LPC_ADC_NUM(0), 0);
		TMP36_display(LPC_ADC_NUM(1), 0);
	}
	return 0;
}



