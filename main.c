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


#ifndef MODULE_SERIAL_NUM
#define MODULE_SERIAL_NUM 1
#define MODULE_VERSION    0x02
#define MODULE_NAME "GPIO Demo Module"
#endif

#define SELECTED_FREQ  FREQ_SEL_24MHz

#ifdef DEBUG
#define debug(uart_num, c) \
	do { \
		struct lpc_uart* uart = LPC_UART_ ## uart_num; \
		uart->func.buffer = c; \
	} while (0);
#else
#define debug(uart, c)
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

void temp_config(void)
{
	int ret = 0;

	/* Temp Alert */
	config_gpio(0, 7, (LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP));

	/* Temp sensor */
	ret = sensor_config(TMP_RES_ELEVEN_BITS);
	if (ret != 0) {
		serial_write(1, "Temp config error\r\n", 19);
	}
}

void temp_display(void)
{
	char buff[40];
	uint16_t raw = 0;
	int deci_degrees = 0;
	int len = 0;

	sensor_start_conversion();
	msleep(250); /* Wait for the end of the conversion : 40ms */
	len = temp_read(&raw, &deci_degrees);
	if (len != 0) {
		serial_write(1, "Temp read error\r\n", 19);
	} else {
		len = snprintf(buff, 40, "Temp read: %d,%d - raw: 0x%04x.\r\n",
				(deci_degrees/10), (deci_degrees%10), raw);
		serial_write(1, buff, len);
	}
}

/***************************************************************************** */
/* EEPROM */
#define DUMP_BUFF_SIZE 80
void module_desc_dump(void)
{
	char buff[DUMP_BUFF_SIZE];
	int len = 0, ret = 0;
	struct module_desc desc;

	/* Read module descriptor structure from eeprom */
	ret = eeprom_read(0, (char*)&desc, sizeof(struct module_desc));
	if (ret != sizeof(struct module_desc)) {
		return;
	}
	/* Send the content of the header */
	serial_write(1, "Module :\r\n", 10);
	len = snprintf(buff, 16, "serial: %d, ", desc.serial_number);
	len += snprintf((buff + len), 15, "ver: %d\r\n", desc.version);
	len += snprintf((buff + len), 16, "cap: 0x%04x\r\n", desc.capabilities);
	/* Get and send the module name */
	if (desc.name_size >= DUMP_BUFF_SIZE) {
		desc.name_size = DUMP_BUFF_SIZE - len - 3;
	}
	ret = eeprom_read(desc.name_offset, (buff + len), desc.name_size);
	if (ret == desc.name_size) {
		len += ret;
		len += snprintf((buff + len), 3, "\r\n");
	}
	ret = 0;
	do {
		ret += serial_write(1, (buff + ret), (len - ret));
	} while (ret < len);
}

int module_desc_set(char* name)
{
	int ret = 0;
	struct module_desc desc = {
		.serial_number = MODULE_SERIAL_NUM,
		.version = MODULE_VERSION,
		.capabilities = (UEXT_MOD_HAS_UART | UEXT_MOD_HAS_I2C | UEXT_MOD_HAS_SPI),
		.name_offset = sizeof(struct module_desc),
		.image_offset = 0,
		.image_size = 0,
	};
	desc.name_size = strlen(name);
	ret = eeprom_write(0, (char*)&desc, sizeof(struct module_desc));
	if (ret != sizeof(struct module_desc)) {
		return -1;
	}
	ret += eeprom_write(ret, name, desc.name_size);
	return ret;
}

/***************************************************************************** */
/* Set SPI Chip Select Low for I2C */
/* Temporary hack, should be shared with SPI */
void set_spi_cs_low(void)
{
	struct lpc_gpio* gpio0 = LPC_GPIO_0;

	config_gpio(0, 15, (LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL));

	/* Configure SPI_CS as output and set it low. */
	gpio0->data_dir |= (1 << 15);
	gpio0->clear = (1 << 15);
}

/***************************************************************************** */
void luminosity_display(int adc_num)
{
	uint16_t val = 0;
	int ret = 0;

	adc_start_convertion_once(adc_num, 0);
	msleep(5);
	ret = adc_get_value(&val, adc_num);
	if (ret == 0) {
		debug(0, 'n');
	} else {
		char buff[40];
		int len = 0;
		len = snprintf(buff, 40, "L(%d): %d (raw: %04x)\r\n", adc_num, val, val);
		serial_write(1, buff, len);
		serial_write(0, buff, len);
	}
}

void TMP36_display(int adc_num)
{
	uint16_t val = 0;
	int ret = 0;

	adc_start_convertion_once(adc_num, 0);
	msleep(5);
	ret = adc_get_value(&val, adc_num);
	if (ret == 0) {
		debug(0, 'm');
	} else {
		char buff[60];
		int len = 0;
		int micro_volts = 0;
		/* depends on vref, should use a precise 3.0V Vref and multiply by 3000 */
		micro_volts = (val * 3200);
		int converted = ((micro_volts / 100) - 5000);
		len = snprintf(buff, 60, "TMP36: %d,%d (orig: %d, raw: %04x)\r\n",
						(converted / 100), (converted % 100), val, val);
		serial_write(1, buff, len);
		serial_write(0, buff, len);
	}
}


/***************************************************************************** */
int main(void) {
	system_init();
	uart_on(0, 115200);
	uart_on(1, 115200);
	adc_on();

	set_spi_cs_low();
	i2c_on(I2C_CLK_100KHz);

	/* Set or read Module identification header in EEPROM */
#ifdef EEPROM_WRITE
	if (module_desc_set(MODULE_NAME) <= 0) {
		debug(0, 'E');
	} else {
		debug(0, 'O');
	}
#else
	module_desc_dump();
#endif

	temp_config();
	temp_display();

	while (1) {
		chenillard(250);
		luminosity_display(1);
		TMP36_display(0);
		temp_display();
	}
	return 0;
}




