/****************************************************************************
 *   examples.c
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


/* This file holds all the functions specific to external devices used during tests
 * of the GPIO Demo module.
 */

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
/* DHT11 Humidity and temp sensor */
static uint8_t th_pin_num = 0;
void TH_config(uint8_t pin_num)
{
	struct lpc_gpio* gpio0 = LPC_GPIO_0;

	if (pin_num > 31)
		return;

	config_gpio(0, th_pin_num, (LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL));

	/* Configure as output and set it low. */
	/* This is the "do nothing" state */
	gpio0->data_dir |= (1 << th_pin_num);
	gpio0->set = (1 << th_pin_num);
}

unsigned char read_dht11_dat()
{
	struct lpc_gpio* gpio0 = LPC_GPIO_0;
	int i = 0;
	unsigned char val = 0;
	for (i = 0; i < 8; i++) {
		/* Wait end of 'low' */
		while(!(gpio0->in & (1 << th_pin_num))) {
			nop();
		}
		/* Wait 30ms */
		usleep(35);

		/* read one bit */
		if (gpio0->in & (1 << th_pin_num)) {
			val |= (1 << (7-i));
		}

		/* Wait end of bit */
		while (gpio0->in & (1 << th_pin_num)) {
			nop();
		}
	}
	return val;
}

void TH_display(void)
{
	struct lpc_gpio* gpio0 = LPC_GPIO_0;
	unsigned char data[5];
	unsigned char checksum = 0;
	int i = 0;

	/* Set pin as output */
	gpio0->data_dir |= (1 << th_pin_num);

	/* Send the "start" bit */
	gpio0->clear = (1 << th_pin_num);
	msleep(50);
	gpio0->set = (1 << th_pin_num);

	/* Set pin as input */
	gpio0->data_dir &= ~(1 << th_pin_num);

	/* Wait for start conditions */
	debug(0, 'S');
	status_led(both);
	while (gpio0->in & (1 << th_pin_num)) {
		nop();
	}
	status_led(none);
	debug(0, 'c');
	while (!(gpio0->in & (1 << th_pin_num))) {
		nop();
	}
	status_led(both);
	debug(0, 'C');
	while (gpio0->in & (1 << th_pin_num)) {
		nop();
	}
	status_led(none);
	debug(0, 'D');

	/* Start reading data : 40 bits */
	for (i = 0; i < 5; i++){
	    data[i] = read_dht11_dat();
		if (i < 4) {
			checksum += data[i];
		}
    }

	if ((checksum & 0xFF) != data[4]) {
		char buff[60];
		int len = 0;
		status_led(red_only);
		len = snprintf(buff, 60, "TH_ERR - H: 0x%02x,0x%02x - T: 0x%02x,0x%02x - C: 0x%02x\r\n", data[0], data[1], data[2], data[3], data[4]);
		serial_write(1, buff, len);
	} else {
		char buff[25];
		int len = 0;
		len = snprintf(buff, 25, "H: %d,%d - T: %d,%d\r\n", data[0], data[1], data[2], data[3]);
		serial_write(1, buff, len);
		status_led(green_only);
	}

}

/***************************************************************************** */
/* EEPROM */
#define DUMP_BUFF_SIZE 80
void module_desc_dump(uint8_t serial_num)
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
	serial_write(serial_num, "Module :\r\n", 10);
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
		ret += serial_write(serial_num, (buff + ret), (len - ret));
	} while (ret < len);
}

int module_desc_set(char* name, uint8_t module_version, uint16_t serial_num)
{
	int ret = 0;
	struct module_desc desc = {
		.serial_number = serial_num,
		.version = module_version,
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
	}
}


/***************************************************************************** */
/* RGB Led */
#define LED_RGB_RED   23
#define LED_RGB_GREEN 24
#define LED_RGB_BLUE  25
void RGB_Led_config(uint8_t red_pin, uint8_t green_pin, uint8_t blue_pin)
{
	/* Timer configuration */
	struct timer_config timer_conf = {
		.mode = LPC_TIMER_MODE_PWM,
		.config = { (LPC_PWM_CHANNEL_ENABLE(0) | LPC_PWM_CHANNEL_ENABLE(1) | LPC_PWM_CHANNEL_ENABLE(2)), 0, 0, 0 },
		.match = { 20, 125, 200, 200 },
	};
	timer_setup(LPC_TIMER_32B1, &timer_conf);

	/* Configure the pins as match output */
	timer_pins_setup(0, green_pin, LPC_TIMER_PIN_FUNC_MATCH);
	timer_pins_setup(0, red_pin, LPC_TIMER_PIN_FUNC_MATCH);
	timer_pins_setup(0, blue_pin, LPC_TIMER_PIN_FUNC_MATCH);

	/* Start the timer */
	timer_start(LPC_TIMER_32B1);
}


/***************************************************************************** */
/* Maxim's MAX31855 themocouple to digital converter */
uint16_t Thermocouple_Read(void)
{
	char buff[50];
	int len = 0;
	uint16_t data[2];
	int temp = 0, deci = 0;

	spi_read(data, 2, SPI_USE_FIFO);
	temp = (data[0] >> 4) & 0x07FF;
	/* * ((data & 0x2000) * -1);*/
	deci = 25 * ((data[0] >> 2) & 0x03);
	len = snprintf(buff, 50, "TC: raw(0x%04x - 0x%04x) - deg: %d,%d\r\n", data[0], data[1], temp, deci);
	serial_write(1, buff, len);

	return temp;
}

