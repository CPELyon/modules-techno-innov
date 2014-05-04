/****************************************************************************
 *   main.c
 *
 *
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
#include "core/system.h"
#include "lib/stdio.h"
#include "drivers/i2c.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/status_led.h"
#include "drivers/temp.h"
#include "drivers/adc.h"
#include "drivers/timers.h"
#include "drivers/ssp.h"
#include "drivers/cc1101.h"

#include "examples.h"


#ifndef MODULE_SERIAL_NUM
#define MODULE_SERIAL_NUM 10
#endif
#define MODULE_VERSION    0x04
#define MODULE_NAME "GPIO Demo Module"


#define SELECTED_FREQ  FREQ_SEL_48MHz

/***************************************************************************** */
/* Pins configuration */
struct pio clkout_pin[] = {
	ARRAY_LAST_PIN,
};
struct pio uart0_pins[] = {
	LPC_UART0_RX_PIO_0_1,
	LPC_UART0_TX_PIO_0_2,
	LPC_UART0_RTS_PIO_0_0,
	ARRAY_LAST_PIN,
};
struct pio uart1_pins[] = {
	LPC_UART1_RX_PIO_0_8,
	LPC_UART1_TX_PIO_0_9,
	ARRAY_LAST_PIN,
};
struct pio i2c0_pins[] = {
	LPC_I2C0_SCL_PIO_0_10,
	LPC_I2C0_SDA_PIO_0_11,
	ARRAY_LAST_PIN,
};
struct pio ssp0_pins[] = {
	/* Warning : Order is used later on, DO NOT change it ! */
	LPC_SSP0_SCLK_PIO_0_14,
	LPC_SSP0_MOSI_PIO_0_17,
	LPC_SSP0_MISO_PIO_0_16,
	LPC_GPIO_0_15, /* Use the GPIO config for the pin when SPI is configured as master */
/*	LPC_SSP0_SSEL_PIO_0_15, */
	ARRAY_LAST_PIN,
};
struct pio timer0_pins[] = { /* TIMER_16B0 */
	ARRAY_LAST_PIN,
};
struct pio timer1_pins[] = { /* TIMER_16B1 */
	ARRAY_LAST_PIN,
};
struct pio timer2_pins[] = { /* TIMER_32B0 */
	LPC_TIMER_32B0_M1_PIO_0_19, /* PWM out for Servo Motor */
	ARRAY_LAST_PIN,
};
struct pio timer3_pins[] = { /* TIMER_32B1 */
	LPC_TIMER_32B1_M0_PIO_0_23, /* RGB Led Red */
	LPC_TIMER_32B1_M1_PIO_0_24, /* RGB Led Green */
	LPC_TIMER_32B1_M2_PIO_0_25, /* RGB Led Blue */
	ARRAY_LAST_PIN,
};
struct pio adc_pins[] = {
	LPC_ADC_AD0_PIO_0_30,
	LPC_ADC_AD1_PIO_0_31,
	LPC_ADC_AD2_PIO_1_0,
	LPC_ADC_AD3_PIO_1_1,
	LPC_ADC_AD4_PIO_1_2,
	LPC_ADC_AD5_PIO_1_3,
	ARRAY_LAST_PIN,
};
struct pio gpio_pins[] = {
	LPC_GPIO_0_7, /* Temp Alert, hard-wired on the board */
	LPC_GPIO_0_6, /* Used for DTH11 */
	LPC_GPIO_0_12, /* ISP Used as button */
	LPC_GPIO_0_4, /* Led toggle on ISP button press */
	LPC_GPIO_0_19, /* Used as SPI chip select for Thermocouple reading */
	ARRAY_LAST_PIN,
};
#define DTH11_GPIO  (&gpio_pins[1])
#define BUTTON_GPIO (&gpio_pins[2])
#define LED_GPIO    (&gpio_pins[3])
#define THERMOCOUPLE_SLAVE_SEL   (&gpio_pins[4])

void system_init()
{
	/* Stop the watchdog */
	stop_watchdog(); /* Do it right now, before it gets a chance to break in */

	system_brown_out_detection_config(0);
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	gpio_on();
	status_led_config();
	/* System tick timer MUST be configured and running in order to use the sleeping
	 * functions */
	systick_timer_on(1); /* 1ms */
}

/* Define our fault handler. This one is not mandatory, the dummy fault handler 
 * will be used when it's not overridden here.
 * Note : The default one does a simple infinite loop. If the watchdog is deactivated
 * the system will hang.
 */
void fault_info(const char* name, uint32_t len)
{
	serial_write(1, name, len);
	/*FIXME : wait for end of Tx and perform soft reset of the micro-controller ! */
}


#define PWM_CHAN  1


void test(uint32_t curent_tick)
{
	struct lpc_gpio* gpio = LPC_GPIO_REGS(0);
	gpio->data_dir |= (1 << 4);
	gpio->toggle |= (1 << 4);
}


#define RX_BUFF_LEN  50
#if RS485
/* Put any data received from RS485 UART in rs485_rx_buff and set rs485_rx flag
 * when either '\r' or '\n' is received.
 * This function is very simple and data received between rs485_rx flag set and
 * rs485_ptr rewind to 0 may be lost. */
static volatile uint32_t rs485_rx = 0;
static volatile uint8_t rs485_rx_buff[RX_BUFF_LEN];
static volatile uint8_t rs485_ptr = 0;
void rs485_out(uint8_t c)
{
	if (rs485_ptr < RX_BUFF_LEN) {
		rs485_rx_buff[rs485_ptr++] = c;
	} else {
		rs485_ptr = 0;
	}
	if (c == '\n') {
		rs485_rx = 1;
	}
}
#endif

#if CC1101
/* Data sent on radio comes from the UART, put any data received from UART in
 * cc_tx_buff and send when either '\r' or '\n' is received.
 * This function is very simple and data received between cc_tx flag set and
 * cc_ptr rewind to 0 may be lost. */
static volatile uint32_t cc_tx = 0;
static volatile uint8_t cc_tx_buff[RX_BUFF_LEN];
static volatile uint8_t cc_ptr = 0;
void cc1101_test_rf_serial_link_tx(uint8_t c)
{
	if (cc_ptr < RX_BUFF_LEN) {
		cc_tx_buff[cc_ptr++] = c;
	} else {
		cc_ptr = 0;
	}
	if ((c == '\n') || (c == '\r')) {
		cc_tx = 1;
	}
}
#endif

/***************************************************************************** */
int main(void) {
	system_init();
	uart_on(0, 115200, NULL);
	uart_on(1, 115200, NULL);
#if RS485
	uart_on(0, 115200, rs485_out);
#endif
#if CC1101
	uart_on(1, 115200, cc1101_test_rf_serial_link_tx);
#endif
	adc_on();
	timer_on(LPC_TIMER_32B1, 0);
	timer_on(LPC_TIMER_32B0, 0);
	ssp_master_on(0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000); /* bus_num, frame_type, data_width, rate */

	add_systick_callback(test, 200); /* callback, ms */
	systick_start();

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
	dth11_config(DTH11_GPIO);
	temp_config(0);

	/* GPIO interrupt test */
	gpio_intr_toggle_config(BUTTON_GPIO, LED_GPIO);

	/* Servo motor PWM control test */
	servomotor_config(LPC_TIMER_32B0, PWM_CHAN);

#if CC1101
	/* Radio */
	cc1101_init(0, &(ssp0_pins[3]), &(ssp0_pins[2])); /* ssp_num, cs_pin, miso_pin */
	cc1101_config();
#endif

	RGB_Led_config(LPC_TIMER_32B1);

#if RS485
	/* Configure RS485 */
	{
		uint32_t rs485_ctrl = LPC_RS485_ENABLE;
		rs485_ctrl |= LPC_RS485_DIR_PIN_RTS | LPC_RS485_AUTO_DIR_EN | LPC_RS485_DIR_CTRL_INV;
		uart_set_mode_rs485(0, rs485_ctrl, 0, 1);
	}
#endif

#define BUFF_LEN 60
	while (1) {
		char buff[BUFF_LEN];
		int len = 0;
		chenillard(25);
#if RS485
		if (rs485_rx) {
			/* Fixme: We should copy data before sending. The use of the same buffer
			 *    might lead to data corruption, but a copy is made by serial_write().
			 * Fixme: serial_write() will not send more than 64 bytes at a time, we
			 *    should check the return value ! */
			serial_write(1, (char*)rs485_rx_buff, rs485_ptr);
			rs485_rx = 0;
			rs485_ptr = 0;
		}
#endif
#if CC1101
		/* Any Data to send */
		if (cc_tx) {
			uint8_t cc_tx_data[RX_BUFF_LEN + 2];
			uint8_t len = 0, tx_len = cc_ptr;
			uint8_t val = 0;
			int ret = 0;
			/* Create a local copy */
			memcpy((char*)&(cc_tx_data[2]), (char*)cc_tx_buff, tx_len);
			/* "Free" the rx buffer as soon as possible */
			cc_ptr = 0;
			/* Prepare buffer for sending */
			cc_tx_data[0] = tx_len + 1;
			cc_tx_data[1] = 0; /* Broadcast */
			/* Send */
			ret = cc1101_send_packet(cc_tx_data, (tx_len + 2));
			/* Give some feedback on UART 1 */
			len = snprintf(buff, BUFF_LEN, "Tx ret: %d\r\n", ret);
			serial_write(1, buff, len);
			/* Wait a short time for data to be sent ... */
			/* FIXME : This should be done using the packet sent signal from CC1101 on GDO pin */
			msleep(2);
			do {
				ret = cc1101_tx_fifo_state();
				if (ret < 0) {
					len = snprintf(buff, BUFF_LEN, "Tx Underflow !\r\n");
					serial_write(1, buff, len);
					break;
				}
			} while (ret != 0);
			/* Get back to Receiver mode */
			cc1101_enter_rx_mode();
			/* And give some feedback again */
			val = cc1101_read_status();
			len = snprintf(buff, BUFF_LEN, "Status : 0x%02x, Sending : %d\r\n", val, tx_len);
			serial_write(1, buff, len);
			serial_write(1, (char*)&(cc_tx_data[2]), tx_len);
			cc_tx = 0;
		}
		/* Check for received data on RF link */
		{
			int ret = 0, rxlen = 0, addr = 0;
			uint8_t status = 0;
			/* Check for received packet (and get it if any) */
			ret = cc1101_receive_packet((uint8_t*)buff, 50, &status);
			cc1101_enter_rx_mode(); /* We should already be in receive mode. */
			/* >0 means we got something, <0 is an error, =0 is no packet received */
			if (ret > 0) {
				uint8_t val = 0;
				len = ret;
				/* Send packet on UART 1 */
				serial_write(1, &buff[2], (len - 2));
				serial_write(1, "\r\n", 2);
				/* And also some feedback */
				rxlen = buff[0];
				addr = buff[1];
				val = cc1101_get_signal_strength_indication();
				len = snprintf(buff, BUFF_LEN, "Status: %d, link: %d, len: %d/%d, addr: %d\r\n", status, val, rxlen, len, addr);
				serial_write(1, buff, len);
			} else if (ret < 0) {
				/* Send error on UART 1 */
				len = snprintf(buff, BUFF_LEN, "Rx Error: %d, len: %d\r\n", -ret, (status & 0x7F));
				serial_write(1, buff, len);
			}
		}
#endif
#if SERVO
		{
			uint16_t val = 0;
			/* Get the ADC value */
			val = adc_display(LPC_ADC_NUM(1));
			/* The following computation depends on the resistor and potentiometer used
			 * to change the ADC input voltage */
			val = (((val - 480) & ~(0x03)) / 3);
			/* Display the value and new angle on UART 1 */
			len = snprintf(buff, BUFF_LEN, "Angle: %d/180\r\n", val);
			serial_write(1, buff, len);
			/* And change the servo-motor command */
			servomotor_pwm_update(LPC_TIMER_32B0, PWM_CHAN, val);
		}
#endif
	}
	return 0;
}




