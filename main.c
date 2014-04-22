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
 * will be used when it's not overridden here */
void fault_info(const char* name, uint32_t len)
{
	serial_write(1, name, len);
}


#define PWM_CHAN  1


void test(uint32_t curent_tick)
{
	struct lpc_gpio* gpio = LPC_GPIO_REGS(0);
	gpio->data_dir |= (1 << 0);
	gpio->toggle |= (1 << 0);
}


#define RX_BUFF_LEN  50
static volatile uint32_t rx = 0;
static volatile uint8_t rx_buff[RX_BUFF_LEN];
static volatile uint8_t ptr = 0;
void rs485_out(uint8_t c)
{
	if (ptr < RX_BUFF_LEN) {
		rx_buff[ptr++] = c;
	} else {
		ptr = 0;
	}
	if (c == '\n') {
		rx = 1;
	}
}

/***************************************************************************** */
int main(void) {
	system_init();
	uart_on(0, 115200, rs485_out);
	uart_on(1, 115200, NULL);
	adc_on();
	timer_on(LPC_TIMER_32B1, 0);
	timer_on(LPC_TIMER_32B0, 0);
	ssp_master_on(0, LPC_SSP_FRAME_SPI, 16, 4*1000*1000); /* bus_num, frame_type, data_width, rate */

	add_systick_callback(test, 200);
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
	voltage_to_position_config(LPC_TIMER_32B0, PWM_CHAN);

	RGB_Led_config(LPC_TIMER_32B1);

	{
		uint32_t rs485_ctrl = LPC_RS485_ENABLE;
		rs485_ctrl |= LPC_RS485_DIR_PIN_RTS | LPC_RS485_AUTO_DIR_EN | LPC_RS485_DIR_CTRL_INV;
		uart_set_mode_rs485(0, rs485_ctrl, 0, 1);
	}

	while (1) {
		uint16_t val = 0;
		char buff[50];
		int len = 0;
		chenillard(25);
		if (rx) {
			/* Should copy data */
			serial_write(1, (char*)rx_buff, ptr);
			rx = 0;
			ptr = 0;
		}
		val = adc_display(LPC_ADC_NUM(1));
		val = (((val - 480) & ~(0x03)) / 3);
		len = snprintf(buff, 50, "Angle: %d/180\r\n", val);
		serial_write(1, buff, len);
		pwm_update(LPC_TIMER_32B0, PWM_CHAN, val);
		/* TH_display(); */
		TMP36_display(LPC_ADC_NUM(0));
		Thermocouple_Read(THERMOCOUPLE_SLAVE_SEL);
		temp_display(0);
	}
	return 0;
}




