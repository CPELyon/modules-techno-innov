/****************************************************************************
 *   apps/epaper/main.c
 *
 * e-paper example
 *
 * Copyright 2013-2015 Nathael Pajani <nathael.pajani@ed3l.fr>
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
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/timers.h"
#include "extdrv/status_led.h"
#include "drivers/ssp.h"

#include "extdrv/epaper.h"
#include "lib/font.h"

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
	/* UART 1 */
	{ LPC_UART1_RX_PIO_0_8, LPC_IO_DIGITAL },
	{ LPC_UART1_TX_PIO_0_9, LPC_IO_DIGITAL },
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	/* PWM */
#define LPC_TIMER_PIN_CONFIG   (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL | LPC_IO_DRIVE_HIGHCURENT)
	{ LPC_TIMER_32B0_M1_PIO_0_19, LPC_TIMER_PIN_CONFIG },
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

const struct pio button = LPC_GPIO_0_12; /* ISP button */

/*
 * E-paper configuration, 2.7" size
 *   see Pervasive display documentation (Doc. No. 4P008-00)
 */
#define LINE_SIZE  264
struct epaper_definition epaper_def = {
	.pin_reset = LPC_GPIO_0_20,
	.pin_power_ctrl = LPC_GPIO_0_21,
	.pin_discharge = LPC_GPIO_0_22,
	.pin_border_ctrl = LPC_GPIO_0_23,
	.pin_busy = LPC_GPIO_0_24,
	/* SPI */
	.pin_spi_cs = LPC_GPIO_0_15,
	.spi_num = 0,
	/* PWM */
	.pwm_timer_num = LPC_TIMER_32B0,
	.pwm_timer_conf = {
		.mode = LPC_TIMER_MODE_PWM,
		/* pin PIO0_19 is channel 1 - PWM cycle control on channel 3 */
		.config = { LPC_PWM_CHANNEL_ENABLE(1), 3, 0, 0 },
		.match = { 0, 60, 0, 120 }, /* 50% duty cycle on channel 1, approx 200 KHz whith 48MHz clock */
	},
	.pixels_per_line = LINE_SIZE,
	.bytes_per_line = (LINE_SIZE / 8),
	.lines = 176,
	.bytes_per_scan = (176 / 4),
	.channel = {0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFE, 0x00, 0x00},  /* Reg 0x01 */
	.gate_source_level = 0x00,  /* Reg 0x04 */
	.stage_time = 630,
	.line_termination_required = 1,
};


/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	stop_watchdog(); /* Do it right now, before it gets a chance to break in */

	/* Note: Brown-Out detection must be powered to operate the ADC. adc_on() will power
	 *  it back on if called after system_init() */
	system_brown_out_detection_config(0);
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
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
	serial_write(1, name, len);
	/*FIXME : wait for end of Tx and perform soft reset of the micro-controller ! */
}

static volatile uint8_t white_request = 0;
void button_request(uint32_t gpio) {
    white_request = 1;
}

extern unsigned int _sram_base;
extern unsigned int _end_text;
extern unsigned int _end_data;


#define TEXT_LENGTH_MAX 33
static volatile uint8_t display_text_buff[TEXT_LENGTH_MAX];
static volatile uint8_t update_text = 0;
void recv_text(uint8_t c)
{
	static uint8_t idx = 0;
	static uint8_t rx_buff[TEXT_LENGTH_MAX];
	rx_buff[idx++] = c;
	if ((c == '\0') || (c == '\r') || (c == '\n')) {
		update_text = 1;
		rx_buff[idx++] = '\0';
		memcpy((void*)display_text_buff, rx_buff, idx);
		idx = 0;
	}
}

void build_line(uint8_t* img_buff, uint8_t* text)
{
	int i = 0, line = 0;
	int len = strlen((char*)text);

	for (i = 0; i < len; i++) {
		uint8_t tile = (text[i] > first_font_char) ? (text[i] - first_font_char) : 0;
		uint8_t* tile_data = (uint8_t*)(&font[tile]);
		for (line = 0; line < 8; line++) {
			img_buff[ (line * epaper_def.bytes_per_line) + i ] = tile_data[7 - line];
		}
	}
}


/* FIXME :
 * Actual font data is stored in "human drawing order", but must be sent in revers bit order.
 * This function takes care of bit reversal for each font byte once and for all.
 * This should be done on the font file for execution efficiency and code size, but would make
 *   font update harder.
 */
void fix_font(uint64_t* font_64, uint8_t size)
{
	int i = 0;
	uint32_t* f = (uint32_t*)font_64;
	size = size * 2;
	for (i = 0; i < size; i++) {
		// swap odd and even bits
		f[i] = ((f[i] >> 1) & 0x55555555) | ((f[i] & 0x55555555) << 1);
		// swap consecutive pairs
		f[i] = ((f[i] >> 2) & 0x33333333) | ((f[i] & 0x33333333) << 2);
		// swap nibbles ... 
		f[i] = ((f[i] >> 4) & 0x0F0F0F0F) | ((f[i] & 0x0F0F0F0F) << 4);
	}
}

/***************************************************************************** */
#define BUFF_LEN 60
int main(void) {

	system_init();
	uart_on(0, 115200, recv_text);
	ssp_master_on(0, LPC_SSP_FRAME_SPI, 8, 8*1000*1000); /* bus_num, frame_type, data_width, rate */
	set_gpio_callback(button_request, &button, EDGE_RISING);
	status_led(none);

	/* E-Paper */
	epaper_config(&epaper_def);
	/* Font fix */
	fix_font(font, NB_FONT_TILES);

	while (1) {
		if (white_request == 1) {
			static int img_num = 0;
			uint8_t* images = (uint8_t*)(&_end_text + (&_end_data - &_sram_base));
			uint8_t* image = (images + ((img_num % 2) * (epaper_def.bytes_per_line * epaper_def.lines)));

			status_led(green_only);

			epaper_on();
			/* Blank screen */
			epaper_uniform_display(0xAA); /* White */
			/*	epaper_uniform_display(0xFF); */ /* Black */
			/* Send image */
			epaper_send_frame(image, epaper_normal);
			epaper_off();

			white_request = 0;
			update_text = 1;
			img_num = ((img_num + 1) % 2);
		}

		if (update_text == 1) {
			uint8_t white_line[LINE_SIZE] = {0};  /* ((LINE_SIZE / 8) * 8) */
			uint8_t buffer_line[LINE_SIZE] = {0}; /* ((LINE_SIZE / 8) * 8) */

			build_line(buffer_line, (uint8_t*)display_text_buff);

			epaper_on();
			/* Blank text line */
			epaper_send_partial_frame(white_line, 0, 8, epaper_normal);
			/* Real text line */
			epaper_send_partial_frame(buffer_line, 0, 8, epaper_normal);
			epaper_off();

			update_text = 0;
		}
	}
	return 0;
}




