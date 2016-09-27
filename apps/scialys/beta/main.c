/****************************************************************************
 *   apps/scialys/beta/main.c
 *
 * Scialys system for solar-panel power generation tracking and fair use.
 *
 * Copyright 2016 Nathael Pajani <nathael.pajani@ed3l.fr>
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



#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/adc.h"
#include "drivers/ssp.h"
#include "drivers/i2c.h"

#include "extdrv/status_led.h"
#include "extdrv/ws2812.h"
#include "extdrv/max31855_thermocouple.h"
#include "extdrv/tmp101_temp_sensor.h"

#define MODULE_VERSION    0x01
#define MODULE_NAME "Scialys uC"


#define SELECTED_FREQ  FREQ_SEL_48MHz


/***************************************************************************** */
/* System configuration
 * Most of the defines in here should go to configuration setting in user flash
 */

/* Period of the decrementer handler from the systick interrupt */
#define DEC_PERIOD  100

/* If temperature falls bellow FORCE_HEATER_TEMP value, we enter forced heater mode, until
 *    TARGET_FORCED_HEATER_TEMP is reached.
 * When in forced heater mode, the heated is controlled to heat at FORCED_MODE_VALUE which 
 *    is between 0 and 255.
 */
#define FORCE_HEATER_TEMP  25
#define TARGET_FORCED_HEATER_TEMP 45
#define NO_FORCED_HEATING_ON_SUNNY_DAYS 750
#define FORCED_MODE_VALUE  190 /* A fraction of 255 */
uint32_t forced_heater_mode = 0;
uint32_t forced_heater_delay = 0;
uint32_t forced_heater_time = 0;

#define FORCED_HEATER_DELAY      (7 * 3600 * 1000 / DEC_PERIOD)  /* Delay before automatic forced heating */
#define FORCED_HEATER_DURATION   (3 * 3600 * 1000 / DEC_PERIOD)  /* Duration of automatic forced heating */

#define MANUAL_ACTIVATION_DURATION   (3600 * 1000 / DEC_PERIOD)  /* One hour */

uint32_t never_force = 0;


#define DAY_IS_EJP  0  /* Input is pulled low when EJP is ON */
int ejp_in = 0;



/***************************************************************************** */
/* Pins configuration */
/* pins blocks are passed to set_pins() for pins configuration.
 * Unused pin blocks can be removed safely with the corresponding set_pins() call
 * All pins blocks may be safelly merged in a single block for single set_pins() call..
 */
const struct pio_config common_pins[] = {
	/* UART 0 : Config / Debug / USB */
	{ LPC_UART0_RX_PIO_0_1,  LPC_IO_DIGITAL },
	{ LPC_UART0_TX_PIO_0_2,  LPC_IO_DIGITAL },
	/* UART 1 : UEXT */
	{ LPC_UART1_RX_PIO_0_8,  LPC_IO_DIGITAL },
	{ LPC_UART1_TX_PIO_0_9,  LPC_IO_DIGITAL },
	/* I2C : RTC, Display, UEXT */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	/* SPI (Thermocouple + uSD card + UEXT) */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	/* TIMER_32B0 */
	{ LPC_TIMER_32B0_M0_PIO_0_18, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Fan control */
	/* GPIO */
	{ LPC_GPIO_0_0, LPC_IO_DIGITAL },  /* Clkout / interrupt from RTC */
	{ LPC_GPIO_0_3, LPC_IO_DIGITAL },  /* EJP / External switch input */
	{ LPC_GPIO_0_4, LPC_IO_DIGITAL },  /* Zero crossing detection input */
	{ LPC_GPIO_0_5, LPC_IO_DIGITAL },  /* Temperature driver warning (Mosfet board only) */
	{ LPC_GPIO_0_6, LPC_IO_DIGITAL },  /* Mosfet driver Shutdown (Mosfet board only) */
	{ LPC_GPIO_0_7, LPC_IO_DIGITAL },  /* Mosfet / Triac control */
	{ LPC_GPIO_0_12, LPC_IO_DIGITAL }, /* ISP / User button OK */
	{ LPC_GPIO_0_15, LPC_IO_DIGITAL }, /* Thermocouple chip select */
	{ LPC_GPIO_0_23, LPC_IO_DIGITAL }, /* WS2812B RGB Leds control */
	{ LPC_GPIO_0_26, LPC_IO_DIGITAL }, /* User button B2 */
	{ LPC_GPIO_0_27, LPC_IO_DIGITAL }, /* User button B1 */
	{ LPC_GPIO_0_28, LPC_IO_DIGITAL }, /* Charge State */
	{ LPC_GPIO_1_1, LPC_IO_DIGITAL },  /* Uext Chip select / Module eeprom select */
	{ LPC_GPIO_1_6, LPC_IO_DIGITAL },  /* uSD Card SPI Chip Select */
	ARRAY_LAST_PIO,
};

const struct pio_config adc_pins[] = {
	{ LPC_ADC_AD0_PIO_0_30, LPC_IO_ANALOG },  /* ADC0 */
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },  /* ADC1 */
	{ LPC_ADC_AD2_PIO_1_0, LPC_IO_ANALOG },   /* ADC2 */
	{ LPC_ADC_AD7_PIO_1_5, LPC_IO_ANALOG },   /* ADC3 */
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_2;
const struct pio status_led_red = LPC_GPIO_1_3;

/* Inputs */
/* Buttons */
const struct pio button_ok = LPC_GPIO_0_12;
const struct pio button_b1 = LPC_GPIO_0_27;
const struct pio button_b2 = LPC_GPIO_0_26;
/* External signals */
const struct pio rtc_in_pin = LPC_GPIO_0_0;
const struct pio ejp_in_pin = LPC_GPIO_0_3;
const struct pio zero_cross_in_pin = LPC_GPIO_0_4;
const struct pio th_warn_in_pin = LPC_GPIO_0_5;
const struct pio charge_status_in_pin = LPC_GPIO_0_28;

/* Thermocouple reading */
const struct max31855_sensor_config thermo = {
	.ssp_bus_num = 0,
	.chip_select = LPC_GPIO_0_15,
};

/* TMP101 onboard I2C temperature sensor */
#define TMP101_ADDR  0x94  /* Pin Addr0 (pin5 of tmp101) connected to VCC */
struct tmp101_sensor_config tmp101_sensor = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR,
	.resolution = TMP_RES_ELEVEN_BITS,
};

/* Led control data pin */
const struct pio ws2812_data_out_pin = LPC_GPIO_0_23;


/***************************************************************************** */
/* Basic system init and configuration */
static volatile int got_wdt_int = 0;
void wdt_callback(void)
{
	got_wdt_int = 1;
}

const struct wdt_config wdconf = {
	.clk_sel = WDT_CLK_IRC,
	.intr_mode_only = 0,
	.callback = wdt_callback,
	.locks = 0,
	.nb_clk = 0x0FFFFFF, /* 0x3FF to 0x03FFFFFF */
	.wdt_window = 0,
	.wdt_warn = 0x3FF,
};

void system_init()
{
	/* Configure the Watchdog */
	watchdog_config(&wdconf);
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
	uprintf(UART0, name);
	while (1);
}


/***************************************************************************** */
void config_rx(uint8_t c)
{
}
void cmd_rx(uint8_t c)
{
}

void dmx_send_frame(uint8_t start_code, uint8_t* slots, uint16_t nb_slots)
{
}

enum buttons {
	BUTTON_NONE = 0,
	BUTTON_OK,
	BUTTON_UP,
	BUTTON_DOWN,
};

uint32_t manual_activation_request = 0;
uint8_t button_pressed = 0;
void manual_activation(uint32_t gpio) {
    manual_activation_request = MANUAL_ACTIVATION_DURATION;
	button_pressed = BUTTON_OK;
}
void manual_up(uint32_t gpio) {
    manual_activation_request = MANUAL_ACTIVATION_DURATION;
	button_pressed = BUTTON_UP;
}
void manual_down(uint32_t gpio) {
    manual_activation_request = MANUAL_ACTIVATION_DURATION;
	button_pressed = BUTTON_DOWN;
}
void handle_dec_request(uint32_t curent_tick) {
	if (manual_activation_request > 0) {
		manual_activation_request--;
	}
	if (forced_heater_mode == 1) {
		if (forced_heater_delay > 0) {
			forced_heater_delay--;
		}
		if (forced_heater_time > 0) {
			forced_heater_time--;
		}
	}
}


void zero_cross(uint32_t gpio) {
}
void th_warning(uint32_t gpio) {
}


void temp_config(int uart_num)
{
	int ret = 0;
	ret = tmp101_sensor_config(&tmp101_sensor);
	if (ret != 0) {
		uprintf(uart_num, "Temp config error: %d\n", ret);
	}
}


#define NB_VAL 20


enum modes {
	heat = 'C',
	ejp = 'E',
	no_heat_prod = 'P',
	forced = 'F',
	temp_OK = 'T',
	manual = 'M',
	idle_heat = 'L',
	full_heat = 'F',
};

/***************************************************************************** */
int main(void)
{
	uint16_t isnail_solar_values[NB_VAL];
	uint16_t isnail_home_values[NB_VAL];
	uint8_t idx = 0;
	uint32_t loop = 0;
	char mode = heat; /* Debug info */

	system_init();
	status_led(red_only);
	uart_on(UART0, 115200, config_rx);
	uart_on(UART1, 115200, cmd_rx);
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	ssp_master_on(thermo.ssp_bus_num, LPC_SSP_FRAME_SPI, 8, 4*1000*1000);
	adc_on(NULL);

	/* Thermocouple configuration */
	max31855_sensor_config(&thermo);
	uprintf(UART0, "Thermocouple config done\n");

	/* TMP101 sensor config */
	temp_config(UART0);

	/* Activate on Rising edge (button release) */
	set_gpio_callback(manual_activation, &button_ok, EDGE_RISING);
	set_gpio_callback(manual_up, &button_b1, EDGE_RISING);
	set_gpio_callback(manual_down, &button_b2, EDGE_RISING);

	/* Zero cross and alert pin */
	set_gpio_callback(zero_cross, &zero_cross_in_pin, EDGE_RISING);
	set_gpio_callback(th_warning, &th_warn_in_pin, EDGE_RISING);

	/* Start ADC sampling */
	adc_start_burst_conversion(ADC_MCH(0) | ADC_MCH(1) | ADC_MCH(2) | ADC_MCH(7), LPC_ADC_SEQ(0));

	/* Configure Input GPIO */
	config_gpio(&ejp_in_pin, 0, GPIO_DIR_IN, 0);
	config_gpio(&rtc_in_pin, 0, GPIO_DIR_IN, 0);
	config_gpio(&charge_status_in_pin, 0, GPIO_DIR_IN, 1);

	/* WS2812B Leds on display board */
	ws2812_config(&ws2812_data_out_pin);

	status_led(green_only);

	msleep(50);
	/* Read parameters from memory */
	if (1) {
		never_force = 0;
		forced_heater_delay = FORCED_HEATER_DELAY;
		forced_heater_time = FORCED_HEATER_DURATION;
	}

	while (1) {
		static uint8_t command_val = 0;
		uint32_t moyenne_solar = 0;
		uint32_t moyenne_home = 0;
		uint16_t isnail_val_solar = 0;
		uint16_t isnail_val_home = 0;
		uint16_t acs_val_load = 0;
		uint16_t user_potar = 0;
		int water_centi_degrees = 0;
		int tmp101_deci_degrees = 0;

		mode = heat;
		tmp101_sensor_start_conversion(&tmp101_sensor);
		/* Always track power consumption and production */
		adc_get_value(&isnail_val_solar, LPC_ADC(1));
		adc_get_value(&isnail_val_home, LPC_ADC(0));
		adc_get_value(&acs_val_load, LPC_ADC(2));
		adc_get_value(&user_potar, LPC_ADC(7));
		/* Convert to mA value */
		isnail_val_solar = ((isnail_val_solar * 32) * 2); /* 3.2mV / digit, 50mV -> 1A */
		isnail_val_home = ((isnail_val_home * 32) * 2); /* 3.2mV / digit, 50mV -> 1A */
		/* Store value */
		isnail_solar_values[idx] = isnail_val_solar;
		isnail_home_values[idx++] = isnail_val_home;
		if (idx == NB_VAL) {
			idx = 0;
		}
		/* Compute average value when we sampled enough values */
		/* FIXME : Improve by removing oldest value before storing new one in table and adding new one */
		if ((idx == 0) || (idx == (NB_VAL / 2))) {
			int i = 0;
			for (i = 0; i < NB_VAL; i++) {
				moyenne_solar += isnail_solar_values[i];
				moyenne_home += isnail_home_values[i];
			}
			moyenne_solar = moyenne_solar / NB_VAL;
			moyenne_home = moyenne_home / NB_VAL;
		} else {
			/* Sleep for a litle more than a period (20ms at 50Hz) */
			msleep(23);
			continue;
		}

		/* Feed the dog */
		if ((moyenne_solar != 0) && (moyenne_home != 0)) {
			watchdog_feed();
		}

		/* Get internal temperature */
		if (1) {
			int ret = 0;
			msleep(40);
			ret = tmp101_sensor_read(&tmp101_sensor, NULL, &tmp101_deci_degrees);
			if (ret != 0) {
				uprintf(UART0, "TMP101 read error : %d\n", ret);
			}
		}

		/* Get thermocouple value */
		if (1) {
			int ret = 0;
			ret = max31855_sensor_read(&thermo, NULL, &water_centi_degrees);
			if (ret != 0) {
				uprintf(UART0, "Water Temp read error : %d\n", ret);
			}
		}
		if (water_centi_degrees < (FORCE_HEATER_TEMP * 100)) {
			if (forced_heater_mode == 0) {
				uprintf(UART0, "Entering forced mode\n");
				forced_heater_mode = 1;
			}
			status_led(red_on);
			mode = forced;
		} else if ((water_centi_degrees > (TARGET_FORCED_HEATER_TEMP * 100)) && (forced_heater_mode == 1)) {
			status_led(red_off);
			forced_heater_mode = 0;
			command_val = 0;
			uprintf(UART0, "Forced mode exit\n");
			mode = temp_OK;
		}

		/* Do not force if there is some sun, it may be enough to heat again */
		if (moyenne_solar > NO_FORCED_HEATING_ON_SUNNY_DAYS) {
			forced_heater_mode = 0;
			mode = no_heat_prod;
			forced_heater_delay = FORCED_HEATER_DELAY;
		}

		/* Do not force heating if this is an EJP day */
		ejp_in = gpio_read(ejp_in_pin);
		if (ejp_in == DAY_IS_EJP) {
			forced_heater_mode = 0;
			mode = ejp;
		}

		if (never_force == 1) {
			forced_heater_mode = 0;
		}

		/* Did the user request a forced heating ? */
		if (manual_activation_request > 1) {
			forced_heater_mode = 1;
			mode = manual;
			if (manual_activation_request == MANUAL_ACTIVATION_DURATION) {
				uprintf(UART0, "Entering manual forced mode for %d ticks\n", manual_activation_request);
				/* Add a systick callback to handle time counting */
				add_systick_callback(handle_dec_request, DEC_PERIOD);
			}
			if (manual_activation_request < 10) {
				uprintf(UART0, "Leaving manual forced mode\n");
				manual_activation_request = 0;
				remove_systick_callback(handle_dec_request);
			}
		}


		/* Which is the current mode ? */
		if (forced_heater_mode == 1) {
			/* Forced heating mode */
			if ((forced_heater_delay == 0) && (forced_heater_time > 0)) {
				command_val = FORCED_MODE_VALUE;
			}
			if (forced_heater_time == 0) {
				forced_heater_delay = FORCED_HEATER_DELAY;
				forced_heater_time = FORCED_HEATER_DURATION;
			}
		} else if (moyenne_solar < moyenne_home) {
			/* Low production mode */
			if (command_val > 25) {
				command_val -= 25;
			} else {
				command_val = 0;
				mode = idle_heat;
			}
			status_led(green_off);
		} else {
			/* High production mode */
			if (command_val < 245) {
				command_val += 10;
			} else {
				command_val = 255;
				mode = full_heat;
			}
			status_led(green_on);
		}

		/* Send DMX frame */
		dmx_send_frame(0x00, 0, 1);
		/* Display */
		if (1) {	
			int abs_centi = water_centi_degrees;
			uprintf(UART0, "%c:%d - Is: %d,%04d - Ih: %d,%04d\n", mode, loop++,
						(moyenne_solar / 1000), (moyenne_solar % 1000),
						(moyenne_home / 1000), (moyenne_home % 1000));
			if (water_centi_degrees < 0) {
				abs_centi = -water_centi_degrees;
			}
			uprintf(UART0, "Water Temp : % 4d.%02d\n", (water_centi_degrees / 100), (abs_centi % 100));
			if (tmp101_deci_degrees < 0) {
				abs_centi = -tmp101_deci_degrees;
			} else {
				abs_centi = tmp101_deci_degrees;
			}
			uprintf(UART0, "Internal Temp : % 4d.%02d\n", (tmp101_deci_degrees / 10), (abs_centi % 10));
			uprintf(UART0, "ADC: Sol: %dmA, Home: %dmA, Load: %d, User: %d\n",
							isnail_val_solar, isnail_val_home, acs_val_load, user_potar);
			if (button_pressed != 0) {
				uprintf(UART0, "Button : %d\n", button_pressed);
				button_pressed  = 0;
			}
			uprintf(UART0, "CMD: %d\n\n", 0);
			ws2812_set_pixel(0, (isnail_val_home / 100), (isnail_val_solar / 100), 0);
			ws2812_set_pixel(1, (acs_val_load >> 2), 0, (user_potar >> 2));
			ws2812_send_frame(0);
		}
	}
	return 0;
}



