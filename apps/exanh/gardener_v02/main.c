/****************************************************************************
 *   apps/exanh/gardener/main.c
 *
 * E-Xanh Gardener main board support
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

/* ISP button release disables the chenillard for 5 seconds */

#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "lib/errno.h"
#include "drivers/adc.h"
#include "drivers/ssp.h"
#include "extdrv/status_led.h"
#include "drivers/timers.h"

#include "extdrv/ws2812.h"
#include "extdrv/tmp101_temp_sensor.h"
#include "extdrv/rtc_pcf85363a.h"

#include "extdrv/sdmmc.h"



#define MODULE_VERSION   0x02
#define MODULE_NAME "E-Xanh Gardener"


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
	/* I2C 0 */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	/* ADC */
	{ LPC_ADC_AD1_PIO_0_31,  LPC_IO_ANALOG }, /* ADC1 : External sensor */
	{ LPC_ADC_AD4_PIO_1_2,  LPC_IO_ANALOG },   /* ADC2 : Battery */
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_GPIO_0_15, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* SD Card CS */
	/* GPIO */
	{ LPC_GPIO_0_0, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Led strip */
	{ LPC_GPIO_0_4, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Humidity Sensors Power control */
	{ LPC_GPIO_0_6, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Water level switch input */
	{ LPC_GPIO_0_12, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* ISP Button */
	{ LPC_GPIO_0_20, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Motor Enable / Sleep */
	{ LPC_GPIO_0_21, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Motor Direction control */
	{ LPC_GPIO_0_22, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Motor ON */
	{ LPC_GPIO_0_23, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Pompe */
	{ LPC_GPIO_0_24, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 0 */
	{ LPC_GPIO_0_25, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 1 */
	{ LPC_GPIO_0_26, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 2 */
	{ LPC_GPIO_0_27, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 3 */
	{ LPC_GPIO_0_28, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 4 */
	{ LPC_GPIO_0_29, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 5 */
	{ LPC_GPIO_1_1, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Button */
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

const struct pio isp = LPC_GPIO_0_12;   /* ISP button */
const struct pio button = LPC_GPIO_1_1; /* User button */

const struct pio ws2812_data_out_pin = LPC_GPIO_0_0; /* Led control data pin */

const struct pio motor_power = LPC_GPIO_0_22; /* Motor Power */
const struct pio motor_enable = LPC_GPIO_0_20; /* Motor Enable */
const struct pio motor_direction = LPC_GPIO_0_21; /* Motor Direction */

const struct pio pompe_on = LPC_GPIO_0_23; /* Water Pump */


#define NB_POS_SENSORS   6
const struct pio positions[NB_POS_SENSORS] = {
	LPC_GPIO_0_24,
	LPC_GPIO_0_25,
	LPC_GPIO_0_26,
	LPC_GPIO_0_27,
	LPC_GPIO_0_28,
	LPC_GPIO_0_29,
};


/***************************************************************************** */
/* SD/MMC Card */
struct sdmmc_card micro_sd = {
	.ssp_bus_num = SSP_BUS_0,
	.card_type = MMC_CARDTYPE_UNKNOWN,
	.block_size = 64,
	.chip_select = LPC_GPIO_0_15,
};

uint8_t mmc_data[MMC_MAX_SECTOR_SIZE];

const struct pio sclk_error = LPC_GPIO_0_18; /* Water Pump */


/***************************************************************************** */
/* RTC and time */
#define RTC_ADDR   0xA2
struct rtc_pcf85363a_config rtc_conf = {
	.bus_num = I2C0,
	.addr = RTC_ADDR,
	.mode = PCF85363A_MODE_RTC,
	.config_marker = PCF85363A_CONFIGURED_1,
	.batt_ctrl = PCF85363A_CONF_BATT_TH_2_8V,
};
/* Oldest acceptable time in RTC. BCD coded. */
const struct rtc_time oldest = {
	.year = 0x17,
	.month = 0x04,
	.day = 0x20,
	.hour = 0x13,
	.min = 0x00,
};
static struct rtc_time now;


/***************************************************************************** */
/* TMP101 onboard I2C temperature sensor */
#define TMP101_ADDR  0x94  /* Pin Addr0 (pin5 of tmp101) connected to VCC */
struct tmp101_sensor_config tmp101_sensor = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR,
	.resolution = TMP_RES_ELEVEN_BITS,
};


/***************************************************************************** */
void system_init(void)
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_set_default_power_state();
	system_brown_out_detection_config(0); /* No ADC used */
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
	uprintf(UART0, name);
	while (1);
}


static void pos_detect(uint32_t gpio);
static void pompe_force(uint32_t gpio);
static void motor_force(uint32_t gpio);

void system_config(void)
{
	int ret = 0;
	int i = 0;

	/* Configure temperature sensor */
	ret = tmp101_sensor_config(&tmp101_sensor);
	if (ret != 0) {
		uprintf(UART0, "Temp config error: %d\n", ret);
	}

	/* GPIO for Pump control */
	config_gpio(&pompe_on, 0, GPIO_DIR_OUT, 0);
	set_gpio_callback(pompe_force, &button, EDGES_BOTH);

	/* Selector position sensors */
	for (i = 0; i < NB_POS_SENSORS; i++) {
		set_gpio_callback(pos_detect, &(positions[i]), EDGES_BOTH);
	}

	/* GPIO for Motor control */
	config_gpio(&motor_power, 0, GPIO_DIR_OUT, 0);
	config_gpio(&motor_enable, 0, GPIO_DIR_OUT, 0);
	config_gpio(&motor_direction, 0, GPIO_DIR_OUT, 0);
	set_gpio_callback(motor_force, &isp, EDGES_BOTH);

	/* WS2812B Leds */
	ws2812_config(&ws2812_data_out_pin);

	/* RTC init */
	ret = rtc_pcf85363a_config(&rtc_conf);
	ret = rtc_pcf85363a_is_up(&rtc_conf, &oldest);
	if (ret == 1) {
		char buff[30];
		rtc_pcf85363_time_read(&rtc_conf, &now);
		rtc_pcf85363_time_to_str(&now, buff, 30);
		/* Debug */
		uprintf(UART0, buff);
	} else if (ret == -EFAULT) {
		memcpy(&now, &oldest, sizeof(struct rtc_time));
		rtc_pcf85363_time_write(&rtc_conf, &now);
	}

	/* microSD card init */
	/* FIX for v02 */
	config_gpio(&sclk_error, 0, GPIO_DIR_IN, 0);
	ret = sdmmc_init(&micro_sd);
	if (ret == 0) {
		msleep(1);
		ret = sdmmc_init_wait_card_ready(&micro_sd);
		if (ret == 0) {
			ret = sdmmc_init_end(&micro_sd);
		}
	}
	uprintf(UART0, "uSD init: %d, type: %d, bs: %d\n", ret, micro_sd.card_type, micro_sd.block_size);
	ret = sdmmc_read_block(&micro_sd, 0, mmc_data);
	uprintf(UART0, "uSD read: %s\n", mmc_data);

	uprintf(UART0, "E-Xanh Gardener started\n");
}

/***************************************************************************** */
volatile uint8_t current_position = 255;
static void pos_detect(uint32_t gpio)
{
	if (gpio_read(positions[(gpio - 22)]) == 0) {
		/* The sensor output is 0 when it detects a magnet */
		current_position = gpio - 22;
	} else {
		current_position = gpio - 12;
	}
}


volatile char env_buff[20];
volatile int idx = 0;
volatile int mode = 0;
void handle_cmd(uint8_t c)
{
	if (c == '#') {
		mode = 1; /* Raw data */
		env_buff[idx++] = c;
	} else if (mode == 1) {
		if (idx < 20) {
			env_buff[idx++] = c;
		}
		if (idx >= 20) {
			mode = 0;
		}
	} else {
		/* Echo ? */
		if (0) {
			serial_send_quickbyte(UART0, c);
		}
	}

}
volatile uint8_t motor_sens = 2;
volatile uint8_t motor_move_req = 0;
static void motor_force(uint32_t gpio)
{
	static int next_motor_sens = 1;
	if (gpio_read(isp) == 0) {
		motor_sens = next_motor_sens++;
		if (next_motor_sens > 2) {
			next_motor_sens = 1;
		}
		motor_move_req = 1;
	} else {
		motor_move_req = 0;
	}
}
volatile uint8_t pompe_cmd = 0;
static void pompe_force(uint32_t gpio)
{
	if (gpio_read(button) == 0) {
		pompe_cmd = 1;
	} else {
		pompe_cmd = 0;
	}
}

/***************************************************************************** */
int main(void)
{
	uint8_t old_position = 255;
	uint8_t old_pompe = 0;

	system_init();

	uart_on(UART0, 115200, handle_cmd);
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	ssp_master_on(SSP_BUS_0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000);
	adc_on(NULL);

	system_config();

	while (1) {
		tmp101_sensor_start_conversion(&tmp101_sensor);
		chenillard(700);
		if (current_position != old_position) {
			old_position = current_position;
			uprintf(UART0, "New position : %d\n", current_position);
			old_position = current_position;
		}
		if (old_pompe != pompe_cmd) {
			if (pompe_cmd == 1) {
				uprintf(UART0, "Pump On\n");
				gpio_set(pompe_on);
			} else {
				uprintf(UART0, "Pump Stopped\n");
				gpio_clear(pompe_on);
			}
			old_pompe = pompe_cmd;
		}
		if (motor_move_req == 1) {
			if (motor_sens == 1) {
				uprintf(UART0, "Motor moving forward\n");
				gpio_set(motor_direction);
			} else {
				uprintf(UART0, "Motor moving backward\n");
				gpio_clear(motor_direction);
			}
			msleep(1);
			/* Turn motor ON */
			gpio_set(motor_enable);
			gpio_set(motor_power);
		} else {
			/* turn motor back Off */
			gpio_clear(motor_power);
			gpio_clear(motor_enable);
		}
		/* Temp sensor */
		if (1) {
			int tmp101_deci_degrees = 0;
			int ret = 0;
			ret = tmp101_sensor_read(&tmp101_sensor, NULL, &tmp101_deci_degrees);
			if (ret != 0) {
				uprintf(UART0, "TMP101 read error : %d\n", ret);
			} else {
				int abs_deci = tmp101_deci_degrees;
				if (tmp101_deci_degrees < 0) {
					abs_deci = -tmp101_deci_degrees;
				}
				uprintf(UART0, "Internal Temp : % 4d.%02d\n", (tmp101_deci_degrees / 10), (abs_deci % 10));
			}
		}
		/* ADC Read */
		if (1) {
			uint16_t humidity_val = 0;
			int mv_humidity = 0;
			uint16_t batt_val = 0;
			int mv_batt = 0;
			adc_start_convertion_once(LPC_ADC(1), LPC_ADC_SEQ(0), 0);
			msleep(8);
			adc_get_value(&humidity_val, LPC_ADC(1));
			mv_humidity = ((humidity_val * 32) / 10);
			adc_start_convertion_once(LPC_ADC(4), LPC_ADC_SEQ(0), 0);
			adc_get_value(&batt_val, LPC_ADC(4));
			msleep(8);
			mv_batt = (batt_val * 16);  /*  = ((val * 32) / 10) * 5  (pont diviseur) */
			/* Display */
			uprintf(UART0, "Capteur: %d mV\n", mv_humidity);
			uprintf(UART0, "Batterie: %d.%03d V\n", (mv_batt/1000), ((mv_batt/10)%100));
			/* Set led */
			ws2812_set_pixel(0, 0, 0, (((mv_humidity - 1000) / 10) & 0xFF));
			ws2812_set_pixel(1, 10, 0, 10);
			ws2812_send_frame(0);
		}
		if (idx >= 20) {
			uint16_t* data = (uint16_t*)env_buff;
			uprintf(UART0, "From raw:\n");
			uprintf(UART0, "- Soil: %d\n", data[1]);
			uprintf(UART0, "- Lux: %d, IR: %d, UV: %d\n", data[2], data[3], data[4]);
			uprintf(UART0, "- Patm: %d hPa, Temp: %d,%02d degC, Humidity: %d,%d rH\n\n",
							data[5],
							data[6] / 10,  (data[6]> 0) ? (data[6] % 10) : ((-data[6]) % 10),
							data[7] / 10, data[7] % 10);
			idx = 0;
		}
	}
	return 0;
}



