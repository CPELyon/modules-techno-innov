/****************************************************************************
 *  drivers/ssp.c
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
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



/***************************************************************************** */
/*                SSP                                                          */
/***************************************************************************** */

#include <stdint.h>
#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"
#include "core/system.h"
#include "lib/string.h"
#include "drivers/ssp.h"
#include "drivers/gpio.h"


/* Store our current SPI clock rate */
static uint32_t ssp_current_clk = 0;
/* Device configured ? */
static uint32_t spi_device_pins_in_use = 0;
/* Do we use SPI to handle chip-select or a GPIO ? */
static uint32_t use_spi_cs = 1;


/* Handlers */
void SSP_0_Handler(void)
{
	struct lpc_ssp* ssp = LPC_SSP0;
	uint32_t intr_flags = ssp->masked_int_status;

	/* Clear the interrupts. Other bits are cleared by fifo access */
	ssp->int_clear = (intr_flags & (LPC_SSP_INTR_RX_OVERRUN | LPC_SSP_INTR_RX_TIMEOUT));
}



/***************************************************************************** */
/* SPI SSEL */

/* SPI ssel mutex for SPI bus */
static uint32_t spi_cs_mutex = 0;
static void spi_get_cs_mutex(void)
{
	do {} while (sync_lock_test_and_set(&spi_cs_mutex, 1) == 1);
}
static void spi_release_cs_mutex(void)
{
	sync_lock_release(&spi_cs_mutex);
}

/* SSEL Alternative:
 * Set SPI Chip Select Low using GPIO (for I2C or to prevent CS going high between frames
 */
#define SPI_CS_PIN 15
void spi_cs_pull_low(void)
{
	struct lpc_gpio* gpio0 = LPC_GPIO_0;

	/* Get SPI chip select mutex */
	spi_get_cs_mutex();

	/* Configure pin as GPIO */
    io_config_clk_on();
	config_gpio(0, SPI_CS_PIN, (LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL));
    io_config_clk_off();
	/* Configure pin as output and set it low */
	gpio0->data_dir |= (1 << SPI_CS_PIN);
    gpio0->clear = (1 << SPI_CS_PIN);
}
void spi_cs_release(void)
{
	if (spi_device_pins_in_use == 1) {
    	io_config_clk_on();
		/* Configure pin as SPI */
		config_gpio(0, SPI_CS_PIN, (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL));
    	io_config_clk_off();
	} else {
		struct lpc_gpio* gpio0 = LPC_GPIO_0;
		/* Set pin high */
    	gpio0->set = (1 << SPI_CS_PIN);
	}
	spi_release_cs_mutex();
}


/***************************************************************************** */
/* This function is used to transfer a byte to AND from a device
 * As the SPI bus is full duplex, data can flow in both directions, but the clock is
 *   always provided by the master. The SPI clock cannont be activated without sending
 *   data, which means we must send data to the device when we want to read data from
 *   the device.
 */
uint16_t spi_transfer_single_frame(uint16_t data)
{
	struct lpc_ssp* ssp = LPC_SSP0;
	ssp->data = data;
	/* Wait until the busy bit is cleared */
	while (ssp->status & LPC_SSP_ST_BUSY);
	return ssp->data;
}

/* Note: there's no need to count Rx data as it is equal to Tx data */
int spi_transfer_multiple_frames(uint16_t* data_out, int size, uint16_t* data_in)
{
	struct lpc_ssp* ssp = LPC_SSP0;
	int count = 0;
	uint16_t data_drop = 0; /* Used to store unused SPI Rx data */

	do {
		/* Fill Tx fifo with available data, but stop if rx fifo is full */
		while ((count < size) &&
				((ssp->status & (LPC_SSP_ST_TX_NOT_FULL | LPC_SSP_ST_RX_FULL)) == LPC_SSP_ST_TX_NOT_FULL)) {
			ssp->data = *data_out;
			count++;
			data_out++;
		}

		/* Read some of the replies, but stop if there's still data to send and the fifo is running short */
		while ((ssp->status & LPC_SSP_ST_RX_NOT_EMPTY) &&
				!((count < size) && (ssp->raw_int_status & LPC_SSP_INTR_TX_HALF_EMPTY))) {
			if (data_in != NULL) {
				/* Store them ... */
				*data_in = ssp->data;
				data_in++;
			} else {
				/* ... or drop them */
				data_drop = ssp->data;
			}
		}
	/* Go on till both all data is sent and all data is received */
	} while ((count < size) || (ssp->status & (LPC_SSP_ST_BUSY | LPC_SSP_ST_RX_NOT_EMPTY)));

	return (data_drop - data_drop); /* 0, but then data_drop is used :) */
}


/* Handle diferent kind of chip-select methods.
 * Some devices require that the CS be maintained low between frames, a high on chip select
 *   meaning a reset on the SPI communication (MAX31855 thermocouple to digital converter for
 *   example).
 */
static void spi_cs_get(void)
{
	if (use_spi_cs == 1) {
		spi_get_cs_mutex();
	} else {
		spi_cs_pull_low();
	}
}
void spi_cs_put(void)
{
	if (use_spi_cs == 1) {
		spi_release_cs_mutex();
	} else {
		spi_cs_release();
	}
}
/* Write data to the SPI bus.
 * Any data received while sending is discarded.
 * Data will be read in the lower bits of the 16 bits values pointed by "data" for each frame.
 * size is the number of frames, each one having the configured data width (4 to 16 bits).
 * If use_fifo is 0 frames are received one at a time, otherise the fifo is used to leave
 *   as little time between frames as possible.
 */
int spi_write(uint16_t *data, int size, int use_fifo)
{
	int i;
	spi_cs_get();
	if (use_fifo == 0) {
		for (i = 0; i < size; i++) {
			spi_transfer_single_frame(*data);
			data++;
		}
	} else {
		spi_transfer_multiple_frames(data, size, NULL);
	}
	spi_cs_put();
	return size;
}

/* Read data from the SPI bus.
 * Data will be stored in the lower bits of the 16 bits values pointed by "data" for each frame.
 * size is the number of frames, each one having the configured data width (4 to 16 bits).
 * In order to activate the bus clock a default "all ones" frame is sent for each frame we
 *   want to read.
 * If use_fifo is 0 frames are received one at a time, otherise the fifo is used to leave
 *   as little time between frames as possible.
 */
int spi_read(uint16_t *data, int size, int use_fifo)
{
	int i;
	spi_cs_get();
	if (use_fifo == 0) {
		for (i = 0; i < size; i++) {
			*data = spi_transfer_single_frame(0xFFFF);
			data++;
		}
	} else {
		spi_transfer_multiple_frames(data, size, data);
	}
	spi_cs_put();
	return size;
}


/***************************************************************************** */
uint32_t ssp_clk_on(uint32_t rate)
{
	struct lpc_sys_control* sys_ctrl = LPC_SYS_CONTROL;
	struct lpc_ssp* ssp = LPC_SSP0;
	uint32_t prescale = 0, pclk_div = 0;
	uint32_t pclk = 0, div = 0;

	/* Do not divide by 0 */
	if (rate == 0) {
		return 0;
	}

	pclk = get_main_clock();

	/* Make sure we can get this clock rate */
	/* NOTE: We use only to divisors, so we could achieve lower clock rates by using
     *   the third one. Any need for this though ?
	 */
	if ((pclk / rate) > 0xFFF0) {
		/* What should we do ? */
		div = 0xFFF0;
	} else {
		div = pclk / rate;
	}

	do {
		prescale += 2; /* Minimum value is 2, and must be even */
		pclk_div = (div / prescale);
	} while ((prescale > 0xFF) || (pclk_div > 0xFF));

	/* Activate the SSP clock (maybe divide main clock) */
	sys_ctrl->ssp0_clk_div = pclk_div;

	/* Set the prescaler */
	ssp->clk_prescale = prescale;

	/* And return the achieved clock */
	return (pclk / (prescale * pclk_div));
}

void ssp_clk_update(void)
{
	if (ssp_current_clk) {
		ssp_clk_on(ssp_current_clk);
	}
}

void set_ssp_pins(void)
{
	/* Make sure IO_Config is clocked */
	io_config_clk_on();

	/* We need this as we share the chip-select with the I2C bus. */
	spi_device_pins_in_use = 1;

	/* SPI pins */
	config_gpio(0, 14, (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL)); /* SPI Clock */
	config_gpio(0, 15, (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL)); /* SPI Chip Select */
	config_gpio(0, 16, (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL)); /* SPI MISO */
	config_gpio(0, 17, (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL)); /* SPI MOSI */

	/* Config done, power off IO_CONFIG block */
	io_config_clk_off();
}




/***************************************************************************** */
/* SSP Setup as master */
/* Returns 0 on success
 * frame_type is SPI, TI or MICROWIRE (use apropriate defines for this one).
 * data_width is a number between 4 and 16.
 * spi_cs_spi: set to 0 handle Slave select as a GPIO and keep it loww during the whole
 *    data transaction (do not pull high between frame). This is required by some slave
 *    devices.
 * rate : The bit rate, in Hz.
 */
int ssp_master_on(uint8_t frame_type, uint8_t data_width, uint8_t spi_cs_spi, uint32_t rate )
{
	struct lpc_ssp* ssp = LPC_SSP0;

	NVIC_DisableIRQ(SSP0_IRQ);
	/* Power up the ssp block */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_SSP0, 1);

	/* Configure the SSP mode */
	ssp->ctrl_0 = LPC_SSP_DATA_WIDTH(data_width);
	ssp->ctrl_0 |= (frame_type | LPC_SSP_SPI_CLK_LOW | LPC_SSP_SPI_CLK_FIRST);
	ssp->ctrl_1 = LPC_SSP_MASTER_MODE;

	/* Configure the clock : done after basic configuration */
	ssp_current_clk = ssp_clk_on(rate);

	if (spi_cs_spi == 0) {
		use_spi_cs = 0;
	} else {
		use_spi_cs = 1;
	}

	/* Enable SSP */
	ssp->ctrl_1 |= LPC_SSP_ENABLE;

	NVIC_EnableIRQ(SSP0_IRQ);
	return 0; /* Config OK */
}

int ssp_slave_on(uint8_t frame_type, uint8_t data_width, uint8_t out_en, uint32_t max_rate)
{
	struct lpc_ssp* ssp = LPC_SSP0;

	NVIC_DisableIRQ(SSP0_IRQ);
	/* Power up the ssp block */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_SSP0, 1);

	/* Configure the SSP mode */
	ssp->ctrl_0 = LPC_SSP_DATA_WIDTH(data_width);
	ssp->ctrl_0 |= (frame_type | LPC_SSP_SPI_CLK_LOW | LPC_SSP_SPI_CLK_FIRST);
	ssp->ctrl_1 = LPC_SSP_SLAVE_MODE;
	if (!out_en) {
		ssp->ctrl_1 |= LPC_SSP_SLAVE_OUT_DISABLE;
	}

	/* Configure the clock : done after basic configuration.
	 * Our clock must be at least 12 times the master clock */
	ssp_current_clk = ssp_clk_on(max_rate * 16);

	/* Enable SSP */
	ssp->ctrl_1 |= LPC_SSP_ENABLE;

	NVIC_EnableIRQ(SSP0_IRQ);
	return 0; /* Config OK */

}

/* Turn off the SSP block */
void ssp_off(void)
{
	ssp_current_clk = 0;
	NVIC_DisableIRQ(SSP0_IRQ);
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_SSP0, 0);
}
