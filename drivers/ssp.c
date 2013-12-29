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


/* Handlers */
void SSP_0_Handler(void)
{
	struct lpc_ssp* ssp = LPC_SSP0;
	uint32_t intr_flags = ssp->masked_int_status;

	/* Clear the interrupts. Other bits are cleared by fifo access */
	ssp->int_clear = (intr_flags & (LPC_SSP_INTR_RX_OVERRUN | LPC_SSP_INTR_RX_TIMEOUT));
}



/***************************************************************************** */
/* SPI Bus mutex */

/* SPI global mutex for SPI bus */
static uint32_t spi_mutex = 0;

#if MULTITASKING == 1
int spi_get_mutex(void)
{
	/* Note : a multitasking OS should call the scheduler here. Any other system
	 *   will get frozen, unless some interrupt routine has been set to release
	 *   the mutex.
	 */
	do {} while (sync_lock_test_and_set(&spi_mutex, 1) == 1);
	return 1;
}
#else
int spi_get_mutex(void)
{
	if (sync_lock_test_and_set(&spi_mutex, 1) == 1) {
		return -EBUSY;
	}
	return 1;
}
#endif
void spi_release_mutex(void)
{
	sync_lock_release(&spi_mutex);
}

/***************************************************************************** */
/* SPI device SSEL:
 * Set SPI Chip Select Low using GPIO mode when SPI driver is in use
 */

/* SPI slave select mutex for SPI bus */
static uint32_t spi_cs_mutex = 0;

/* Use this function when you need to enable access to the module EEPROM on the
 * GPIO Demo Module from Techno-Innov but the micro-controller acts as a slave on the
 * SPI bus.
 * Any data sent by the master on the SPI bus while the spi_cs_mutex is held will be lost.
 */
int spi_device_cs_pull_low(void)
{
	struct lpc_gpio* gpio0 = LPC_GPIO_0;

	if (sync_lock_test_and_set(&spi_cs_mutex, 1) == 1) {
		return -EBUSY;
	}

	/* Configure pin as GPIO */
	config_gpio(0, SPI_CS_PIN, (LPC_IO_FUNC_ALT(0) | LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL));
	/* Configure pin as output and set it low */
	gpio0->data_dir |= (1 << SPI_CS_PIN);
    gpio0->clear = (1 << SPI_CS_PIN);

	return 0;
}
void spi_device_cs_release(void)
{
	struct lpc_gpio* gpio0 = LPC_GPIO_0;
	/* Set pin high */
    gpio0->set = (1 << SPI_CS_PIN);
	/* Configure pin as SPI again */
	config_gpio(0, SPI_CS_PIN, (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL)); /* SPI Chip Select */
}



/***************************************************************************** */
/* This function is used to transfer a word (4 to 16 bits) to AND from a device
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

/* Multiple words (4 to 16 bits) transfer function on the SPI bus.
 * The SSP fifo is used to leave as little time between frames as possible.
 * Parameters :
 *  size is the number of frames, each one having the configured data width (4 to 16 bits).
 *  data_out : data to be sent. Data will be read in the lower bits of the 16 bits values
 *             pointed by "data_out" for each frame. If NULL, then the content of data_in
               will be used.
 *  data_in : buffer for read data. If NULL, read data will be discarded.
 * As the SPI bus is full duplex, data can flow in both directions, but the clock is
 *   always provided by the master. The SPI clock cannont be activated without sending
 *   data, which means we must send data to the device when we want to read data from
 *   the device.
 * This function does not take care of the SPI chip select.
 * Note: there's no need to count Rx data as it is equal to Tx data
 */
int spi_transfer_multiple_frames(uint16_t* data_out, int size, uint16_t* data_in)
{
	struct lpc_ssp* ssp = LPC_SSP0;
	int count = 0;
	uint16_t data_read = 0; /* Used to store SPI Rx data */

	/* Did the user provide a buffer to send data ? */
	if (data_out == NULL) {
		if (data_in == NULL) {
			return 0;
		}
		data_out = data_in;
	}
	/* Transfer */
	do {
		/* Fill Tx fifo with available data, but stop if rx fifo is full */
		while ((count < size) &&
				((ssp->status & (LPC_SSP_ST_TX_NOT_FULL | LPC_SSP_ST_RX_FULL)) == LPC_SSP_ST_TX_NOT_FULL)) {
			ssp->data = *data_out;
			count++;
			data_out++;
		}

		/* Read some of the replies, but stop if there's still data to send and the fifo
		 *  is running short */
		while ((ssp->status & LPC_SSP_ST_RX_NOT_EMPTY) &&
				!((count < size) && (ssp->raw_int_status & LPC_SSP_INTR_TX_HALF_EMPTY))) {
			/* Read the data (mandatory) */
			data_read = ssp->data;
			if (data_in != NULL) {
				/* And store when requested */
				*data_in = data_read;
				data_in++;
			}
		}
	/* Go on till both all data is sent and all data is received */
	} while ((count < size) || (ssp->status & (LPC_SSP_ST_BUSY | LPC_SSP_ST_RX_NOT_EMPTY)));

	return count;
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

/* Configure main SPI pins, used in both master and device mode.
 * The slave select is not configured here as it's use is implementation dependant in master
 *  mode. In slave mode it is configured in the ssp_slave_on() function.
 */
void set_ssp_pins(void)
{
	/* Main SPI pins */
	config_gpio(0, 14, (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL)); /* SPI Clock */
	config_gpio(0, 16, (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL)); /* SPI MISO */
	config_gpio(0, 17, (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL)); /* SPI MOSI */
}




/***************************************************************************** */
/* SSP Setup as master */
/* Returns 0 on success
 * Parameters :
 *  frame_type is SPI, TI or MICROWIRE (use apropriate defines for this one).
 *  data_width is a number between 4 and 16.
 *  rate : The bit rate, in Hz.
 * The SPI Chip Select is not handled by the SPI driver in master SPI mode as it's
 *   handling highly depends on the device on the other end of the wires. Use a GPIO
 *   to activate the device's chip select (usually active low)
 */
int ssp_master_on(uint8_t frame_type, uint8_t data_width, uint32_t rate )
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

	/* Enable SSP */
	ssp->ctrl_1 |= LPC_SSP_ENABLE;

	NVIC_EnableIRQ(SSP0_IRQ);
	return 0; /* Config OK */
}

int ssp_slave_on(uint8_t frame_type, uint8_t data_width, uint8_t out_en, uint32_t max_rate)
{
	struct lpc_ssp* ssp = LPC_SSP0;

	/* Is the slave select pin available ?
	 * We need this as we share the chip-select with the I2C bus. */
	if (sync_lock_test_and_set(&spi_cs_mutex, 1) == 1) {
		return -EBUSY;
	}

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

	/* Use SPI as Device : configure the SSEL pin */
	config_gpio(0, SPI_CS_PIN, (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL)); /* SPI Chip Select */

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
	/* Can be done even if we don't hold the mutex */
	sync_lock_release(&spi_cs_mutex);
}

