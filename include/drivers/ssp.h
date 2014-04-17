/****************************************************************************
 *  drivers/ssp.h
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

#ifndef DRIVERS_SSP_H
#define DRIVERS_SSP_H


/***************************************************************************** */
/*                SSP                                                          */
/***************************************************************************** */

#include <stdint.h>


/* Set this to 1 for use of this driver in a multitasking OS, it will activate the SPI Mutex */
#define MULTITASKING 0

enum spi_fifo_or_pooling {
	SPI_NO_FIFO = 0,
	SPI_USE_FIFO = 1,
};


/***************************************************************************** */
/* SPI device mode slave select sharing.
 * Note: Used only by non SPI functions to request the SSEL pin when SPI driver is used.
 */
int spi_device_cs_pull_low(void);
void spi_device_cs_release(void);


/***************************************************************************** */
/* SPI Bus mutex */
/* In multitasking environment spi_get_mutex will block until mutex is available and always
 *  return 1. In non multitasking environments spi_get_mutex returns either 1 (got mutex)
 *  or -EBUSY (SPI bus in use)
 */
int spi_get_mutex(void);
void spi_release_mutex(void);


/***************************************************************************** */
/* This function is used to transfer a byte to AND from a device
 * As the SPI bus is full duplex, data can flow in both directions, but the clock is
 *   always provided by the master. The SPI clock cannont be activated without sending
 *   data, which means we must send data to the device when we want to read data from
 *   the device.
 * This function does not take care of the SPI chip select.
 */
uint16_t spi_transfer_single_frame(uint16_t data);


/***************************************************************************** */
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
int spi_transfer_multiple_frames(uint16_t* data_out, int size, uint16_t* data_in);


/***************************************************************************** */
uint32_t ssp_clk_on(uint32_t rate);
void ssp_clk_update(void);

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
int ssp_master_on(uint8_t frame_type, uint8_t data_width, uint32_t rate);

int ssp_slave_on(uint8_t frame_type, uint8_t data_width, uint8_t out_en, uint32_t max_rate);

/* Turn off the SSP block */
void ssp_off(void);


#endif /* DRIVERS_SSP_H */

