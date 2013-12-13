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



/***************************************************************************** */
/*                SSP                                                          */
/***************************************************************************** */

#include <stdint.h>
#include "drivers/gpio.h"

#define SPI_USE_FIFO  1


/***************************************************************************** */
/* SPI slave select / activate
 * Note: Used only by non SPI functions to request the SSEL pin when SPI driver is used.
 */
void spi_cs_pull_low(void);
void spi_cs_release(void);


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
/* Write data to the SPI bus.
 * Any data received while sending is discarded.
 * Data will be read in the lower bits of the 16 bits values pointed by "data" for each frame.
 * size is the number of frames, each one having the configured data width (4 to 16 bits).
 * If use_fifo is 0 frames are received one at a time, otherise the fifo is used to leave
 *   as little time between frames as possible.
 */
int spi_write(uint16_t *data, int size, int use_fifo);

/* Read data from the SPI bus.
 * Data will be stored in the lower bits of the 16 bits values pointed by "data" for each frame.
 * size is the number of frames, each one having the configured data width (4 to 16 bits).
 * In order to activate the bus clock a default "all ones" frame is sent for each frame we
 *   want to read.
 * If use_fifo is 0 frames are received one at a time, otherise the fifo is used to leave
 *   as little time between frames as possible.
 */
int spi_read(uint16_t *data, int size, int use_fifo);


/***************************************************************************** */
uint32_t ssp_clk_on(uint32_t rate);
void ssp_clk_update(void);
void set_ssp_pins(void);



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
int ssp_master_on(uint8_t frame_type, uint8_t data_width, uint8_t spi_cs_spi, uint32_t rate);

int ssp_slave_on(uint8_t frame_type, uint8_t data_width, uint8_t out_en, uint32_t max_rate);

/* Turn off the SSP block */
void ssp_off(void);
