/****************************************************************************
 *   drivers/i2c.h
 *
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef DRIVERS_I2C_H
#define DRIVERS_I2C_H

#include <stdint.h>


#define I2C_CLK_100KHz  (100*1000)
#define I2C_CLK_400KHz  (400*1000)

#define I2C_READ_BIT 0x01
#define I2C_WRITE_BIT 0x00

enum i2c_conditions {
	I2C_CONT = 0,
	I2C_DO_REPEATED_START,
	I2C_DO_STOP_START,
	I2C_STOP,
};

enum i2c_states {
	/* Must be set before starting the state machine to be able to wait for completion. */
	I2C_BUSY = 0,
	I2C_OK, /* All right, default state after init has been done. */
	/* Sent slave address and it got ACK'ed, but no data.
	 *  Used to probe or wait for completion. */
	I2C_NO_DATA,
	I2C_NACK, /* NACK received */
	I2C_TIME_OUT,
	I2C_ARBITRATION_LOST, /* Another master took the I2C Bus from us ... */
	I2C_BUS_ERROR, /* Illegal start or stop (status of 0x00) */
	I2C_ERROR_UNKNOWN,
};

enum i2c_state_machine_states {
	/* Error states */
	I2C_ILLEGAL = 0x00, /* Illegal start or stop */
	I2C_ARBIT_LOST = 0x38,
	/* Start condition states */
	I2C_STARTED = 0x08,
	I2C_RESTARTED, /* Unused, should be set when restarting (STOP+START) after a NACK */
	I2C_REPEATED_START = 0x10,
	/* Transmitter states */
	I2C_ACK_ON_ADDRESS_W = 0x18,
	I2C_NACK_ON_ADDRESS_W = 0x20,
	I2C_ACK_ON_DATA_W = 0x28,
	I2C_NACK_ON_DATA_W = 0x30,
	/* Receiver states */
	I2C_ACK_ON_ADDRESS_R = 0x40,
	I2C_NACK_ON_ADDRESS_R = 0x48,
	I2C_DATA_ACK = 0x50,
	I2C_DATA_NACK = 0x58,
};




/***************************************************************************** */
/*                Modules I2C access                                           */
/***************************************************************************** */
/* I2C Read
 * Performs a non-blocking read on the i2c bus.
 *   cmd_buf : buffer containing all control byte to be sent on the i2c bus
 *   cmd_size : size of the cmd_buf command buffer
 *   ctrl_buf : buffer containing action to be done after sending, like repeated START conditions
 *         ctrl_buf has the same size as cmd_buf
 *   inbuff : the buffer where read data will be put.
 *   count : the number of bytes to be read.
 * RETURN VALUE
 *   Upon successfull completion, returns the number of bytes read. On error, returns a negative
 *   integer equivalent to errors from glibc.
 *   -EBADFD : Device not initialized
 *   -EBUSY : Device or ressource Busy or Arbitration lost
 *   -EAGAIN : Device already in use
 *   -EINVAL : Invalid argument (buf)
 *   -EREMOTEIO : Device did not acknowledge
 *   -EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 */
int i2c_read(const void *cmd_buf, size_t cmd_size, const void* ctrl_buf, void* inbuff, size_t count);

/* I2C Write
 * Performs a non-blocking write on the i2c bus.
 *   buf : buffer containing all byte to be sent on the i2c bus,
 *         including conrtol bytes (address, offsets, ...)
 *   count : the number of bytes to be sent, including address bytes and so on.
 *   ctrl_buf : buffer containing action to be done after sending, like repeated START conditions
 *         ctrl_buf has the same size as cmd_buf
 *         FIXME : note that STOP + START conditions are not allowed, the STOP would lead to sending
 *         the first bytes of buf, creating an infinite loop.
 * RETURN VALUE
 *   Upon successfull completion, returns the number of bytes written. On error, returns a negative
 *   integer equivalent to errors from glibc.
 *   -EBADFD : Device not initialized
 *   -EBUSY : Device or ressource Busy or Arbitration lost
 *   -EAGAIN : Device already in use
 *   -EINVAL : Invalid argument (buf)
 *   -EREMOTEIO : Device did not acknowledge
 *   -EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 */
int i2c_write(const void *buf, size_t count, const void* ctrl_buf);




/***************************************************************************** */
/*                I2C Init                                                     */
/***************************************************************************** */
void i2c_on(uint32_t i2c_clk_freq);
void i2c_off(void);
/* Allow system to propagate main clock */
void i2c_clock_update(void);



#endif /* DRIVERS_I2C_H */
