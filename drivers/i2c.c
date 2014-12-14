/****************************************************************************
 *   drivers/i2c.c
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


/**************************************************************************** */
/*                      I2C                                                   */
/**************************************************************************** */

#include <stdint.h>

#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"
#include "core/system.h"
#include "core/pio.h"
#include "lib/string.h"
#include "drivers/i2c.h"


/*
 * I2C Bus structure
 *
 * clock : current i2c clock.
 * state : global state of the i2c engine.
 * master_status : status returned by i2c block as found in "status" register.
 *
 * repeated_start_restart : This buffer, if used, must have the SAME size as out_buff.
 *           Then, instead of simply moving to the next byte, the corresponding condition
 *           will be executed : I2C_CONT, I2C_DO_REPEATED_START, I2C_DO_STOP_START or I2C_STOP.
 * restart_after_addr : Can be used instead of repeated_start_restart buffer to perform a
 *           single repeated start or stop/start sequence after first adress got sent.
 * restart_after_data : Can be used instead of repeated_start_restart buffer to perform a
 *           single repeated start or stop/start sequence after given data byte.
 */
struct i2c_bus {
	volatile struct lpc_i2c* regs;
	volatile uint32_t clock;
	volatile uint32_t state;
	volatile uint32_t master_status;
	volatile uint32_t slave_status;
	volatile uint32_t timeout;

	volatile const char* out_buff;
	volatile const char* repeated_start_restart;
	volatile uint32_t write_length;
	volatile uint32_t write_index;
	volatile uint32_t restart_after_addr;
	volatile uint32_t restart_after_data;

	volatile char* in_buff;
	volatile uint32_t read_length;
	volatile uint32_t read_index;
};

static struct i2c_bus mod_i2c;


/* FIXME : Should add a field "what to do on addr NACK" to i2c_bus structure, and use
   it with the timeout to create a retry mechanism */
/* FIXME : For case 58 ... What would be the use of a restart ?? perform periodic reads ? */
/* FIXME : Implement Slave when arbitration lost ? */



/* I2C Interrupt handler */
/* Actual version will stop on NACKs */
/* See LPC1764 user's manual UM10360 on page 457 (19.9.5) for details on I2C State machine */
void I2C_0_Handler(void)
{
	uint8_t status;
	struct i2c_bus* i2c = &mod_i2c;

	i2c->timeout = 0;

	/* this handler deals with master read and master write only */
	status = (i2c->regs->status & 0xFF);
	i2c->master_status = status; /* Store current status */

	/* I2C State Machine ! */
	switch (status) {

	/* All modes */
		case 0x00: /* Bus Error. Enter not addressed Slave mode and release bus. */
			i2c->regs->ctrl_set = (I2C_ASSERT_ACK | I2C_STOP_FLAG);
			i2c->state = I2C_BUS_ERROR;
			break;
		case 0x08:  /* A START condition has been transmitted. */
			i2c->write_index = 0;
			if (i2c->write_length != 0) {
				/* Send Slave Address (SLA)
				 * Depending on R/W bit, Master Receive or master Transmit mode will be enterred. */
				i2c->regs->data = i2c->out_buff[i2c->write_index++];
				i2c->regs->ctrl_clear = I2C_START_FLAG;
			} else {
				i2c->regs->ctrl_clear = I2C_START_FLAG;
				i2c->regs->ctrl_set = I2C_STOP_FLAG;
				i2c->state = I2C_NO_DATA;
			}
			break;
		case 0x10:  /* A repeated START has been transmitted. */
			/* Setting read_index to 0 is usefull only if next data byte is
			 *    Slave Address + Read (SLA + R), but it's OK if we perform a write too, and
			 *    is necessary for read with a null data length used to probe for devices.
			 */
			i2c->read_index = 0;
			/* Send Slave Address and Read/Write bit (SLA + R/W)
			 * Depending on R/W bit, Master Receive or master Transmit mode will be enterred. */
			i2c->regs->data = i2c->out_buff[i2c->write_index++];
			/* FIXME : Shouldn't this be done only in receiver mode (SLA + R) ? */
			i2c->regs->ctrl_clear = I2C_START_FLAG;
			break;
		case 0x38: /* Arbitration lost. We don't deal with multiple master situation */
			/* FIXME : We have two option :
			 *  - do nothing, which will release the bus. Will lead to "Not Addressed Slave" state.
			 *      This options allows handling of multiple master topologies.
			 *  - Set start flag, and a start condition will be transmitted when bus becomes free.
			 */
			/* We choose to do nothing, we ARE the true master after all ! */
			i2c->state = I2C_ARBITRATION_LOST;
			break;

	/* Master Transmitter Mode Only */
		case 0x18:  /* Address + Write transmitted and ACK'ed */
			if (i2c->write_length == 1) { /* Nothing more to send, we must stop. */
				/* This one is used to probe devices, or wait for completion */
				i2c->regs->ctrl_set = I2C_STOP_FLAG;
				i2c->state = I2C_NO_DATA;
			} else {
				uint32_t condition = i2c->restart_after_addr;
				if (i2c->repeated_start_restart)
					condition = i2c->repeated_start_restart[i2c->write_index - 1];
				switch (condition) {
					case I2C_CONT: /* Send next data byte */
						i2c->regs->data = i2c->out_buff[i2c->write_index++];
						break;
					case I2C_DO_REPEATED_START: /* Send repeated start condition */
						i2c->regs->ctrl_set = I2C_START_FLAG;
						break;
					case I2C_DO_STOP_START: /* Send STOP / START condition */
						i2c->regs->ctrl_set = I2C_STOP_FLAG | I2C_START_FLAG;
						break;
					case I2C_STOP: /* Send STOP condition */
						i2c->regs->ctrl_set = I2C_STOP_FLAG;
						i2c->state = I2C_NO_DATA;
						break;
				}
				if (i2c->restart_after_addr)
					i2c->restart_after_addr = 0; /* This one must be defused once used */
			}
			break;
		case 0x20:  /* NACK on Address + Write (SLA + W) */
		case 0x30:  /* NACK on Data byte */
			/* FIXME : We have three other options : Resend / Repeated START / STOP + START
			 *     If I'm right, Resend would require moving write_index back and using same data
			 *     Repeated START ? ignore the NACK and move to reading data ?
			 *     STOP + START ? Start over ? write_index will be set to 0 in I2C_STARTED case (0x08)
			 * Sending a STOP condition will end transactions and let the driver take the decision.
			 */
			i2c->regs->ctrl_set = I2C_STOP_FLAG;
			i2c->state = I2C_NACK;
			break;
		case 0x28: /* Data byte has been transmitted and ACK received */
			if (i2c->write_index < i2c->write_length) {
				/* More to write, but what do we do ? */
				uint32_t condition = I2C_CONT;
				if (i2c->restart_after_data == i2c->write_index)
					condition = I2C_DO_REPEATED_START;
				else if (i2c->repeated_start_restart)
					condition = i2c->repeated_start_restart[i2c->write_index - 1];
				switch (condition) {
					case I2C_CONT: /* Send next data byte */
						i2c->regs->data = i2c->out_buff[i2c->write_index++];
						break;
					case I2C_DO_REPEATED_START: /* Send repeated start condition */
						i2c->regs->ctrl_set = I2C_START_FLAG;
						break;
					case I2C_DO_STOP_START: /* Send STOP / START condition */
						i2c->regs->ctrl_set = I2C_STOP_FLAG | I2C_START_FLAG;
						break;
					case I2C_STOP: /* Send STOP condition */
						/* Hey, why sending a STOP and terminate communication if we are not at
						 * the end of the write buffer ?? */
						i2c->regs->ctrl_set = I2C_STOP_FLAG;
						i2c->state = I2C_OK;
						break;
				}
			} else {
				if (i2c->read_length != 0) { /* Anything to read ? */
					i2c->regs->ctrl_set = I2C_START_FLAG;
				} else {
					i2c->regs->ctrl_set = I2C_STOP_FLAG;
					i2c->state = I2C_OK;
				}
			}
			break;

	/* Master Receiver Mode Only */
		case 0x40:  /* Address + Read transmitted and ACK'ed */
			if ((i2c->read_index + 1) < i2c->read_length) {
				/* Will go to State 0x50 (assert ACK after data is received) */
				i2c->regs->ctrl_set = I2C_ASSERT_ACK;
			} else {
				/* Will go to State 0x58 (NACK after data is received) */
				i2c->regs->ctrl_clear = I2C_ASSERT_ACK;
			}
			break;

		case 0x48:  /* NACK on Address + Read (SLA + R) */
			/* FIXME : We have two other options : Repeated START / STOP + START */
			i2c->regs->ctrl_set = I2C_STOP_FLAG;
			i2c->state = I2C_NACK;
			break;

		case 0x50: /* Data byte has been received and ACK sent */
			i2c->in_buff[i2c->read_index++] = i2c->regs->data;
			if ((i2c->read_index + 1) < i2c->read_length) {
				/* assert ACK after data is received, requesting next Data from slave */
				i2c->regs->ctrl_set = I2C_ASSERT_ACK;
			} else {
				/* Assert NACK on last byte, telling the slave to stop transmitting data
				   and release the I2C Bus */
				i2c->regs->ctrl_clear = I2C_ASSERT_ACK;
			}
			break;

		case 0x58: /* Data byte has been received and NACK "sent" */
			/* This tells the slave it was the last byte. We should be done. */
			i2c->in_buff[i2c->read_index++] = i2c->regs->data;
			/* FIXME : We have two other options : Repeated START or STOP + START,
			 *    but what for ? periodic reads ? */
			i2c->regs->ctrl_set = I2C_STOP_FLAG;
			i2c->state = I2C_OK;
			break;

		default:
			i2c->state = I2C_ERROR_UNKNOWN;
			break;
	}

	/* Clear interrupt flag. This has to be done last. */
	i2c->regs->ctrl_clear = I2C_INTR_FLAG;
	return;
}



/***************************************************************************** */
/*                        I2C access                                           */
/***************************************************************************** */
/* Read
 * Performs a non-blocking read on the module's i2c bus.
 *   cmd_buf : buffer containing all control byte to be sent on the i2c bus
 *   cmd_size : size of the cmd_buf command buffer
 *   ctrl_buf : buffer containing action to be done after sending, like repeated START conditions
 *         if not NULL, ctrl_buf has the same size as cmd_buf
 *   inbuff : the buffer where read data will be put. May be NULL if count is 0.
 *   count : the number of bytes to be read.
 * RETURN VALUE
 *   Upon successfull completion, returns the number of bytes read. On error, returns a negative
 *   integer equivalent to errors from glibc.
 *   -EBADFD : Device not initialized (-77)
 *   -EBUSY : Device or ressource Busy or Arbitration lost (-16)
 *   -EAGAIN : Device already in use (-11)
 *   -EINVAL : Invalid argument (no cmd_buf, or no inbuff when count > 0) (-22)
 *   -EREMOTEIO : Device did not acknowledge (-121)
 *   -EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine (-5)
 */
int i2c_read(const void *cmd_buf, size_t cmd_size, const void* ctrl_buf, void* inbuff, size_t count)
{
	int ret = 0;

	/* Checks */
	if (mod_i2c.regs != LPC_I2C0)
		return -EBADFD;
	if (cmd_buf == NULL)
		return -EINVAL;
	if ((inbuff == NULL) && (count > 0))
		return -EINVAL;

	if (mod_i2c.state == I2C_BUSY) {
		return -EBUSY;
	}
	if (mod_i2c.state != I2C_OK) {
		/* What should we do ??? someone failed to reset status ? */
	}

	/* Set up mod_i2c for read operation */
	/* command (write) buffer */
	mod_i2c.out_buff = cmd_buf;
	mod_i2c.write_length = cmd_size;
	/* control buffer, if any. Note that it's the only way to control
	 *   operations on modules i2C bus to simplify the interface */
	mod_i2c.repeated_start_restart = ctrl_buf;
	mod_i2c.restart_after_addr = I2C_CONT;
	mod_i2c.restart_after_data = 0;
	/* read buffer */
	mod_i2c.in_buff = inbuff;
	mod_i2c.read_length = count;
	mod_i2c.read_index = 0;

	/* Start the process */
	mod_i2c.state = I2C_BUSY;
	mod_i2c.regs->ctrl_set = I2C_START_FLAG;
	/* And wait for process completion */
	do {} while (mod_i2c.state == I2C_BUSY);

	/* Handle returned state (errors or OK) */
	switch (mod_i2c.state) {
		case I2C_OK:
		case I2C_NO_DATA:
			ret = mod_i2c.read_index;
			break;
		case I2C_NACK:
			ret = -EREMOTEIO;
			break;
		case I2C_ARBITRATION_LOST:
			ret = -EBUSY;
			break;
		case I2C_BUS_ERROR: /* This one is bad ... */
		case I2C_ERROR_UNKNOWN:
		default:
			ret = -EIO;
			break;
	}

	/* Force device to release the bus :
	 *    send a START followed by a STOP (initiate transmission with nul write_length) */
	mod_i2c.state = I2C_BUSY;
	mod_i2c.write_length = 0;
	mod_i2c.regs->ctrl_set = I2C_START_FLAG;
	do {} while (mod_i2c.state == I2C_BUSY);
	mod_i2c.state = I2C_OK;

	return ret;
}

/* Write
 * Performs a non-blocking write on the module's i2c bus.
 *   buf : buffer containing all byte to be sent on the i2c bus,
 *         including conrtol bytes (address, offsets, ...)
 *   count : the number of bytes to be sent, including address bytes and so on.
 *   ctrl_buf : buffer containing action to be done after sending, like repeated START conditions
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
int i2c_write(const void *buf, size_t count, const void* ctrl_buf)
{
	int ret = 0;

	/* Checks */
	if (mod_i2c.regs != LPC_I2C0)
		return -EBADFD;
	if (buf == NULL)
		return -EINVAL;

	if (mod_i2c.state == I2C_BUSY) {
		return -EBUSY;
	}
	if (mod_i2c.state != I2C_OK) {
		/* What should we do ??? someone failed to reset status ? */
	}

	/* Clear read information to prevent entering master receiver states */
	mod_i2c.read_length = 0;
	mod_i2c.in_buff = NULL;
	mod_i2c.state = I2C_BUSY;
	/* Set up mod_i2c for write operation */
	mod_i2c.out_buff = buf;
	mod_i2c.write_length = count;
	/* control buffer, if any. Note that it's the only way to control
	 *   operations on modules i2C bus to simplify the interface */
	mod_i2c.restart_after_addr = I2C_CONT;
	mod_i2c.repeated_start_restart = ctrl_buf;
	mod_i2c.restart_after_data = 0;

	/* Start the process */
	mod_i2c.state = I2C_BUSY;
	mod_i2c.regs->ctrl_set = I2C_START_FLAG;
	/* Wait for process completion */
	do {} while (mod_i2c.state == I2C_BUSY);

	/* Handle returned state (errors or OK) */
	switch (mod_i2c.state) {
		case I2C_OK:
		case I2C_NO_DATA:
			ret = mod_i2c.write_length;
			break;
		case I2C_NACK:
			ret = -EREMOTEIO;
			break;
		case I2C_ARBITRATION_LOST:
			ret = -EBUSY;
			break;
		case I2C_BUS_ERROR: /* This one is bad ... */
		case I2C_ERROR_UNKNOWN:
		default:
			ret = -EIO;
			break;
	}

	/* Force device to release the bus :
	 *    send a START followed by a STOP (initiate transmission with nul write_length) */
	mod_i2c.state = I2C_BUSY;
	mod_i2c.write_length = 0;
	mod_i2c.regs->ctrl_set = I2C_START_FLAG;
	do {} while (mod_i2c.state == I2C_BUSY);
	mod_i2c.state = I2C_OK;

	return ret;
}



/***************************************************************************** */
/*                I2C Init                                                     */
/***************************************************************************** */
static void i2c_clock_on(uint32_t i2c_clk_freq)
{
	struct lpc_i2c* i2c = LPC_I2C0;
	uint32_t main_clock = get_main_clock();
	uint32_t scl_clk = 0;

	/* Setup I2C clock */
	scl_clk = (main_clock / i2c_clk_freq);
	i2c->clk_duty_high = (scl_clk / 2);
	i2c->clk_duty_low = (scl_clk - i2c->clk_duty_high);
}

void i2c_on(uint32_t i2c_clk_freq)
{
	struct lpc_i2c* i2c = LPC_I2C0;

	NVIC_DisableIRQ(I2C0_IRQ);
	/* Power on I2C 0 block */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_I2C, 1);
	/* Set clock */
	i2c_clock_on(i2c_clk_freq);
	mod_i2c.clock = i2c_clk_freq;
	/* Initialize i2c_bus struct */
	memset(&mod_i2c, 0, sizeof(struct i2c_bus));
	mod_i2c.regs = (struct lpc_i2c*)LPC_I2C0;
	mod_i2c.state = I2C_OK;
	/* Enable I2C */
	/* FIXME: if enabling slave functions, add I2C_ASSERT_ACK flag */
	i2c->ctrl_set = (I2C_ENABLE_FLAG);
	/* And enable interrupts */
	NVIC_EnableIRQ(I2C0_IRQ);
}

void i2c_off(void)
{
	NVIC_DisableIRQ(I2C0_IRQ);
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_I2C, 0);
	mod_i2c.clock = 0;
}

/* Allow system to propagate main clock */
void i2c_clock_update(void)
{
	if (mod_i2c.clock) {
		/* FIXME : we should stop I2C transfers, disable I2C interrupts and stop I1C clock. */
		i2c_clock_on(mod_i2c.clock); /* 6 is not a module num, nor system i2c (5) */
	}
}


