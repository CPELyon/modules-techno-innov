/****************************************************************************
 *   drivers/temp.c
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

#include <stdint.h>

#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"
#include "core/system.h"
#include "drivers/i2c.h"
#include "drivers/temp.h"


/***************************************************************************** */
/*          Support for TMP101 temperature sensors from Texas Instrument       */
/***************************************************************************** */
/* This driver is made for the TMP101 version of the chip, though there's few
 * diferences between the TMP100 and TMP101 version.
 * This driver does not handle the SMBus Alert command.
 */


/* Config */
#define TMP101_ADDR  0x94

enum tmp10x_internal_reg_numbers {
	TMP_REG_TEMPERATURE = 0,
	TMP_REG_CONFIG,
	TMP_REG_ALERT_LOW,
	TMP_REG_ALERT_HIGH,
};


/* Aditional defines, not exported to userspace */
/* Mode config */
#define TMP_SHUTDOWN_MODE_ON            (1 << 0)
#define TMP_THERMOSTAT_COMPARATOR_MODE  (0 << 1)
#define TMP_THERMOSTAT_INTERRUPT_MODE   (1 << 1)
/* Alert signal polarity */
#define TMP_ALERT_POLARITY_LOW          (0 << 2)
#define TMP_ALERT_POLARITY_HIGH         (1 << 2)
/* One-shot measurement trigger */
#define TMP_ONE_SHOT_TRIGGER            (1 << 7)



/* This static value is used to keep track of the last register accessed to
 * prevent sending the pointer register again if we want to read the same
 * register again. */
static int last_accessed_register = 0;

/* Check the sensor presence, return 1 if sensor was found.
 * This is a basic check, it could be anything with the same address ...
 */
int probe_sensor(void)
{
	static int ret = -1;
	char cmd_buf[1] = { (TMP101_ADDR | I2C_READ_BIT), };

	/* Did we already probe the sensor ? */
	if (ret != 1) {
		ret = i2c_read(cmd_buf, 1, NULL, NULL, 0);
	}
	return ret;
}

/* Convert raw temperature data (expressed as signed value of 16 times the
 * actual temperature in the twelve higher bits of a sixteen bits wide value)
 * into a decimal integer value of ten times the actual temperature.
 * The value returned is thus in tenth of degrees centigrade.
 */
int convert_to_deci_degrees(uint16_t raw)
{
	return (((int16_t)raw * 10) >> 8);
}

/* Temp Read
 * Performs a non-blocking read of the temperature from the sensor.
 * RETURN VALUE
 *   Upon successfull completion, returns 0 and the temperature read is placed in the
 *   provided integer. On error, returns a negative integer equivalent to errors from glibc.
 *   -EBADFD : I2C not initialized
 *   -EBUSY : Device or ressource Busy or Arbitration lost
 *   -EINVAL : Invalid argument (buf)
 *   -ENODEV : No such device
 *   -EREMOTEIO : Device did not acknowledge : Any device present ?
 *   -EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 */
#define CMD_BUF_SIZE 3
int temp_read(uint16_t* raw, int* deci_degrees)
{
	int ret = 0;
	uint16_t temp = 0;
	char cmd_buf[CMD_BUF_SIZE] = { TMP101_ADDR, TMP_REG_TEMPERATURE, (TMP101_ADDR | I2C_READ_BIT), };
	char ctrl_buf[CMD_BUF_SIZE] = { I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };

	if (probe_sensor() != 1) {
		return -ENODEV;
	}

	/* Read the requested data */
	if (last_accessed_register == TMP_REG_TEMPERATURE) {
		/* No need to switch back to temperature register */
		ret = i2c_read((cmd_buf + 2), 1, (ctrl_buf + 2), (char*)&temp, 2);
	} else {
		/* Send (write) temperature register address to TMP101 internal pointer */
		ret = i2c_read(cmd_buf, CMD_BUF_SIZE, ctrl_buf, (char*)&temp, 2);
	}

	if (ret == 2) {
		if (raw != NULL) {
			*raw = byte_swap_16(temp);
		}
		if (deci_degrees != NULL) {
			*deci_degrees = convert_to_deci_degrees(byte_swap_16(temp));
		}
		return 0;
	}
	return ret;
}


/* Sensor config
 * Performs default configuration of the temperature sensor.
 * The sensor is thus placed in shutdown mode, the thermostat is in interrupt mode,
 * and the polarity is set to active high.
 * The conversion resolution is set to the provided "resolution".
 * RETURN VALUE
 *   Upon successfull completion, returns 0. On error, returns a negative integer
 *   equivalent to errors from glibc.
 *   -EBADFD : I2C not initialized
 *   -EBUSY : Device or ressource Busy or Arbitration lost
 *   -EINVAL : Invalid argument (buf)
 *   -ENODEV : No such device
 *   -EREMOTEIO : Device did not acknowledge : Any device present ?
 *   -EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 */
static uint8_t actual_config = 0;
#define CONF_BUF_SIZE 4
int sensor_config(uint32_t resolution)
{
	int ret = 0;
	char cmd[CONF_BUF_SIZE] = { TMP101_ADDR, TMP_REG_CONFIG, };

	if (probe_sensor() != 1) {
		return -ENODEV;
	}

	/* Store the new configuration */
	actual_config = (TMP_SHUTDOWN_MODE_ON | TMP_THERMOSTAT_INTERRUPT_MODE | TMP_ALERT_POLARITY_HIGH);
	actual_config |= (resolution & (0x03 << 5));
	cmd[2] = actual_config;
	ret = i2c_write(cmd, 3, NULL);
	last_accessed_register = TMP_REG_CONFIG;
	if (ret == 3) {
		return 0; /* Config success */
	}
	return ret;
}

/* Start a conversion when the sensor is in shutdown mode. */
int sensor_start_conversion(void)
{
	int ret = 0;
	char cmd[CONF_BUF_SIZE] = { TMP101_ADDR, TMP_REG_CONFIG, };

	if (probe_sensor() != 1) {
		return -ENODEV;
	}

	cmd[2] = actual_config;
	cmd[2] |= TMP_ONE_SHOT_TRIGGER;
	ret = i2c_write(cmd, 3, NULL);
	last_accessed_register = TMP_REG_CONFIG;
	if (ret == 3) {
		return 0; /* Conversion start success */
	}
	return ret;
}


