/****************************************************************************
 *   extdrv/temp.h
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

#ifndef EXTDRV_TEMP_H
#define EXTDRV_TEMP_H

#include <stdint.h>


/***************************************************************************** */
/*          Support for TMP101 temperature sensors from Texas Instrument       */
/***************************************************************************** */
/* This driver is made for the TMP101 version of the chip, though there's few
 * diferences between the TMP100 and TMP101 version.
 * This driver does not handle the SMBus Alert command.
 */


/* Faulty mesures required to trigger alert */
#define TMP_FAULT_ONE       ((0x00 & 0x03) << 3)
#define TMP_FAULT_TWO       ((0x01 & 0x03) << 3)
#define TMP_FAULT_FOUR      ((0x02 & 0x03) << 3)
#define TMP_FAULT_SIX       ((0x03 & 0x03) << 3)

/* Temp mesurement resolution bits */            /* conversion time */
#define TMP_RES_NINE_BITS   ((0x00 & 0x03) << 5)   /*  40ms */
#define TMP_RES_TEN_BITS    ((0x01 & 0x03) << 5)   /*  80ms */
#define TMP_RES_ELEVEN_BITS ((0x02 & 0x03) << 5)   /* 160ms */
#define TMP_RES_TWELVE_BITS ((0x03 & 0x03) << 5)   /* 320ms */


/* Check the sensor presence, return 1 if found
 * This is a basic check, it could be anything with the same address ...
 * addr: the sensor address on most significant bits.
 */
int tmp101_probe_sensor(uint8_t addr);


/* Convert raw temperature data (expressed as signed value of 16 times the
 * actual temperature in the twelve higher bits of a sixteen bits wide value)
 * into a decimal interger value of ten times the actual temperature.
 * The value returned is thus in tenth of degrees centigrade.
 */
int tmp101_convert_to_deci_degrees(uint16_t raw);


/* Temp Read
 * Performs a non-blocking read of the temperature from the sensor.
 * addr: the sensor address on most significant bits.
 * 'raw' and 'deci_degrees': integer addresses for conversion result, may be NULL.
 * Return value(s):
 *   Upon successfull completion, returns 0 and the temperature read is placed in the
 *   provided integer(s). On error, returns a negative integer equivalent to errors from
 *   glibc.
 *   -EBADFD : I2C not initialized
 *   -EBUSY : Device or ressource Busy or Arbitration lost
 *   -EINVAL : Invalid argument (buf)
 *   -ENODEV : No such device
 *   -EREMOTEIO : Device did not acknowledge : Any device present ?
 *   -EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 */
int tmp101_sensor_read(uint8_t addr, uint16_t* raw, int* deci_degrees);


/* Sensor config
 * Performs default configuration of the temperature sensor.
 * The sensor is thus placed in shutdown mode, the thermostat is in interrupt mode,
 * and the polarity is set to active high.
 * The conversion resolution is set to the provided "resolution".
 * addr: the sensor address on most significant bits.
 * Return value:
 *   Upon successfull completion, returns 0. On error, returns a negative integer
 *   equivalent to errors from glibc.
 *   -EBADFD : I2C not initialized
 *   -EBUSY : Device or ressource Busy or Arbitration lost
 *   -EINVAL : Invalid argument (buf)
 *   -ENODEV : No such device
 *   -EREMOTEIO : Device did not acknowledge : Any device present ?
 *   -EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 */
int tmp101_sensor_config(uint8_t addr, uint32_t resolution);

/* Start a conversion when the sensor is in shutdown mode.
 * addr : the sensor address on most significant bits.
 */
int tmp101_sensor_start_conversion(uint8_t addr);


#endif /* EXTDRV_TEMP_H */

