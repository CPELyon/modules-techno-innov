/****************************************************************************
 *   extdrv/eeprom.h
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

#ifndef EXTDRV_EEPROM_H
#define EXTDRV_EEPROM_H

#include <stdint.h>


/***************************************************************************** */
/*       Module identification support for DTPlug and DomoTab                  */
/***************************************************************************** */

/* Module capabilities */
#define UEXT_MOD_HAS_NONE  0
#define UEXT_MOD_HAS_UART (1 << 0)
#define UEXT_MOD_HAS_I2C  (1 << 1)
#define UEXT_MOD_HAS_SPI  (1 << 2)

struct module_desc {
	uint16_t serial_number;
	uint8_t version;
	uint8_t header_size;
	uint8_t capabilities; /* Bit mask of UEXT_MOD_HAS_* */
	uint8_t name_offset;
	uint8_t name_size;
	uint8_t image_offset;
	uint16_t image_size;
} __attribute__ ((packed));

enum i2c_eeprom_type {
	EEPROM_TYPE_NONE = 0,
	EEPROM_TYPE_SMALL,
	EEPROM_TYPE_BIG,
};



/***************************************************************************** */
/*          Read and Write for system eeprom                                   */
/***************************************************************************** */
/* EEPROM Read
 * Performs a non-blocking read on the eeprom.
 *   address : data offset in eeprom.
 * RETURN VALUE
 *   Upon successfull completion, returns the number of bytes read. On error, returns a negative
 *   integer equivalent to errors from glibc.
 *   -EFAULT : address above eeprom size
 *   -EBADFD : Device not initialized
 *   -EBUSY : Device or ressource Busy or Arbitration lost
 *   -EAGAIN : Device already in use
 *   -EINVAL : Invalid argument (buf)
 *   -EREMOTEIO : Device did not acknowledge
 *   -EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 */
int eeprom_read(uint32_t offset, void *buf, size_t count);

/* EEPROM Write
 * Performs a non-blocking write on the eeprom.
 *   address : data offset in eeprom.
 * RETURN VALUE
 *   Upon successfull completion, returns the number of bytes written. On error, returns a negative
 *   integer equivalent to errors from glibc.
 *   -EFAULT : address above eeprom size
 *   -EBADFD : Device not initialized
 *   -EBUSY : Device or ressource Busy or Arbitration lost
 *   -EAGAIN : Device already in use
 *   -EINVAL : Invalid argument (buf)
 *   -EREMOTEIO : Device did not acknowledge
 *   -EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 */
int eeprom_write(uint32_t offset, const void *buf, size_t count);




#endif /* EXTDRV_EEPROM_H */
