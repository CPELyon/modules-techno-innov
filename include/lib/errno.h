/****************************************************************************
 *   lib/errno.h
 *
 * Copyright 2016 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef LIB_ERRNO_H
#define LIB_ERRNO_H

/* Error Values, from glibc errno.h and errno-base.h */
#define EIO          5 /* Bad one: Input or Output error. */
#define E2BIG        7 /* Argument list too long or Data size beyond buffer size */
#define EAGAIN      11 /* Device already in use */
#define EFAULT      14 /* Address error */
#define EBUSY       16 /* Device or ressource Busy */
#define ENODEV      19 /* No such device */
#define EINVAL      22 /* Invalid argument */
#define EBADFD      77 /* Device not initialized */
#define EREMOTEIO  121 /* Device did not acknowledge */

/* Note on error values for I2C :
 *  EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 *  EFAULT : address above eeprom size
 *  EBUSY : Device or ressource Busy or Arbitration lost
 *  EREMOTEIO : Device did not acknowledge
 */


#endif /* LIB_ERRNO_H */
