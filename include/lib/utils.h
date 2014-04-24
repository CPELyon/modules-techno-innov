/****************************************************************************
 *  lib/utils.h
 *
 * Copyright 2014 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef LIB_UTILS_H
#define LIB_UTILS_H

/***************************************************************************** */
/* Library routines                                                            */
/***************************************************************************** */

#include <stdint.h>


/***************************************************************************** */
/* Bit twidling hacks.
 * http://graphics.stanford.edu/~seander/bithacks.html
 */

/* Counting consecutive trailing or leading zero bits (or finding bit indices)
 * The ARM Cortex M0 core does not have the __builtin_clz() and __builtin_ctz()
 * instructions.
 */

/* Count leading zeroes
 * The following function is an effitient way to implement __builtin_clz().
 */
uint8_t clz(uint32_t x);

/* Count traling zeroes
 * The following function is an effitient way to implement __builtin_ctz().
 */
uint8_t ctz(uint32_t x);

#endif /* LIB_UTILS_H */

