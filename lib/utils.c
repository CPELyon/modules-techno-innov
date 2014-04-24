/****************************************************************************
 *  lib/utils.c
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
uint8_t clz(uint32_t x)
{
	static const uint8_t bval[] = {0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4};
	unsigned int r = 32;
	if (x >= 0x10000) { /* Quicker than (x & 0xFFFF0000) on a 32bit arm arch */
		r -= 16;
		x >>= 16;
	}
	if (x & 0xFF00) {
		r -= 8;
		x >>= 8;
	}
	if (x & 0xF0) {
		r -= 4;
		x >>= 4;
	}
	return r - bval[x];
}

/* Count traling zeroes
 * The following function is an effitient way to implement __builtin_ctz().
 */
uint8_t ctz(uint32_t x)
{
	static const uint8_t bval[] = {4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0};
	unsigned int r = 0;
	if (x & 0x1) {
		/* special case for odd value (assumed to happen half of the time) */
		return r;
	}
	if ((x & 0xFFFF) == 0) {
		r += 16;
		x >>= 16;
	}
	if ((x & 0xFF) == 0) {
		r += 8;
		x >>= 8;
	}
	if ((x & 0xF) == 0) {
		r += 4;
		x >>= 4;
	}
	return r + bval[(x & 0x0F)];
}
