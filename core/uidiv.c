/****************************************************************************
 *   core/uidiv.c
 *
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

/*******************************************************************************/
/*            Integer division using ROM based division routines               */
/*******************************************************************************/

struct idiv_return {
	int quotient;
	int remainder;
};
struct uidiv_return {
	unsigned quotient;
	unsigned remainder;
};

struct lpc_rom_div_helpers {
	/* Signed integer division */
	int (*rom_sidiv)(int numerator, int denominator);
	/* Unsigned integer division */
	unsigned (*rom_uidiv)(unsigned numerator, unsigned denominator);
	/* Signed integer division with remainder */
	struct idiv_return (*rom_sidivmod)(int numerator, int denominator);
	/* Unsigned integer division with remainder */
	struct uidiv_return (*rom_uidivmod)(unsigned numerator, unsigned denominator);
};

#define LPC_122x_DIVROM_LOC (0x1FFC0000)
static struct lpc_rom_div_helpers* rom_div_helpers;

void rom_div_helpers_init(void)
{
	rom_div_helpers = *((struct lpc_rom_div_helpers**)LPC_122x_DIVROM_LOC);
}

/* Division (/) */
int __aeabi_idiv(int numerator, int denominator)
{
	return rom_div_helpers->rom_sidiv(numerator, denominator);
}
unsigned __aeabi_uidiv(unsigned numerator, unsigned denominator)
{
	return rom_div_helpers->rom_uidiv(numerator, denominator);
}

/* Modulo (%) */
void __aeabi_idivmod(int numerator, int denominator)
{
	struct idiv_return result = rom_div_helpers->rom_sidivmod(numerator, denominator);
	register uint32_t r0 asm("r0") = result.quotient;
	register uint32_t r1 asm("r1") = result.remainder;
	asm volatile("" : "+r"(r0), "+r"(r1) : "r"(r0), "r"(r1));
}
void __aeabi_uidivmod(unsigned numerator, unsigned denominator)
{
	struct uidiv_return result = rom_div_helpers->rom_uidivmod(numerator, denominator);
	register uint32_t r0 asm("r0") = result.quotient;
	register uint32_t r1 asm("r1") = result.remainder;
	asm volatile("" : "+r"(r0), "+r"(r1) : "r"(r0), "r"(r1));
}

