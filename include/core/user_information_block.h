/****************************************************************************
 *   core/user_information_block.h
 *
 *
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef USER_INFORMATION_BLOCK_H
#define USER_INFORMATION_BLOCK_H

#include <stdint.h>

struct user_info {
	char name[48];
	uint32_t version;
	uint32_t serial_number;
	char user_info[8];
} __attribute__ ((__packed__));


/* Get a pointer to the begining address of the user information block in flash. */
void* get_user_info(void);


#endif /* USER_INFORMATION_BLOCK_H */

