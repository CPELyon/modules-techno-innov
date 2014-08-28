/****************************************************************************
 *   core/iap.h
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

#ifndef CORE_IAP_H
#define CORE_IAP_H

#include <stdint.h>


/*******************************************************************************/
/*            In Application Programming ROM based routines                    */
/*******************************************************************************/

/* Provide access to IAP ROM based routines.
 * This is the only access to User information block, and allows in-application re-programming
 *   of the micro-controller (for bootloaders, drivers, loadable RTOS tasks, ....)
 */


/* Erase some pages from the user information block.
 * Page numbers may be 0, 1 or 2 for the LPC1224.
 * Provide the same page number as start and end to erase a single page.
 * Of course, end page number MUST be higher than (or equal to) start page number.
 * There is no way to erase only parts of a page.
 * Interrputs are disabled during this operation.
 */
int iap_erase_info_page(uint32_t start_page, uint32_t end_page);


/* Prepare sectors from the programm flash memory for erasing or writting
 * Sectors numbers start at 0. A flash sector size is 4kB (0x1000 bytes)
 * Provide the same sector number as start and end to prepare a single sector.
 * Of course, end sector number MUST be higher than (or equal to) start sector number.
 */
int iap_prepare_flash(uint32_t start_sector, uint32_t end_sector);

/* Erase full flash sectors from the programm memory
 * Sectors numbers start at 0. A flash sector size is 4kB (0x1000 bytes)
 * Provide the same sector number as start and end to erase a single sector.
 * Of course, end sector number MUST be higher than (or equal to) start sector number.
 * Use iap_erase_flash_pages() to erase a single page within a sector.
 * A sector must be prepared for writing before the erase operation.
 * Interrputs are disabled during this operation.
 */
int iap_erase_flash_sectors(uint32_t start_sector, uint32_t end_sector);

/* Erase pages  from the programm memory
 * Page numbers start at 0. A flash page size is 512 bytes.
 * Provide the same page number as start and end to erase a single page.
 * Of course, end page number MUST be higher than (or equal to) start page number.
 * There is no way to erase only parts of a page.
 * The sector in which the page reside must be prepared for writing before the page erase
 *    operation.
 * Interrputs are disabled during this operation.
 */
int iap_erase_flash_pages(uint32_t start_page, uint32_t end_page);


/* Copy some data from RAM to Flash.
 * When writting to the programm flash memory, the sectors must be prepared for writing
 *   before the copy operation.
 * Both dest and src parameters must be aligned to 4 bytes boundary and size must be a
 *   multiple of 4.
 * dest is the destination address and must be the address of some flash memory.
 * ser is the source address and must be the address of some RAM memory.
 * The sector or page in dest must have been erased before writing and no writing operation
 *   performed between (dest) and (dest+size) addresses. (Writting in other parts of the
 *   page or sector is OK)
 * Interrputs are disabled during this operation.
 */
int iap_copy_ram_to_flash(uint32_t dest, uint32_t src, uint32_t size);


/* Return the part ID */
uint32_t iap_read_part_id(void);


#endif /* CORE_IAP_H */

