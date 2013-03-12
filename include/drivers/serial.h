/****************************************************************************
 *  serial.h
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef DRIVERS_SERIAL_H
#define DRIVERS_SERIAL_H


#include <stdint.h>


/***************************************************************************** */
/*    Serial Write
 *
 * Try to send at most "length" characters from "buf" on the requested uart.
 * Returns -1 on error, or number of characters copied into output buffer, witch
 * may be less than requested "length"
 * Possible errors: requested uart does not exists or unable to acquire uart lock.
 *
 * Warning for Real Time : This implementation will block if there's already a
 * transmission ongoing.
 */
int serial_write(uint32_t uart_num, const char *buf, uint32_t length);





/***************************************************************************** */
/*   Public access to UART setup   */

void set_uarts_pins(void);

/* Allow any change to the main clock to be forwarded to us */
void uart_clk_update(void);

/* Do we need to allow setting of other parameters ? (Other than 8n1) */
/* Do we need to allow specifying an interrupt handler ? */
void uart_on(uint32_t uart_num, uint32_t baudrate);

void uart_off(uint32_t uart_num);



#endif /* DRIVERS_SERIAL_H */
