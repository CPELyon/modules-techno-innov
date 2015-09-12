/*
 * lib/dtplug_protocol_slave.h
 *
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
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
 */

#ifndef LIB_DTPLUG_PROTOCOL_SLAVE_H
#define LIB_DTPLUG_PROTOCOL_SLAVE_H


#include <stdint.h>
#include "lib/dtplug_protocol.h"


/******************************************************************************/
/* DTPlug (or DomoTab, PC, ...) Communication */


/* Setup the UART used for communication with the host / master (the module is slave) */
void dtplug_protocol_set_dtplug_comm_uart(uint8_t uart_num);


/* Tell the receive routine that the "packet_ok" packet is no more in use and that
 *  we are ready to handle a new one */
void dtplug_protocol_release_old_packet(void);


/* Get a pointer to the new packet received.
 * Return NULL when no new packet were received since last packet was released.
 */
struct packet* dtplug_protocol_get_next_packet_ok(void);


/* When a packet has not been handled we must not count it as acknowledged
 * On the next ping request the master will then see wich packet caused the problem.
 */
void dtplug_protocol_add_error_to_list(struct header* info, uint8_t error_code);


/* This function handle sending replies when requested by the host.
 * When there is an error but the host did not request a reply, this function stores the error for
 *    future request.
 * When a reply is effectively sent, the PACKET_NEEDS_REPLY bit is removed from the sequence filed
 *   packet handling code will know if there is still a PING request to be answered.
 */
void dtplug_protocol_send_reply(struct packet* question, uint8_t error, int size, uint8_t* data);



#endif /* LIB_DTPLUG_PROTOCOL_SLAVE_H */
