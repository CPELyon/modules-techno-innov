/****************************************************************************
 *  drivers/serial.h
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

/***************************************************************************** */
/*                UARTs                                                        */
/***************************************************************************** */
/* UART driver for the integrated UARTs of the LPC1224.
 * Refer to LPC1224 documentation (UM10441.pdf) for more information.
 */

/* Both UARTs are available, UART numbers are in the range 0 - 1 */

#ifndef DRIVERS_SERIAL_H
#define DRIVERS_SERIAL_H


#include <stdint.h>


#define SERIAL_CAP_UART   (1 << 0)
#define SERIAL_CAP_RS485  (1 << 1)
#define SERIAL_CAP_IRDA   (1 << 2)

#define SERIAL_MODE_UART  SERIAL_CAP_UART
#define SERIAL_MODE_RS485 SERIAL_CAP_RS485
#define SERIAL_MODE_IRDA  SERIAL_CAP_IRDA


/* This buffer size is the maximum write size for a serial_printf or a uprintf call.
 * Previously this value was 64, but it is often too short, so I set it to 96, which
 *    should be OK for most cases. In need of a bigger write buffer, change this value
 *    or perform multiple write calls (better).
 */
#define SERIAL_OUT_BUFF_SIZE 96


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
/*    Serial Flush
 *
 * Wait until all characters have been sent
 * Returns -1 on error, 0 on success.
 * Possible errors: requested uart does not exists or unable to acquire uart lock.
 *
 * Warning for Real Time : This implementation will block if there's already a
 * transmission ongoing.
 */
int serial_flush(uint32_t uart_num);


/****************************************************************************** */
/*    Serial send byte - quick function with almost no tests.
 * If the uart is not sending, the byte is placed directly in the data buffer and
 * the call returns 0.
 * Else, the call returns -EBUSY and nothing is sent.
 */
int serial_send_quickbyte(uint32_t uart_num, uint8_t data);


/***************************************************************************** */
/*   Public access to UART setup   */


/* Change UART configuration (number of data, parity and stop bits).
 * config is a mask of LPC_UART_xBIT (x = 5..8), LPC_UART_xSTOP (x = 1..2)
 *   and one of LPC_UART_NO_PAR, LPC_UART_ODD_PAR or LPC_UART_EVEN_PAR.
 */
int uart_set_config(uint8_t uart_num, uint8_t config);

/* Change uart mode to RS485
 * return -ENODEV when the device does not support RS485 mode.
 */
int uart_set_mode_rs485(uint32_t uart_num, uint32_t control, uint8_t addr, uint8_t delay);

/* Change uart mode to IRDA
 * return -ENODEV when the device does not support IrDA mode.
 */
int uart_set_mode_irda(uint32_t uart_num, uint32_t control, uint16_t pulse_width);

/* Allow any change to the main clock to be forwarded to us */
void uart_clk_update(void);

/* Do we need to allow setting of other parameters ? (Other than 8n1) */
int uart_on(uint32_t uart_num, uint32_t baudrate, void (*rx_callback)(uint8_t));

void uart_off(uint32_t uart_num);



#endif /* DRIVERS_SERIAL_H */
