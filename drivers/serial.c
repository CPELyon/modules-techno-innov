/****************************************************************************
 *  drivers/serial.c
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
/* Both UARTs are available, UART numbers are in the range 0 - 1 */

#include <stdint.h>
#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"
#include "core/system.h"
#include "lib/string.h"

#define SERIAL_OUT_BUFF_SIZE 64
struct uart_device
{
	uint32_t num;
	struct lpc_uart* regs;
	uint32_t baudrate;
	uint32_t config;

	/* Output buffer */
	volatile char out_buff[SERIAL_OUT_BUFF_SIZE];
	volatile uint32_t sending; /* Actual sending position in out buffer */
	volatile uint32_t out_lock;
	volatile uint32_t out_length; /* actual position to add in out buffer */

	/* Input buffer */
};

#define NUM_UARTS 2
static struct uart_device uarts[NUM_UARTS] = {
	{
		.num = 0,
		.regs = (struct lpc_uart*)LPC_UART_0,
		.baudrate = 0,
		.config = (LPC_UART_8BIT | LPC_UART_NO_PAR | LPC_UART_1STOP),
		.out_buff = {0},
		.sending = 0,
		.out_lock = 0,
	},
	{
		.num = 1,
		.regs = (struct lpc_uart*)LPC_UART_1,
		.baudrate = 0,
		.config = (LPC_UART_8BIT | LPC_UART_NO_PAR | LPC_UART_1STOP),
		.out_buff = {0},
		.sending = 0,
		.out_lock = 0,
	},
};

/* Generic UART handler */
static void UART_Handler(struct uart_device* uart)
{
	uint32_t intr = uart->regs->func.intr_pending;

	if ((intr & LPC_UART_INT_MASK) == LPC_UART_INT_RX) {
		uint8_t data = uart->regs->func.buffer;
		/* Echo */
		uart->regs->func.buffer = data;
	}
	/* We are currently sending, send next char */
	if ((intr & LPC_UART_INT_MASK) == LPC_UART_INT_TX) {
		if (uart->out_buff && uart->sending && (uart->out_length > uart->sending)) {
			uart->regs->func.buffer = uart->out_buff[uart->sending];
			uart->sending++;
		} else {
			uart->sending = 0;
		}
	}
}


/* Handlers */
void UART_0_Handler(void)
{
	UART_Handler(&uarts[0]);
}
void UART_1_Handler(void)
{
	UART_Handler(&uarts[1]);
}


/* Start sending buffer content */
static void uart_start_sending(uint32_t uart_num)
{
	struct uart_device* uart = &uarts[uart_num];

	if (uart->out_buff == NULL)
		return;

	if (!uart->sending && (uart->out_length != 0)) {
		uart->sending++;
		uart->regs->func.buffer = uart->out_buff[0];
	}
}


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
int serial_write(uint32_t uart_num, const char *buf, uint32_t length)
{
	struct uart_device* uart = NULL;

	if (uart_num >= NUM_UARTS)
		return -1;

	uart = &uarts[uart_num];
	/* Lock acquire */
	if (sync_lock_test_and_set(&uart->out_lock, 1) == 1) {
		return -1;
	}

	/* If UART is sending wait for buffer empty */
	do {} while (uart->sending != 0);

	if (length > SERIAL_OUT_BUFF_SIZE) {
		length = SERIAL_OUT_BUFF_SIZE;
	}
	memcpy((char*)uart->out_buff, buf, length);
	uart->out_length = length;

	/* Turn output on */
	uart_start_sending(uart_num);

	/* Release the lock */
	sync_lock_release(&uart->out_lock);

	return length;
}




/***************************************************************************** */
/*   UART Setup : private part : Clocks, Pins, Power and Mode   */

/* UART Clock Setup */
/* Note : for both uarts we use full peripheral clock.
 *    With a minimum main clock of 12MHz, ths gives 12MHz/16 = 750kbauds at least
 *    for UARTs baudrates.
 * Note : IRQ are off, whether called by system update or by UART on helper
 */
static void uart_clk_on(uint32_t uart_num, uint32_t baudrate)
{
	struct lpc_sys_control* sys_ctrl = LPC_SYS_CONTROL;
	struct lpc_uart* uart = uarts[uart_num].regs; /* Get the right registers */
	uint32_t div = 0, pclk = 0;
	/* Save baudrate value */
	uarts[uart_num].baudrate = baudrate;
	/* Configure UART clock */
	pclk = get_main_clock(); /* See above note */
	sys_ctrl->uart_clk_div[uart_num] = 0x01;
	div = (pclk / (baudrate * 16));
	uart->line_ctrl |= LPC_UART_ENABLE_DLAB;
	uart->ctrl.divisor_latch_lsb = (div & 0xff);
	uart->ctrl.divisor_latch_msb = ((div >> 8) & 0xFF);
	uart->line_ctrl &= ~LPC_UART_ENABLE_DLAB;
}
static void uart_clk_off(uint32_t uart_num)
{
	struct lpc_sys_control* sys_ctrl = LPC_SYS_CONTROL;
	/* Erase saved baudrate */
	uarts[uart_num].baudrate = 0;
	sys_ctrl->uart_clk_div[uart_num] = 0;
}

static uint32_t uart_mode_setup(uint32_t uart_num)
{
	struct lpc_uart* uart = uarts[uart_num].regs; /* Get the right registers */
	uint32_t status = 0;
	/* Set up UART mode */
	uart->line_ctrl = uarts[uart_num].config;;
	/* Clear all fifo, reset and enable them */
	/* Note : fifo trigger level is one bit */
	uart->ctrl.fifo_ctrl = (LPC_UART_FIFO_EN | LPC_UART_TX_CLR | LPC_UART_RX_CLR);
	/* Clear the Line Status Register, return it to prevent compiler from removing the read */
	status = uart->line_status;
	/* Enable UART Interrupt */
	uart->func.intr_enable = (LPC_UART_RX_INT_EN | LPC_UART_TX_INT_EN);

	return status;
}

/* Pin settings */
/* Note : We MUST set LPC_IO_DIGITAL for Rx even if the bit is marked as "Reserved" in
 *        the datasheet !
 */
static void uart_set_pin_func(uint32_t uart_num)
{
	struct lpc_io_control* ioctrl = LPC_IO_CONTROL;

	/* Make sure IO_Config is clocked */
	io_config_clk_on();
	/* Configure UART pins (Only Rx and Tx) */
	switch (uart_num) {
		case 0:
			ioctrl->pio0_1 = LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL;
			ioctrl->pio0_2 = LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL;
			break;
		case 1:
			ioctrl->pio0_8 = LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL;
			ioctrl->pio0_9 = LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL;
			break;
	}
	/* Config done, power off IO_CONFIG block */
	io_config_clk_off();
}
struct uart_def
{
	uint32_t irq;
	uint32_t power_offset;
};
static struct uart_def uart_defs[NUM_UARTS] = {
	{ UART0_IRQ, LPC_SYS_ABH_CLK_CTRL_UART0 },
	{ UART1_IRQ, LPC_SYS_ABH_CLK_CTRL_UART1 },
};

/***************************************************************************** */
/*   Public access to UART setup   */

void set_uarts_pins(void)
{
	int i = 0;
	for (i = 0; i < NUM_UARTS; i++) {
		uart_set_pin_func(i);
	}
}

/* Allow any change to the main clock to be forwarded to us */
void uart_clk_update(void)
{
	int i = 0;
	for (i = 0; i < NUM_UARTS; i++) {
		if (uarts[i].baudrate != 0) {
			uart_clk_on(i, uarts[i].baudrate);
		}
	}
}

/* Do we need to allow setting of other parameters ? (Other than 8n1) */
/* Do we need to allow specifying an interrupt handler ? */
int uart_on(uint32_t uart_num, uint32_t baudrate)
{
	struct uart_def* uart = NULL;
	uint32_t status = 0;

	if (uart_num >= NUM_UARTS)
		return -EINVAL;
	uart = &uart_defs[uart_num];

	NVIC_DisableIRQ( uart->irq );
	/* Setup pins, must be done before clock setup and with uart powered off. */
	uart_clk_off(uart_num);
	subsystem_power(uart->power_offset, 0);
	uart_set_pin_func(uart_num);
	/* Turn On power */
	subsystem_power(uart->power_offset, 1);
	/* Setup clock acording to baudrate */
	uart_clk_on(uart_num, baudrate);
	/* Setup mode, fifo, ... */
	status = uart_mode_setup(uart_num);
	/* Enable interrupts back on */
	NVIC_EnableIRQ( uart->irq );

	return status;
}

void uart_off(uint32_t uart_num)
{
	struct uart_def* uart = NULL;
	if (uart_num >= NUM_UARTS)
		return;
	uart = &uart_defs[uart_num];

	NVIC_DisableIRQ( uart->irq );
	uart_clk_off(uart_num);
	/* Turn Off power */
	subsystem_power(uart->power_offset, 0);
}



