/****************************************************************************
 *  core/pio.h
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
 ****************************************************************************/

#ifndef CORE_PIO_H
#define CORE_PIO_H

/* The "PIO" module gives access to the configuration of all the pins of the
 * micro-controller, whatever the function of the pin.
 * It has nothing related to GPIO function of the pins.
 */

#include <stdint.h>

struct pio {
    uint8_t port;  /* 0xFF indicates the end of a pin array */
    uint8_t pin;
    uint8_t alt_setting;
};

#define ARRAY_LAST_PIN   {0xFF, 0xFF, 0xFF}
#define PIO_LAST   ARRAY_LAST_PIN

struct pio_config {
	struct pio pio;
	uint32_t mode;
};
#define ARRAY_LAST_PIO  { PIO_LAST, 0xFF }


#define PORT0_NB_PINS 32
#define PORT1_NB_PINS 7
#define PORT2_NB_PINS 16

/* Simple copy function. */
void pio_copy(struct pio* dst, const struct pio* src);

/* Configure the pin in the requested function and mode. */
void config_pio(const struct pio* pp, uint32_t mode);

/* Configure a set (array) of pins in a single loop */
void set_pins(const struct pio_config* pins);

/****************************************************************************/
/*  GPIO Pins  */
#define LPC_GPIO_0_0  {0,  0, 0}
#define LPC_GPIO_0_1  {0,  1, 0}
#define LPC_GPIO_0_2  {0,  2, 0}
#define LPC_GPIO_0_3  {0,  3, 0}
#define LPC_GPIO_0_4  {0,  4, 0}
#define LPC_GPIO_0_5  {0,  5, 0}
#define LPC_GPIO_0_6  {0,  6, 0}
#define LPC_GPIO_0_7  {0,  7, 0}
#define LPC_GPIO_0_8  {0,  8, 0}
#define LPC_GPIO_0_9  {0,  9, 0}
#define LPC_GPIO_0_10 {0, 10, 0}
#define LPC_GPIO_0_11 {0, 11, 0}
#define LPC_GPIO_0_12 {0, 12, 0}
#define LPC_GPIO_0_13 {0, 13, 1}
#define LPC_GPIO_0_14 {0, 14, 0}
#define LPC_GPIO_0_15 {0, 15, 0}
#define LPC_GPIO_0_16 {0, 16, 0}
#define LPC_GPIO_0_17 {0, 17, 0}
#define LPC_GPIO_0_18 {0, 18, 0}
#define LPC_GPIO_0_19 {0, 19, 0}
#define LPC_GPIO_0_20 {0, 20, 0}
#define LPC_GPIO_0_21 {0, 21, 0}
#define LPC_GPIO_0_22 {0, 22, 0}
#define LPC_GPIO_0_23 {0, 23, 0}
#define LPC_GPIO_0_24 {0, 24, 0}
#define LPC_GPIO_0_25 {0, 25, 6}
#define LPC_GPIO_0_26 {0, 26, 6}
#define LPC_GPIO_0_27 {0, 27, 0}
#define LPC_GPIO_0_28 {0, 28, 0}
#define LPC_GPIO_0_29 {0, 29, 0}
#define LPC_GPIO_0_30 {0, 30, 1}
#define LPC_GPIO_0_31 {0, 31, 1}

#define LPC_GPIO_1_0  {1,  0, 1}
#define LPC_GPIO_1_1  {1,  1, 1}
#define LPC_GPIO_1_2  {1,  2, 0}
#define LPC_GPIO_1_3  {1,  3, 0}
#define LPC_GPIO_1_4  {1,  4, 0}
#define LPC_GPIO_1_5  {1,  5, 0}
#define LPC_GPIO_1_6  {1,  6, 0}

#define LPC_GPIO_2_0  {2,  0, 0}
#define LPC_GPIO_2_1  {2,  1, 0}
#define LPC_GPIO_2_2  {2,  2, 0}
#define LPC_GPIO_2_3  {2,  3, 0}
#define LPC_GPIO_2_4  {2,  4, 0}
#define LPC_GPIO_2_5  {2,  5, 0}
#define LPC_GPIO_2_6  {2,  6, 0}
#define LPC_GPIO_2_7  {2,  7, 0}
#define LPC_GPIO_2_8  {2,  8, 1}
#define LPC_GPIO_2_9  {2,  9, 1}
#define LPC_GPIO_2_10 {2, 10, 1}
#define LPC_GPIO_2_11 {2, 11, 0}
#define LPC_GPIO_2_12 {2, 12, 0}
#define LPC_GPIO_2_13 {2, 13, 0}
#define LPC_GPIO_2_14 {2, 14, 0}
#define LPC_GPIO_2_15 {2, 15, 0}


/****************************************************************************/
/*  CLKOUT and Reset Pin  */
#define LPC_CLKOUT_PIO_0_12 {0, 12, 2}
#define LPC_RESET_PIO_0_13  {0, 13, 0}


/****************************************************************************/
/*  SWD (Debug) pins */
#define LPC_SWD_SWCLK_PIO_0_18 {0, 18, 1}
#define LPC_SWD_SWCLK_PIO_0_26 {0, 26, 0}
#define LPC_SWD_SWDIO_PIO_0_25 {0, 25, 0}
#define LPC_SWD_SWDIO_PIO_1_2  {1,  2, 1}


/****************************************************************************/
/*  UART0 Rx/Tx Pins  */
#define LPC_UART0_RX_PIO_0_1  {0,  1, 2}
#define LPC_UART0_RX_PIO_2_1  {2,  1, 4}
#define LPC_UART0_TX_PIO_0_2  {0,  2, 2}
#define LPC_UART0_TX_PIO_2_2  {2,  2, 4}
/* UART0 - Other UART function pins */
#define LPC_UART0_CTS_PIO_0_7 {0,  7, 2}
#define LPC_UART0_CTS_PIO_2_4 {2,  4, 4}
#define LPC_UART0_DCD_PIO_0_5 {0,  5, 2}
#define LPC_UART0_DCD_PIO_2_6 {2,  6, 4}
#define LPC_UART0_DSR_PIO_0_4 {0,  4, 2}
#define LPC_UART0_DSR_PIO_2_7 {2,  7, 4}
#define LPC_UART0_DTR_PIO_0_3 {0,  3, 2}
#define LPC_UART0_DTR_PIO_2_3 {2,  3, 4}
#define LPC_UART0_RI_PIO_0_6  {0,  6, 2}
#define LPC_UART0_RI_PIO_2_5  {2,  5, 4}
#define LPC_UART0_RTS_PIO_0_0 {0,  0, 2}
#define LPC_UART0_RTS_PIO_2_0 {2,  0, 5}  /* FIXME: Doc does not list func 1 ... maybe its func 4 */

/*  UART1 Rx/Tx Pins  */
#define LPC_UART1_RX_PIO_0_8  {0,  8, 2}
#define LPC_UART1_RX_PIO_2_11 {2, 11, 5}
#define LPC_UART1_RX_PIO_2_12 {2, 12, 3}
#define LPC_UART1_TX_PIO_0_9  {0,  9, 2}
#define LPC_UART1_TX_PIO_2_10 {2, 10, 5}
#define LPC_UART1_TX_PIO_2_13 {2, 13, 3}

/****************************************************************************/
/*  I2C Pins  */
#define LPC_I2C0_SCL_PIO_0_10 {0, 10, 2}
#define LPC_I2C0_SDA_PIO_0_11 {0, 11, 2}

/****************************************************************************/
/*  SPI Pins  */
/* SSP 0 */
#define LPC_SSP0_SCLK_PIO_0_14 {0, 14, 2}
#define LPC_SSP0_MOSI_PIO_0_17 {0, 17, 2}
#define LPC_SSP0_MISO_PIO_0_16 {0, 16, 2}
#define LPC_SSP0_SSEL_PIO_0_15 {0, 15, 2}


/****************************************************************************/
/*  ADC Pins  */
#define LPC_ADC_AD0_PIO_0_30  {0, 30, 3}
#define LPC_ADC_AD1_PIO_0_31  {0, 31, 3}
#define LPC_ADC_AD2_PIO_1_0   {1,  0, 2}
#define LPC_ADC_AD3_PIO_1_1   {1,  1, 2}
#define LPC_ADC_AD4_PIO_1_2   {1,  2, 2}
#define LPC_ADC_AD5_PIO_1_3   {1,  3, 1}
#define LPC_ADC_AD6_PIO_1_4   {1,  4, 1}
#define LPC_ADC_AD7_PIO_1_5   {1,  5, 1}


/****************************************************************************/
/*  Timer Pins  */

/* 16 bits timer 0 match pins */
#define LPC_TIMER_16B0_M0_PIO_0_11 {0, 11, 4}
#define LPC_TIMER_16B0_M0_PIO_0_28 {0, 28, 4}
#define LPC_TIMER_16B0_M0_PIO_2_0  {2,  0, 4}
#define LPC_TIMER_16B0_M1_PIO_0_12 {0, 12, 4}
#define LPC_TIMER_16B0_M1_PIO_0_29 {0, 29, 4}
#define LPC_TIMER_16B0_M1_PIO_2_1  {2,  1, 4}
/* 16 bits timer 0 capture pins */
#define LPC_TIMER_16B0_C0_PIO_0_11 {0, 11, 3}
#define LPC_TIMER_16B0_C0_PIO_0_28 {0, 28, 3}
#define LPC_TIMER_16B0_C0_PIO_2_0  {2,  0, 3}
#define LPC_TIMER_16B0_C1_PIO_0_12 {0, 12, 3}
#define LPC_TIMER_16B0_C1_PIO_0_29 {0, 29, 3}
#define LPC_TIMER_16B0_C1_PIO_2_1  {2,  1, 3}

/* 16 bits timer 1 match pins */
#define LPC_TIMER_16B1_M0_PIO_0_15 {0, 15, 4}
#define LPC_TIMER_16B1_M0_PIO_1_5  {1,  5, 3}
#define LPC_TIMER_16B1_M0_PIO_2_2  {2,  2, 3}
#define LPC_TIMER_16B1_M1_PIO_0_16 {0, 16, 4}
#define LPC_TIMER_16B1_M1_PIO_1_6  {1,  6, 2}
#define LPC_TIMER_16B1_M1_PIO_2_3  {2,  3, 3}
/* 16 bits timer 1 capture pins */
#define LPC_TIMER_16B1_C0_PIO_0_15 {0, 15, 3}
#define LPC_TIMER_16B1_C0_PIO_1_5  {1,  5, 2}
#define LPC_TIMER_16B1_C0_PIO_2_2  {2,  2, 2}
#define LPC_TIMER_16B1_C1_PIO_0_16 {0, 16, 3}
#define LPC_TIMER_16B1_C1_PIO_1_6  {1,  6, 1}
#define LPC_TIMER_16B1_C1_PIO_2_3  {2,  3, 2}

/* 32 bits timer 0 match pins */
#define LPC_TIMER_32B0_M0_PIO_0_1  {0,  1, 4}
#define LPC_TIMER_32B0_M0_PIO_0_18 {0, 18, 4}
#define LPC_TIMER_32B0_M0_PIO_2_4  {2,  4, 3}
#define LPC_TIMER_32B0_M1_PIO_0_2  {0,  2, 4}
#define LPC_TIMER_32B0_M1_PIO_0_19 {0, 19, 4}
#define LPC_TIMER_32B0_M1_PIO_2_5  {2,  5, 3}
#define LPC_TIMER_32B0_M2_PIO_0_3  {0,  3, 4}
#define LPC_TIMER_32B0_M2_PIO_0_20 {0, 20, 4}
#define LPC_TIMER_32B0_M2_PIO_2_6  {2,  6, 3}
#define LPC_TIMER_32B0_M3_PIO_0_4  {0,  4, 4}
#define LPC_TIMER_32B0_M3_PIO_0_21 {0, 21, 4}
#define LPC_TIMER_32B0_M3_PIO_2_7  {2,  7, 3}
/* 32 bits timer 0 capture pins */
#define LPC_TIMER_32B0_C0_PIO_0_1  {0,  1, 3}
#define LPC_TIMER_32B0_C0_PIO_0_18 {0, 18, 3}
#define LPC_TIMER_32B0_C0_PIO_2_4  {2,  4, 2}
#define LPC_TIMER_32B0_C1_PIO_0_2  {0,  2, 3}
#define LPC_TIMER_32B0_C1_PIO_0_19 {0, 19, 3}
#define LPC_TIMER_32B0_C1_PIO_2_5  {2,  5, 2}
#define LPC_TIMER_32B0_C2_PIO_0_3  {0,  3, 3}
#define LPC_TIMER_32B0_C2_PIO_0_20 {0, 20, 3}
#define LPC_TIMER_32B0_C2_PIO_2_6  {2,  6, 2}
#define LPC_TIMER_32B0_C3_PIO_0_4  {0,  4, 3}
#define LPC_TIMER_32B0_C3_PIO_0_21 {0, 21, 3}
#define LPC_TIMER_32B0_C3_PIO_2_7  {2,  7, 2}

/* 32 bits timer 1 match pins */
#define LPC_TIMER_32B1_M0_PIO_0_6  {0,  6, 4}
#define LPC_TIMER_32B1_M0_PIO_0_23 {0, 23, 4}
#define LPC_TIMER_32B1_M0_PIO_2_8  {2,  8, 3}
#define LPC_TIMER_32B1_M1_PIO_0_7  {0,  7, 4}
#define LPC_TIMER_32B1_M1_PIO_0_24 {0, 24, 4}
#define LPC_TIMER_32B1_M1_PIO_2_9  {2,  9, 3}
#define LPC_TIMER_32B1_M2_PIO_0_8  {0,  8, 4}
#define LPC_TIMER_32B1_M2_PIO_0_25 {0, 25, 4}
#define LPC_TIMER_32B1_M2_PIO_2_10 {2, 10, 3}
#define LPC_TIMER_32B1_M3_PIO_0_9  {0,  9, 4}
#define LPC_TIMER_32B1_M3_PIO_0_26 {0, 26, 4}
#define LPC_TIMER_32B1_M3_PIO_2_11 {2, 11, 3}
/* 32 bits timer 1 capture pins */
#define LPC_TIMER_32B1_C0_PIO_0_6  {0,  6, 3}
#define LPC_TIMER_32B1_C0_PIO_0_23 {0, 23, 3}
#define LPC_TIMER_32B1_C0_PIO_2_8  {2,  8, 2}
#define LPC_TIMER_32B1_C1_PIO_0_7  {0,  7, 3}
#define LPC_TIMER_32B1_C1_PIO_0_24 {0, 24, 3}
#define LPC_TIMER_32B1_C1_PIO_2_9  {2,  9, 2}
#define LPC_TIMER_32B1_C2_PIO_0_8  {0,  8, 3}
#define LPC_TIMER_32B1_C2_PIO_0_25 {0, 25, 3}
#define LPC_TIMER_32B1_C2_PIO_2_10 {2, 10, 2}
#define LPC_TIMER_32B1_C3_PIO_0_9  {0,  9, 3}
#define LPC_TIMER_32B1_C3_PIO_0_26 {0, 26, 3}
#define LPC_TIMER_32B1_C3_PIO_2_11 {2, 11, 2}



#endif /* CORE_PIO_H */
