/****************************************************************************
 *  drivers/timers.h
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


#include <stdint.h>
#include "core/lpc_regs_12xx.h"

/***************************************************************************** */
/*                Timers                                                       */
/***************************************************************************** */
/* Four timers are available, Two 16 bits and two 32 bits
 * All timers have 4 channels though 32 bits timers have all 4 channels available
 *   on capture /match pins while 16bits ones have only two (channels 0 and 1).
 */
#define NUM_TIMERS 4
#define NUM_CHANS 4

/* Timer numbers to be used for functions from this driver. */
#define LPC_TIMER_16B0  0
#define LPC_TIMER_16B1  1
#define LPC_TIMER_32B0  2
#define LPC_TIMER_32B1  3


/* Timer modes for the "mode" timer config structure field */
#define LPC_TIMER_MODE_COUNTER  1
#define LPC_TIMER_MODE_CAPTURE  2
#define LPC_TIMER_MODE_MATCH    3
#define LPC_TIMER_MODE_PWM      4  /* Pulse Width Modulation */
#define LPC_TIMER_MODE_PWD      5  /* Pulse Width Demodulation */

/* Structure used to pass parameters to configure a timer */
/* Notes:
 * In counter or PWM mode, the config is done using config[0]
 * The field "reset_on_capture" must be set to LPC_COUNTER_CLEAR_ON_EVENT_EN ored with one
 *   of the LPC_COUNTER_CLEAR_ON_CHAN*_* to activate the clear timer on event functionality
 */
struct timer_config
{
	uint32_t mode; /* Counter, Timer Capture, Timer Match or PWM */
	uint8_t config[NUM_CHANS]; /* Configure the internal behavior when a capture or a match occurs */
	uint8_t ext_match_config[NUM_CHANS]; /* Configure the external behavior when a match occurs */
	uint32_t match[NUM_CHANS]; /* The match values if the timer is used in match mode */
	uint32_t reset_on_capture;
};



/* Start the timer :
 * Remove the reset flag if present and set timer enable flag.
 * Timer must be turned on and configured (no checks done here).
 */
void timer_start(uint32_t timer_num);

/* Stops the timer counter
 * FIXME: Does not issue reset ... need to check whether it is reseted or not.
 */
void timer_stop(uint32_t timer_num);

/* Resets the timer and lets it count again imediately */
void timer_restart(uint32_t timer_num);

uint32_t timer_get_capture_val(uint32_t timer_num, uint32_t channel);


/* Change the match value of a single timer channel */
void timer_set_match(uint32_t timer_num, uint32_t channel, uint32_t val);

/***************************************************************************** */
/*   Timer Setup */
/* Returns 0 on success
 * Takes a timer number and a timer config structure as arguments.
 * Refer to timer config structure for details.
 * Note: use of channel 3 for PWM cycle length is enforced.
 */
int timer_setup(uint32_t timer_num, struct timer_config* conf);


/* Some alternate pins for timers are on port 2, not handled by our code as they are
 *   unavailable on the 48 pins LQFP package used in the GPIO Demo module.
 * Some alternate pins for timers are on port 1, not handled by our code to get simpler code ...
 * Note that the alternate function numbers for most capture and match functions on
 *   port 2 are 2 and 3 instead of 3 and 4 !
 */
/* 16 bits timer 0 match / capture pins */
#define LPC_TIMER_16B0_CHANNEL_0_PIN_28  28
#define LPC_TIMER_16B0_CHANNEL_1_PIN_29  29
#define LPC_TIMER_16B0_CHANNEL_0_PIN_11  11
#define LPC_TIMER_16B0_CHANNEL_1_PIN_12  12
/* Alternate on PIO2_0 and PIO2_1 */

/* 16 bits timer 1 match / capture pins */
#define LPC_TIMER_16B1_CHANNEL_0_PIN_15  15
#define LPC_TIMER_16B1_CHANNEL_1_PIN_16  16
/* Alternate on PIO1_5 and PIO1_6 */
/* Alternate on PIO2_2 and PIO2_3 */

/* 32 bits timer 0 match / capture pins */
#define LPC_TIMER_32B0_CHANNEL_0_PIN_1   1
#define LPC_TIMER_32B0_CHANNEL_1_PIN_2   2
#define LPC_TIMER_32B0_CHANNEL_2_PIN_3   3
#define LPC_TIMER_32B0_CHANNEL_3_PIN_4   4
#define LPC_TIMER_32B0_CHANNEL_0_PIN_18  18
#define LPC_TIMER_32B0_CHANNEL_1_PIN_19  19
#define LPC_TIMER_32B0_CHANNEL_2_PIN_20  20
#define LPC_TIMER_32B0_CHANNEL_3_PIN_21  21
/* Alternate on PIO2_4 to PIO2_7 */

/* 32 bits timer 1 match / capture pins */
#define LPC_TIMER_32B1_CHANNEL_0_PIN_6   6
#define LPC_TIMER_32B1_CHANNEL_1_PIN_7   7
#define LPC_TIMER_32B1_CHANNEL_2_PIN_8   8
#define LPC_TIMER_32B1_CHANNEL_3_PIN_9   9
#define LPC_TIMER_32B1_CHANNEL_0_PIN_23  23
#define LPC_TIMER_32B1_CHANNEL_1_PIN_24  24
#define LPC_TIMER_32B1_CHANNEL_2_PIN_25  25
#define LPC_TIMER_32B1_CHANNEL_3_PIN_26  26
/* Alternate on PIO2_8 to PIO2_11 */

/* Capture or match alternate function select for port 0 pins */
#define LPC_TIMER_PIN_FUNC_CAPTURE  LPC_IO_FUNC_ALT(3)
#define LPC_TIMER_PIN_FUNC_MATCH    LPC_IO_FUNC_ALT(4)


/* Setup one pin to use as match or capture pin
 * This configure function can be used only for pins on port 0.
 * Use LPC_TIMER_*B*_CHANNEL_*_PIN_* definitions for the pin number and either
 *   LPC_TIMER_PIN_FUNC_CAPTURE or LPC_TIMER_PIN_FUNC_MATCH for the function.
 * Note: These pins are not setup using the usual system set default pins functions
 *   as most functions may be affected to many different pins, and the use of the
 *   timers does no require the use of pins.
 */
void timer_pins_setup(uint8_t port, uint8_t pin_num, uint32_t func);


/* Power up a timer.
 * Note that clkrate should be a divider of the main clock frequency chosed
 *   for your application as it will be used to divide the main clock to get
 *   the prescaler value.
 * Set clkrate to 0 to disable the prescaler.
 */
void timer_on(uint32_t timer_num, uint32_t clkrate);

/* Removes the main clock from the selected timer block */
void timer_off(uint32_t timer_num);
