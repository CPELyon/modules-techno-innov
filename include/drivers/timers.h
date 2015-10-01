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

#ifndef DRIVERS_TIMERS_H
#define DRIVERS_TIMERS_H

#include <stdint.h>

/***************************************************************************** */
/*                Timers                                                       */
/***************************************************************************** */
/* Timers driver for the integrated timers of the LPC1224.
 * Four timers are available, Two 16 bits and two 32 bits
 * All timers have 4 channels though 32 bits timers have all 4 channels available
 *   on capture /match pins while 16bits ones have only two (channels 0 and 1).
 * Refer to LPC1224 documentation (UM10441.pdf) for more information.
 */
#define NUM_TIMERS 4
#define NUM_CHANS 4

/* Timer numbers to be used for functions from this driver. */
#define LPC_TIMER_16B0  0
#define LPC_TIMER_16B1  1
#define LPC_TIMER_32B0  2
#define LPC_TIMER_32B1  3


/* Timer modes for the "mode" timer config structure field */
enum lpc_timer_mode {
	LPC_TIMER_MODE_TIMER = 0,
	LPC_TIMER_MODE_COUNTER,
	LPC_TIMER_MODE_CAPTURE,
	LPC_TIMER_MODE_MATCH,
	LPC_TIMER_MODE_PWM,  /* Pulse Width Modulation */
	LPC_TIMER_MODE_PWD,  /* Pulse Width Demodulation */
};

/* Structure used to pass parameters to configure a timer */
/* Notes:
 * In counter or PWM mode, the config is done using config[0] for enabled channels and config[1] holds
 *   the channel number used to control PWM cycle.
 *   Note that the manual recommends Channel 3 be used for PWM cycle length.
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
void timer_continue(uint32_t timer_num);

/* Pause the timer counter, does not reset */
void timer_pause(uint32_t timer_num);

/* Stop and reset the timer counter */
void timer_stop(uint32_t timer_num);

/* Resets the timer and lets it count again imediately */
void timer_restart(uint32_t timer_num);

uint32_t timer_get_capture_val(uint32_t timer_num, uint32_t channel);

uint32_t timer_get_counter_val(uint32_t timer_num);

/* Change the match value of a single timer channel */
void timer_set_match(uint32_t timer_num, uint32_t channel, uint32_t val);

/***************************************************************************** */
/*   Timer Setup */
/* Returns 0 on success
 * Takes a timer number and a timer config structure as arguments.
 * Refer to timer config structure for details.
 * Note: use of channel 3 for PWM cycle length is enforced.
 */
int timer_setup(uint32_t timer_num, const struct timer_config* conf);



/* Power up a timer.
 * Note that clkrate should be a divider of the main clock frequency chosed
 *   for your application as it will be used to divide the main clock to get
 *   the prescaler value.
 * Set clkrate to 0 to disable the prescaler.
 * callback is used for all the possible timer interrupts (activated using the
 *   config field in timer_config struct upon timer setup)
 *   The interrupt flags are passed to the interrupt routine as argument.
 */
void timer_on(uint32_t timer_num, uint32_t clkrate, void (*callback)(uint32_t));

/* Removes the main clock from the selected timer block */
void timer_off(uint32_t timer_num);

#endif /* DRIVERS_TIMERS_H */

