/****************************************************************************
 *  drivers/rtc.h
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
/*                RTC and RTC Interrupts                                    */
/***************************************************************************** */

#include <stdint.h>


/* Return the number of RTC ticks from system power on.
 * This count is from power being present, even if resets occured.
 * The calls made during the first three seconds after RTC timer start will return 0. (See
 *    UM10441 section 16.6.1)
 */
uint32_t rtc_get_count(void);


/***************************************************************************** */
/*   RTC setup   */

/* In case someone wants the RTC to count something different from seconds
 * No need to call this function if you only want to count seconds, this is the default.
 * source selection values are defined in lpc_regs_12xx.h
 * clk_div is only usefull when selected clock source is PCLK (peripheral clock). Use value
 *    betwween 1 and 255 included.
 */
void rtc_clk_src_select(uint8_t source, uint8_t clk_div);

/* Start the RTC. Once the RTC has been started using this call, it cannot be stopped. */
void rtc_on(void);

/* Disable the RTC control block. Note that once started from software the RTC cannot be stopped. */
void rtc_ctrl_off(void);

/* This will disable the RTC. This is only possible if the RTC has not been started before. */
void rtc_disable(void);

/* Change the RTC value */
void rtc_set_date(uint32_t date);


/***************************************************************************** */
/* Set the match value for the RTC
 * If periodic is 1 then the rtc match register will be updated after the interrupt
 *    has been handled by adding the offset.
 * If date is set, the match register will be set to date. (regardless of the periodic
 *    argument, wich allows to have the first interrupt at a given date, and the following
 *    ones at a period set by the offset argument.
 * If date is 0, then the match register is set to current date + offset.
 * return -1 if RTC is not started and synced.
 */
int rtc_set_match(uint32_t date, uint32_t offset, uint8_t periodic);

/* Register a callback for the RTC
 * 'when' should be in the future (according to RTC counter) and 'period' can be used to get
 *   periodic interrupts. 'period' is a number of RTC counter increments.
 * Return a positive integer if registration is OK and callback has a chance of being called.
 * Return a negative integer if the match configuration was not possible.
 */
int set_rtc_callback(void (*callback) (uint32_t), uint32_t when, uint32_t period);
void remove_rtc_callback();



