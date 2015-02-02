/****************************************************************************
 *   core/system.h
 *
 * All low-level functions for clocks configuration and switch, system
 *  power-up, reset, and power-down.
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef CORE_SYSTEM_H
#define CORE_SYSTEM_H

#include <stdint.h>

#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"


/* Error Values, from glibc errno.h and errno-base.h */
#define EIO          5 /* Bad one: Input or Output error. */
#define E2BIG        7 /* Argument list too long or Data size beyond buffer size */
#define EAGAIN      11 /* Device already in use */
#define EFAULT      14 /* Address error */
#define EBUSY       16 /* Device or ressource Busy */
#define ENODEV      19 /* No such device */
#define EINVAL      22 /* Invalid argument */
#define EBADFD      77 /* Device not initialized */
#define EREMOTEIO  121 /* Device did not acknowledge */

/* Note on error values for I2C :
 *  EIO : Bad one: Illegal start or stop, or illegal state in i2c state machine
 *  EFAULT : address above eeprom size
 *  EBUSY : Device or ressource Busy or Arbitration lost
 *  EREMOTEIO : Device did not acknowledge
 */

/***************************************************************************** */
/*                       Power up defaults                                     */
/***************************************************************************** */
/* Change reset power state to our default, removing power from unused
 * interfaces */
void system_set_default_power_state(void);

/* Configure all default pin functions for dtplug, even if modules or functions
 * are not used */
void system_set_default_pins(void);

/* Stop the watchdog */
void stop_watchdog(void);

/***************************************************************************** */
/*                       Power                                                 */
/***************************************************************************** */
void enter_deep_sleep(void);

/* Power on or off a subsystem */
void subsystem_power(uint32_t power_bit, uint32_t on_off);

/* Configure the brown-out detection.
 * Note: Brown-Out detection must be powered to operate the ADC (See Section 19.2
 *    of UM10441 revision 2.1 or newer for more information)
 */
void system_brown_out_detection_config(uint32_t level);

/***************************************************************************** */
/*                      System Clock                                           */
/***************************************************************************** */
/* A clock frequency is defined as the integer value in MHz divided by 12, shifted
 * by 3 and or'ed with the value to be programmed in the flash config register for
 * the flash access time at the given frequency shifted by one and the flash
 * override bit in the LSB
 */
/* PLL may fail to lock for frenquencies above 60MHz */
#define FREQ_SEL_60MHz   ((5 << 3) | (0x01 << 1) | 0)
#define FREQ_SEL_48MHz   ((4 << 3) | (0x00 << 1) | 0)
#define FREQ_SEL_36MHz   ((3 << 3) | (0x00 << 1) | 0)
#define FREQ_SEL_24MHz   ((2 << 3) | (0x00 << 1) | 1)
#define FREQ_SEL_12MHz   ((1 << 3) | (0x00 << 1) | 1)
#define FREQ_SEL_IRC  FREQ_SEL_12MHz

/* Main clock config
 * We use external crystal and PLL0
 * Note that during PLL lock wait we are running on internal RC
 */
void clock_config(uint32_t freq_sel);

/* return current main clock in HZ */
uint32_t get_main_clock(void);

/* IO config clock */
/* To change GPIO config the io config block must be powered on */
void io_config_clk_on(void);
void io_config_clk_off(void);

/* This is mainly a debug feature, but can be used to provide a clock to an
 * external peripheral */
void clkout_on(uint32_t src, uint32_t div);
void clkout_off(void);



/***************************************************************************** */
/*               System Tick Timer                                             */
/***************************************************************************** */

/* Start the system tick timer
 * Starting the systick timer also resets the internal tick counters.
 * If you need a value that goes beyond one start/stop cycle and accross resets,
 *    then it's up to you to keep track of this using systick_get_tick_count() and/or
 *    systick_get_clock_cycles().
 */
void systick_start(void);

/* Stop the system tick timer */
void systick_stop(void);

/* Reset the system tick timer, making it count down from the reload value again
 * Reseting the systick timer also resets the internal tick counters.
 * If you need a value that goes beyond one start/stop cycle and accross resets,
 *    then it's up to you to keep track of this using systick_get_tick_count() and/or
 *    systick_get_clock_cycles().
 */
void systick_reset(void);

/* Get system tick timer current value (counts at get_main_clock() !) */
uint32_t systick_get_timer_val(void);

/* Check if systick is running (return 1) or not (return 0) */
uint32_t is_systick_running(void);

/* Get the system tick period in ms
 * A vaue of 0 means the system tick timer has not been configured.
 * Note : calls to msleep() or usleep() will configure the system tick timer
 *        with a value of 1ms if it was not configured yet.
 */
uint32_t systick_get_tick_ms_period(void);

/* Get the number of system ticks ... since last wrapping of the counter, which
 * is about 50 days with a 1ms system tick. */
uint32_t systick_get_tick_count(void);

/* Get the number of clock cycles ... since last wrapping of the counter. */
uint32_t systick_get_clock_cycles(void);

/* Power up the system tick timer.
 * ms is the interval between system tick timer interrupts. If set to 0, the default
 *     value is used, which should provide a 1ms period.
 */
void systick_timer_on(uint32_t ms);

/* Removes the main clock from the selected timer block */
void systick_timer_off(void);

/* Register a callback to be called every 'period' system ticks.
 * returns the callback number if registration was OK.
 * returns negative value on error.
 * The callback will get the "global_wrapping_system_ticks" as argument, which wraps every 50 days
 *   or so with a 1ms tick
 */
#define MAX_SYSTICK_CALLBACKS  4
int add_systick_callback(void (*callback) (uint32_t), uint16_t period);
/* Remove a registered callback, given the callback address used to register it. */
int remove_systick_callback(void (*callback) (uint32_t));

/***************************************************************************** */
/* Sleeping functions : these use systick */

/* Set the sleep countdown value
 * A sleep will end when this value reaches 0
 * Note that calls to this function while a sleep() has been initiated will change the
 *   sleep duration ....
 */
void set_sleep(uint32_t ticks);
/* Return current sleep count_down counter */
uint32_t get_sleep(void);

/* Actual sleep function, checks that system tick counter is configured to generate
 * an interrupt to move sleep_count down to 0
 */
uint32_t sleep(void);

void msleep(uint32_t ms);
void usleep(uint32_t us);


#endif /* CORE_SYSTEM_H */
