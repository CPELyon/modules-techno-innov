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

/* Configure the brown-out detection */
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

/* return current main clock */
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

/* Start the system tick timer */
void systick_start(void);
/* Stop the system tick timer */
void systick_stop(void);
/* Reset the system tick timer, making it count down from the reload value again */
void systick_reset(void);
/* Get system tick timer current value */
uint32_t systick_get_timer_val(void);

/* Get the system tick */
uint32_t systick_get_tick_count(void);

/* Power up the system tick timer.
 * ms is the interval between system tick timer interrupts. If set to 0, the default
 *     value is used, which should provide a 1ms period.
 */
void systick_timer_on(uint32_t ms);

/* Removes the main clock from the selected timer block */
void systick_timer_off(void);


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
void usleep(uint32_t ms);


#endif /* CORE_SYSTEM_H */
