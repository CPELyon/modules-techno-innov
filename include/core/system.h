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
#define EIO          5 /* Bad one: Illegal start or stop, or illegal state in i2c state machine */
#define EAGAIN      11 /* Device already in use */
#define EFAULT      14 /* address above eeprom size */
#define EBUSY       16 /* Device or ressource Busy or Arbitration lost */
#define ENODEV      19 /* No such device */
#define EINVAL      22 /* Invalid argument */
#define EBADFD      77 /* Device not initialized */
#define EREMOTEIO  121 /* Device did not acknowledge */


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


/* This "msleep" is a simple wait routine which is close to a millisecond sleep
 * whith a CPU Clock of 24MHz, but has no exact relation to time.
 * It is highly dependent to CPU clock speed anyway.
 * Note : This is an active sleep !
 */
static inline void msleep(uint32_t ms)
{
    volatile uint32_t dec = ms * 2667;
    while (dec--);
}

/* Something that's not too far from a microsecond sleep at 24MHz CPU Clock
 * Note : This is an active sleep !
 */
static inline void usleep(uint32_t us)
{
    volatile uint32_t dec = us * 2;
    while (dec--);
}

#endif /* CORE_SYSTEM_H */
