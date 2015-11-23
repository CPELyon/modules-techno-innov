/****************************************************************************
 *   core/watchdog.h
 *
 * Watchdog support
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

/*
 * This file implements support of the Windowed Watchdog (WWDT)
 */


#ifndef CORE_WATCHDOG_H
#define CORE_WATCHDOG_H

#include <stdint.h>

#define WDT_CLK_POWER_LOCK  (0x01 << 0)
#define WDT_CLK_SRC_LOCK    (0x01 << 1)
#define WDT_EN_LOCK         (0x01 << 2)
#define WDT_TIMER_VAL_LOCK  (0x01 << 3)
#define WDT_POWER_DOWN_LOCK (0x01 << 4)


#define WDT_CLK_IRC 0
#define WDT_CLK_WDTCLK 1
struct wdt_config {
 	/* clk_sel is either 0 (IRC) or 1 (WDTCLK). The corresponding clock source will be powered on. */
	int clk_sel;
	int intr_mode_only; /* If set to 1, a watchdog timeout will trigger an interrupt instead of a reset */
	uint32_t locks; /* Bitfield from WDT_*_LOCK defined in watchdog.h */
	/* Number of clk_src clocks before the watchdog timer times out. Will be divided by 4 to give
	 *   the watchdog reload value */
	uint32_t nb_clk;  /* 0x3FF to 0x03FFFFFF */
	/* The next two values are relative to the timer value, not the number of clk_src clocks. */
	uint32_t wdt_window; /* Watchdog window value : 0x100 to 0x00FFFFFF */
	uint16_t wdt_warn; /* 0x00 to 0x3FF */
};

/***************************************************************************** */
void watchdog_feed(void);


/* Lock the watchdog clock source. Once the clock is locked, the configuration is
 * permanent, there's no way to change it. Make all configuration steps before locking
 * the watchdog Clock source.
*/
void watchdog_lock_clk_src(void);

/* Lock the watchdog clock source power.
 * Once locked, writes to the current watchdog clock power bits in powerdown_*_cfg will
 *   have no effect.
 * It is still possible to switch the watchdog clock source and turn of the clock if the
 *   watchdog clock source has not been locked yet.
 */
void watchdog_lock_clk_src_power(void);

/* Lock the watchdog timer value */
void watchdog_lock_timer_val(void);

/* Change the watchdog timer value, if not protected */
void watchdog_set_timer_val(uint32_t nb_clk);

/* Lock the Watchdog enable bit.
 * It is still possible to disable the watchdog by setting it's clock to an unpowered
 *   source if you did not lock the watchdog clock source and clock source power.
 */
void watchdog_lock_enable(void);

/* Lock the watchdog and all related features.
 * Calls all the other watchdog_lock_* functions (clk_src, clk_src_power, timer_val, enable).
 */
void watchdog_lock_full(void);

/* Disable deep power down mode entry
 * Calls to wfi() will allow entry in sleep and deep-sleep modes, but not deep-power-down mode.
 */
void watchdog_disable_power_down(void);

/*
 * Configure the watchdog.
 * clk_sel is either 0 (IRC) or 1 (WDTCLK). The corresponding clock source will be powered on.
 * Note : only WDTCLK is running in deep power down mode
 * Note : protecting the clock source power will prevent turning off the IRC for power saving
 *   if it is selected as main clock source.
 */
void watchdog_config(const struct wdt_config* wd_conf);


/*
 * Stop the watchdog
 * This function can be used during system operation to stop the watchdog if it has not
 *   been locked or protected against clock source modification, for example when entering
 *   sleep or deep sleep.
 * It will also try to power-down the oscilators if not used for main clock.
 * Return 0 if a solution has been found to stop the watchdog, or -1 if watchdog is still
 *   running after this call.
 * TODO : Check this function, and implement the missing cases
 */
int stop_watchdog(void);


/*
 * Disable the watchdog
 * This function can be used upon system startup to disable watchdog operation
 */
void startup_watchdog_disable(void);


#endif /* CORE_WATCHDOG_H */
