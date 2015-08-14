/****************************************************************************
 *  drivers/timers.c
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
/*                Timers                                                       */
/***************************************************************************** */

#include <stdint.h>
#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"
#include "core/system.h"
#include "core/pio.h"
#include "lib/string.h"
#include "drivers/timers.h"


/* These are local to our file */
struct timer_device
{
 	struct lpc_timer* regs;
	uint32_t power_bit;
	uint32_t irq;
	void (*callback)(uint32_t); /* Possible RX callback */
};
static struct timer_device timer_devices[NUM_TIMERS] = {
	{ LPC_TMR16B0, LPC_SYS_ABH_CLK_CTRL_CT16B0, TIMER0_IRQ }, 
	{ LPC_TMR16B1, LPC_SYS_ABH_CLK_CTRL_CT16B1, TIMER1_IRQ },
	{ LPC_TMR32B0, LPC_SYS_ABH_CLK_CTRL_CT32B0, TIMER2_IRQ },
	{ LPC_TMR32B1, LPC_SYS_ABH_CLK_CTRL_CT32B1, TIMER3_IRQ },
};



/* Handlers */
void TIMER_Handler(struct timer_device* timer)
{
	uint32_t intr_flags = timer->regs->int_reg; /* Backup the flags */

	/* Clear the interrupt */
	timer->regs->int_reg = intr_flags;
	/* And call the user routine if one has been registered */
	if (timer->callback != NULL) {
		timer->callback(intr_flags);
	}
}
void TIMER_0_Handler(void)
{
	TIMER_Handler(&timer_devices[0]);
}
void TIMER_1_Handler(void)
{
	TIMER_Handler(&timer_devices[1]);
}
void TIMER_2_Handler(void)
{
	TIMER_Handler(&timer_devices[2]);
}
void TIMER_3_Handler(void)
{
	TIMER_Handler(&timer_devices[3]);
}



/* Start the timer :
 * Remove the reset flag if present and set timer enable flag.
 * Timer must be turned on and configured (no checks done here).
 */
void timer_start(uint32_t timer_num)
{
	if (timer_num >= NUM_TIMERS)
		return;
	/* Remove reset flag and set timer enable flag */
	timer_devices[timer_num].regs->timer_ctrl = LPC_TIMER_COUNTER_ENABLE;
}
void timer_continue(uint32_t timer_num) __attribute__ ((alias ("timer_start")));
/* Pause the timer counter, does not reset */
void timer_pause(uint32_t timer_num)
{
	if (timer_num >= NUM_TIMERS)
		return;
	/* Remove timer enable flag */
	timer_devices[timer_num].regs->timer_ctrl = 0;
}
/* Stops and resets the timer counter */
void timer_stop(uint32_t timer_num)
{
	if (timer_num >= NUM_TIMERS)
		return;
	/* Remove timer enable flag and request reset */
	timer_devices[timer_num].regs->timer_ctrl = LPC_TIMER_COUNTER_RESET;
	/* Remove reset flag */
	timer_devices[timer_num].regs->timer_ctrl = 0;
}
/* Resets the timer and lets it count again imediately */
void timer_restart(uint32_t timer_num)
{
	if (timer_num >= NUM_TIMERS)
		return;
	/* Set timer reset flag */
	timer_devices[timer_num].regs->timer_ctrl = LPC_TIMER_COUNTER_RESET;
	/* Remove reset flag and start counter */
	timer_devices[timer_num].regs->timer_ctrl = LPC_TIMER_COUNTER_ENABLE;
}

uint32_t timer_get_capture_val(uint32_t timer_num, uint32_t channel)
{
	if (timer_num >= NUM_TIMERS)
		return 0;
	/* FIXME */
	return 0;
}
uint32_t timer_get_counter_val(uint32_t timer_num)
{
	if (timer_num >= NUM_TIMERS)
		return 0;
	return timer_devices[timer_num].regs->timer_counter;
}

/* Change the match value of a single timer channel */
void timer_set_match(uint32_t timer_num, uint32_t channel, uint32_t val)
{
	if (timer_num >= NUM_TIMERS)
		return;
	if (channel > 3)
		return;

	timer_devices[timer_num].regs->match_reg[channel] = val;
}


/***************************************************************************** */
/*   Timer Setup */
/* Returns 0 on success
 * Takes a timer number and a timer config structure as arguments.
 * Refer to timer config structure for details.
 */
int timer_setup(uint32_t timer_num, const struct timer_config* conf)
{
	struct timer_device* timer = NULL;
	int i = 0;
	if (timer_num >= NUM_TIMERS)
		return -ENODEV;
	timer = &(timer_devices[timer_num]);

	/* Configure the reset on capture functionality */
	if (conf->reset_on_capture != 0x00) {
		timer->regs->count_ctrl = LPC_COUNTER_CLEAR_ON_EVENT_EN;
		timer->regs->count_ctrl |= ((conf->reset_on_capture & 0x07) << LPC_COUNTER_CLEAR_ON_EVENT_SHIFT);
	}

	switch (conf->mode) {
		case LPC_TIMER_MODE_TIMER:
			timer->regs->capture_ctrl = 0; /* Timer mode ! */
			timer->regs->count_ctrl = LPC_COUNTER_IS_TIMER;
			break;
		case LPC_TIMER_MODE_COUNTER:
			if ((conf->config[0] & 0x03) == 0x00) {
				return -EINVAL;
			}
			/* Must set capture chanel N config to 0b000 in capture control register,
			 * (see remarks in user manual UM10441 page 268 section 14.7.11)
			 * Use the LPC_COUNTER_INC_INPUT(x) set by the user to do so automatically
			 */
			timer->regs->capture_ctrl &= ~LPC_TIMER_CAPTURE_ERASE(((conf->config[0] >> LPC_COUNTER_INC_INPUT_SHIFT) & 0x03) * 3);
			/* Configure the counter */
			timer->regs->count_ctrl |= (conf->config[0] & 0x0F);
			break;
		case LPC_TIMER_MODE_CAPTURE:
			timer->regs->capture_ctrl = 0;
			for (i = 0; i < NUM_CHANS; i++) {
				timer->regs->capture_ctrl |= ((conf->config[i] & 0x07) << LPC_TIMER_CAPTURE_SHIFT(i));
			}
			break;
		case LPC_TIMER_MODE_MATCH:
			timer->regs->match_ctrl = 0;
			timer->regs->external_match = 0;
			for (i = 0; i < NUM_CHANS; i++) {
				timer->regs->match_ctrl |= ((conf->config[i] & 0x07) << LPC_TIMER_MATCH_SHIFT(i));
				timer->regs->match_reg[i] = conf->match[i];
				timer->regs->external_match |= ((conf->ext_match_config[i] & 0x03) << (LPC_TIMER_EXT_MATCH0_SHIFT + i*2));
			}
			break;
		case LPC_TIMER_MODE_PWM:
			if (conf->match[ conf->config[1] ] == 0) {
				return -EINVAL; /* Force use of Channel 3 for PWM cycle length */
			}
			/* Activate selected PWM channels 0 to 2 */
			timer->regs->pwm_ctrl = (conf->config[0] & 0x07);
			/* Use Channel 3 for PWM cycle length as recommended in the manual */
			timer->regs->match_ctrl &= ~(LPC_TIMER_MATCH_ERASE(conf->config[1]));
			timer->regs->match_ctrl |= (LPC_TIMER_RESET_ON_MATCH << LPC_TIMER_MATCH_SHIFT(conf->config[1]));
			for (i = 0; i < NUM_CHANS; i++) {
				timer->regs->match_reg[i] = conf->match[i];
			}
			break;
		case LPC_TIMER_MODE_PWD:
			break;
	}
	return 0; /* Config OK */
}


/* Power up a timer.
 * Note that clkrate should be a divider of the main clock frequency chosed
 *   for your application as it will be used to divide the main clock to get
 *   the prescaler value.
 * Set clkrate to 0 to disable the prescaler.
 */
void timer_on(uint32_t timer_num, uint32_t clkrate, void (*callback)(uint32_t))
{
	struct timer_device* timer = NULL;
	uint32_t prescale; /* The clock divider for the counter */

	if (timer_num >= NUM_TIMERS)
		return;
	timer = &(timer_devices[timer_num]);

	NVIC_DisableIRQ( timer->irq );
	/* Power up the timer */
	subsystem_power(timer->power_bit, 1);
	/* Reset counter on next PCLK positive edge, and disable counter */
	timer->regs->timer_ctrl = LPC_TIMER_COUNTER_RESET;

	/* Store the callback, OK even if none given */
	timer->callback = callback;

	/* Set the prescaler value */
	if (clkrate == 0) {
		prescale = 0;
	} else {
		prescale = (get_main_clock() / clkrate) - 1;
	}
	timer->regs->prescale = prescale;

	NVIC_EnableIRQ(timer_devices[timer_num].irq);
}

/* Removes the main clock from the selected timer block */
void timer_off(uint32_t timer_num)
{
	if (timer_num >= NUM_TIMERS)
		return;
	NVIC_DisableIRQ( timer_devices[timer_num].irq );
	subsystem_power(timer_devices[timer_num].power_bit, 0);
}
