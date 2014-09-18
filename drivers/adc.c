/****************************************************************************
 *  drivers/adc.c
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
/*                Analog to Digital Converter (ADC)                            */
/***************************************************************************** */

#include <stdint.h>
#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"
#include "core/system.h"
#include "core/pio.h"

/* Should be as near to 9MHz as possible */
#define adc_clk_Val  9000000



/***************************************************************************** */
/* Generic ADC handler */
void ADC_Handler(void)
{
/*
	volatile struct lpc_adc* adc = LPC_ADC;
	uint32_t status = adc->status;
*/
	/* .... What to do ... is specific to your application */
	/* FIXME : Add an handler callback. */
}

/* Start a conversion on the given channel (0 to 7) */
void adc_start_convertion_once(unsigned int channel, int use_int)
{
	struct lpc_adc* adc = LPC_ADC;

	if (channel > 7)
		return;

	/* Set conversion channel bit */
	adc->ctrl = ((adc->ctrl & ~LPC_ADC_CHANNEL_MASK) | LPC_ADC_CHANNEL(channel));
	if (use_int) {
		/* Set interrupt Bit */
		adc->int_en = LPC_ADC_CHANNEL(channel);
	} else {
		adc->int_en = 0;
	}
	/* Clear burst conversion if running ? */
	adc->ctrl &= ~LPC_ADC_BURST;
	/* Start conversion */
	adc->ctrl = ((adc->ctrl & ~LPC_ADC_START_CONV_MASK) | LPC_ADC_START_CONV_NOW);
}


/* Read the conversion from the given channel (0 to 7) 
 * This function reads the conversion value directly in the data register and
 * always returns a value.
 * Return 1 if the value is a new one, else return 0.
 * Return -1 if channel does not exist
 */
int adc_get_value(uint16_t * val, int channel)
{
	struct lpc_adc* adc = LPC_ADC;
	uint32_t save_reg = 0;

	if (channel > 7)
		return -1;

	/* Save the whole register as some bits are cleared when register is read */
	save_reg = adc->data[channel];
	*val = ((save_reg >> LPC_ADC_RESULT_SHIFT) & LPC_ADC_RESULT_MASK);
	/* Has this conversion value been already read ? */
	if (! (save_reg & LPC_ADC_CONV_DONE)) {
		return 0;
	}
	return 1;
}

/* Start burst conversions.
 * channels is a bit mask of requested channels.
 * Use LPC_ADC_CHANNEL(x) (x = 0 .. 7) for channels selection.
 */
void adc_start_burst_conversion(uint8_t channels)
{
	struct lpc_adc* adc = LPC_ADC;
	uint32_t reg_val = 0;

	/* Set conversion channel bits and burst mode */
	reg_val = ((adc->ctrl & ~LPC_ADC_CHANNEL_MASK) | channels);
	reg_val |= LPC_ADC_BURST;
	adc->ctrl = (reg_val & LPC_ADC_CTRL_MASK);
}


/* Unsupported Yet */
/* This should be used to configure conversion start on falling or rising edges of
 * some signals, or on timer for burst conversions.
 */
void adc_prepare_conversion_on_event(void)
{
	/* Unsupported Yet */
}



/***************************************************************************** */
/*   ADC Setup : private part : Clocks, Pins, Power and Mode   */
struct pio adc_pins[] __attribute__ ((weak)) = {
	ARRAY_LAST_PIN,
};

static void set_adc_pins(void)
{
	int i = 0;
	/* Configure ADC pins */
	for (i = 0; adc_pins[i].port != 0xFF; i++) {
		config_pio(&adc_pins[i], LPC_IO_ANALOG);
	}
}

void adc_clk_update(void)
{
	struct lpc_adc* adc = LPC_ADC;
	uint32_t main_clock = get_main_clock();
	uint32_t clkdiv = 0;

	/* Configure ADC */
	clkdiv = (main_clock/adc_clk_Val);
	adc->ctrl |= ((clkdiv & 0x0F) << 8);
}


void adc_on(void)
{
	struct lpc_sys_control* sys_ctrl = LPC_SYS_CONTROL;
	struct lpc_adc* adc = LPC_ADC;

	/* Disable ADC Interrupt */
	NVIC_DisableIRQ(ADC_IRQ);
	/* Power-up ADC */
	sys_ctrl->powerdown_run_cfg &= ~LPC_POWER_DOWN_ADC;
	/* Provide clock to ADC */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_ADC, 1);
	adc_clk_update();

	/* Prevent unconfigured conversion start */
	adc->ctrl &= ~LPC_ADC_START_CONV_MASK;

	/* Configure ADC pins */
	set_adc_pins();

	/* Enable ADC Interrupt */
	NVIC_EnableIRQ(ADC_IRQ);
}

void adc_off(void)
{
	struct lpc_sys_control* sys_ctrl = LPC_SYS_CONTROL;

	/* Disable ADC Interrupt */
	NVIC_DisableIRQ(ADC_IRQ);
	/* Power Down ADC */
	sys_ctrl->powerdown_run_cfg |= LPC_POWER_DOWN_ADC;
	/* Remove clock from ADC block */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_ADC, 0);
}

