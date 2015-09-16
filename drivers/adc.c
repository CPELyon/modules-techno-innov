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

/* ADC driver for the integrated ADC module of the LPC1224.
 * Refer to LPC1224 documentation (UM10441.pdf) for more information.
 */

#include <stdint.h>
#include "core/lpc_regs_12xx.h"
#include "core/lpc_core_cm0.h"
#include "core/system.h"
#include "core/pio.h"
#include "drivers/adc.h"

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

/* Read the conversion from the given channel (0 to 7) 
 * This function reads the conversion value directly in the data register and
 * always returns a value.
 * Return 0 if the value is a new one and no overrun occured.
 * Return -1 if channel does not exist
 * Retuen 1 if the value is an old one
 * Return 2 if an overrun occured
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
		return 1;
	}
	if (save_reg & LPC_ADC_OVERRUN) {
		return 2;
	}
	return 0;
}

/* Start a conversion on the given channel (0 to 7) */
void adc_start_convertion_once(unsigned int channel, int use_int)
{
	struct lpc_adc* adc = LPC_ADC;
	uint32_t reg_val = 0;

	if (channel > 7)
		return;

	/* Get a clean control register */
	reg_val = adc->ctrl & ~(LPC_ADC_CHANNEL_MASK | LPC_ADC_START_CONV_MASK | LPC_ADC_BURST);

	/* Set conversion channel bit */
	reg_val |= LPC_ADC_CHANNEL(channel);

	/*  Use of interrupts for the specified channel ? */
	if (use_int) {
		/* Set interrupt Bit */
		adc->int_en = LPC_ADC_CHANNEL(channel);
	} else {
		adc->int_en = 0;
	}

	/* Start conversion */
	reg_val |= LPC_ADC_START_CONV_NOW;
	adc->ctrl = (reg_val & LPC_ADC_CTRL_MASK);
}


/* Start burst conversions.
 * channels is a bit mask of requested channels.
 * Use LPC_ADC_CHANNEL(x) (x = 0 .. 7) for channels selection.
 */
void adc_start_burst_conversion(uint8_t channels)
{
	struct lpc_adc* adc = LPC_ADC;
	uint32_t reg_val = 0;

	/* Get a clean control register */
	reg_val = adc->ctrl & ~(LPC_ADC_CHANNEL_MASK | LPC_ADC_START_CONV_MASK | LPC_ADC_BURST);

	/* Set conversion channel bits and burst mode */
	reg_val |= channels;
	reg_val |= LPC_ADC_BURST;

	/*  Use of interrupts for the specified channels ? */
	/* FIXME : Need to choose between one single global interrupt or specific interrupts .... */
	/* FIXME : Actually none. */
	adc->int_en = 0;

	/* Start conversion */
	adc->ctrl = (reg_val & LPC_ADC_CTRL_MASK);
}


/* This should be used to configure conversion start on falling or rising edges of
 * some signals, or on timer for burst conversions.
 */
void adc_prepare_conversion_on_event(uint8_t channels, uint8_t event, int use_int)
{
	struct lpc_adc* adc = LPC_ADC;
	uint32_t reg_val = 0;

	/* Get a clean control register */
	reg_val = adc->ctrl & ~(LPC_ADC_CHANNEL_MASK | LPC_ADC_START_CONV_MASK | LPC_ADC_BURST);
	/* Set conversion channel bits and burst mode */
	reg_val |= channels;
	/* Set conversion condition bits */
	switch (event) {
		case ADC_CONV_ON_CT32B0_MAT0_RISING :
			reg_val |= LPC_ADC_START_CONV_EVENT(LPC_ADC_START_CONV_EDGE_CT32B0_MAT0);
			reg_val |= LPC_ADC_START_EDGE_RISING;
			break;
		case ADC_CONV_ON_CT16B0_MAT0_RISING :
			reg_val |= LPC_ADC_START_CONV_EVENT(LPC_ADC_START_CONV_EDGE_CT16B0_MAT0);
			reg_val |= LPC_ADC_START_EDGE_RISING;
			break;
		default:
			break;
	}

	/*  Use of interrupts for the specified channel ? */
	if (use_int) {
		/* FIXME : Need to choose between one single global interrupt or specific interrupts .... */
	} else {
		adc->int_en = 0;
	}

	/* Enable conversion on selected event */
	adc->ctrl = (reg_val & LPC_ADC_CTRL_MASK);
}



/***************************************************************************** */
/*   ADC Setup : private part : Clocks, Power and Mode   */

void adc_clk_update(void)
{
	struct lpc_adc* adc = LPC_ADC;
	uint32_t main_clock = get_main_clock();
	uint32_t clkdiv = 0;

	/* Configure ADC clock to get the 9MHz sample clock */
	clkdiv = (main_clock / adc_clk_Val);
	adc->ctrl |= ((clkdiv & 0xFF) << 8);
}


void adc_on(void)
{
	struct lpc_sys_control* sys_ctrl = LPC_SYS_CONTROL;
	struct lpc_adc* adc = LPC_ADC;

	/* Disable ADC Interrupt */
	NVIC_DisableIRQ(ADC_IRQ);

	/* Brown-Out detection must be powered to operate the ADC.
	 * See Section 19.2 of UM10441 revision 2.1 or newer for more information */
	sys_ctrl->powerdown_run_cfg &= ~LPC_POWER_DOWN_BOD;

	/* Power-up ADC */
	sys_ctrl->powerdown_run_cfg &= ~LPC_POWER_DOWN_ADC;
	/* Provide clock to ADC */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_ADC, 1);
	adc_clk_update();

	/* Prevent unconfigured conversion start */
	adc->ctrl &= ~LPC_ADC_START_CONV_MASK;

	/* Remove the default global interrupt enabled setting */
	adc->int_en = 0;

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

