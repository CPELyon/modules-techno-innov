/****************************************************************************
 *  drivers/adc.h
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

#ifndef DRIVERS_ADC_H
#define DRIVERS_ADC_H


#include <stdint.h>

/***************************************************************************** */
/*                Analog to Digital Converter (ADC)                            */
/***************************************************************************** */

/* ADC driver for the integrated ADC module of the LPC1224.
 * Refer to LPC1224 documentation (UM10441.pdf) for more information.
 */

enum adc_events {
	/* CT32B0 */
	ADC_CONV_ON_CT32B0_CAP0_RISING,
	ADC_CONV_ON_CT32B0_CAP0_FALLING,
	ADC_CONV_ON_CT32B0_MAT0_RISING,
	ADC_CONV_ON_CT32B0_MAT0_FALLING,
	ADC_CONV_ON_CT32B0_MAT1_RISING,
	ADC_CONV_ON_CT32B0_MAT1_FALLING,
	/* CT16B0 */
	ADC_CONV_ON_CT16B0_CAP0_RISING,
	ADC_CONV_ON_CT16B0_CAP0_FALLING,
	ADC_CONV_ON_CT16B0_MAT0_RISING,
	ADC_CONV_ON_CT16B0_MAT0_FALLING,
	ADC_CONV_ON_CT16B0_MAT1_RISING,
	ADC_CONV_ON_CT16B0_MAT1_FALLING,
};


/* Read the conversion from the given channel (0 to 7)
 * This function reads the conversion value directly in the data register and
 * always returns a value.
 * Return 1 if the value is a new one, else return 0.
 * Return -1 if channel does not exist
 */
int adc_get_value(uint16_t * val, int channel);

/* Start a conversion on the given channel (0 to 7)
 * Unsupported yet : Set use_int to 1 to have your interrupt callback called upon conversion done.
 */
void adc_start_convertion_once(unsigned int channel, int use_int);

/* Start burst conversions.
 * channels is a bit mask of requested channels.
 * Use LPC_ADC_CHANNEL(x) (x = 0 .. 7) for channels selection.
 */
void adc_start_burst_conversion(uint8_t channels);

/* This should be used to configure conversion start on falling or rising edges of
 * some signals, or on timer for burst conversions.
 */
void adc_prepare_conversion_on_event(uint8_t channels, uint8_t event, int use_int);


/***************************************************************************** */
/*   ADC Setup : private part : Clocks, Pins, Power and Mode   */
void adc_clk_update(void);
void adc_on(void);
void adc_off(void);

#endif /* DRIVERS_ADC_H */

