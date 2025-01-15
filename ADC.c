/*
 * ADC.c
 *
 *  Created on: Feb 19, 2014
 *      Author: tomi
 */

#include <avr/io.h>


void ADC_Init(void)		//Initialization of AVR's ADC, ref. voltage, resolution, inputs, clock...
{
	//ADMUX =_BV(REFS1)|_BV(REFS0)|_BV(ADLAR);			//activate 2,56V internal Vref, Left adjust of result in ADC data reg.
	//ADMUX =_BV(REFS1)|_BV(REFS0);			//activate 2,56V internal Vref

	//ADCSRA =_BV(ADEN)|_BV(ADIE)|_BV(ADPS2)|_BV(ADPS1);			//enable ADC, enable "conv. complete" interrupt, prescaler 128 (ADC clock ~34KHz)
	//ADCSRA =_BV(ADPS2)|_BV(ADPS1);			//enable ADC, prescaler 64 (ADC clock ~15KHz)
	ADCSRA =_BV(ADPS2);			//enable ADC, prescaler 16 (ADC clock ~62KHz)

	//By default ADC use  A0 pin

	//ADMUX|=_BV(MUX0);
	//ADMUX|=_BV(MUX1)|_BV(MUX2)|_BV(MUX3)|_BV(MUX4);			//Internal 1.22V reference  set as input for ADC

	//ADMUX|=_BV(MUX0)|_BV(MUX1)|_BV(MUX2);			//PinA7 set as input for ADC
				//sei; must be activated to work with ADC
}

void ADC_sample(void)			//start conversion of Light intensity from Photoresistor
{
	//ADMUX|=_BV(MUX0)|_BV(MUX1)|_BV(MUX2);			//PinA7 set as input for ADC

	ADCSRA|=_BV(ADSC);			// start of conversion, if this bit is cleared by MCU, conversion finished
	//ADCSRA|=_BV(ADATE);				//auto trigger enable, ADC continuously running
}
