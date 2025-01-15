/*
 * ADC.h
 *
 *  Created on: Feb 19, 2014
 *      Author: tomi
 */

#ifndef ADC_H_
#define ADC_H_

#define ADC_enable() 				ADCSRA  |=_BV(ADEN);

#define ADC_disable()				ADCSRA &=~_BV(ADEN);

void ADC_Init(void);		//Initialization of AVR's ADC, ref. voltage, resolution, inputs, clock...

void ADC_sample(void);		// start ADC conversion

#endif /* ADC_H_ */
