/*
 * main.c
 *
 *  Created on: Dec 4, 2024
 *      Author: tomi
 *
 *      Temporary MCU: ATtiny2313A
 *      Final MCU: 	   ATtiny44A
 *      LED candle driven by:
 *      - PhotoTransistor - light up only in night
 *      - Timer - does not shine after 20:35
 *      - Estimate winter / summer by detecting DayTime
 *      	- adapting light up time accordingly
 *      - switch on/off by tilt-switch
 *      - during light measurement deactivate LED light
 *      	-connected to INT0 >> wakeup from power-down
 *      	MCU wake-Up by WDT IRQ > max. period = 8 seconds
 *      	TODO: External INT0 resistor to conserve power ?
 *
 *		Event Boolean matrix:
 *	   --|---------------|-----------------|----------------|--------------|--
 *       |    Is day     | IsSwitch active |   LED state    |	Watchdog   |
 *	   --|---------------|-----------------|----------------|--------------|--
 *	     |    YES	     |    YES	       |      OFF  	    |      ON      |
 *	     |    YES	     |    NO	       |      OFF  	    |      OFF     |
 *	     |    NO	     | 	  YES		   |      ON  	    |      ON      |
 *	     |    NO	     |    NO		   |      OFF  	    |      OFF     |
 *
 */


#include "main.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdbool.h>


#if defined (__AVR_ATtiny44A__)
	// ATtiny2313 does not have ADC!
	#include "ADC.h"
#endif

//================= Definitions ======================
//set fuses as 8MHz int.RC clock, 1:8 clock divider, BOD off
/*
 * original FuseBits:
 * 		lfuse:0x64 >>	Divides clk by 8, Clock source: int. RC 8MHz
 * 		hfuse:0xdf >> 	Default: BODLEVEL disabled, ISP enabled
 * 		efuse:0xff >> 	SELFPRGEN 	>> self programming disabled
 * 		lock :0xff >>	Lock bits 	>>	Unlocked
 * 		IDLE current reached 0.324mA
 *
 * updated FuseBits to internal RC 128KHz oscillator
 * 		 lfuse:0xE6 >>	Divides clk by 1, Clock source >> 128KHz
 * 		 IDLE current reached 0.15mA
 *
 * updated FuseBits to internal RC 4MHz oscillator
 * 		 lfuse:0xE2 >>	Divides clk by 8, Clock source >> 500KHz
 * 		 IDLE current reached 0.23mA
 *
 */

#ifdef test

#else

#endif

volatile uint8_t CurrentSeconds = 0;

tStatus State;

//================= Function Declarations ======================
void Ports_Init(void);			//Initialization of I/O ports
bool IsSwitchActive(void);		//When LED candle tilted, return=0, otherwise =1
bool IsDayLight(void);			//During night return=0, during day return =1
void WDT_enable(void);
void WDT_disable(void);

ISR (WDT_OwerFlow_IRQ)				//watchdog interrupt
{
	MCUCR &=~_BV(SE);		//turn off sleep mode
	WDT_SET_IRQ_mode();		//reactivate IRQ instead of reset
	CurrentSeconds+=8;			// 8 seconds passed
	State = UpdateTimer;
}

ISR (INT0_IRQ)				//INT0 external pin low level interrupt occurred
{
	MCUCR &=~_BV(SE);				//turn off sleep mode
	INT0_set_logical_change();		// prevent INT0 call in loop when in low level
	// get state of TiltSwitch
	if(IsSwitchActive()==true)
	{
		WDT_enable();
		State=GetLightCondition;
	}
	cli();		// disable further INT0 events until processing finished
//	LED_TOGGLE();
//	_delay_ms(80);
//	LED_TOGGLE();
//	_delay_ms(80);
//	LED_TOGGLE();
//	_delay_ms(80);
//	LED_TOGGLE();
}


int main()
{
	State = Init;
	while(1)
	{
		switch (State)		// blinking patterns
		{
			case Init:
			{
				/* Clock init:
				 * 	- Default clk = 8MHz internal RC, prescaler = 8 >> clk 1MHz
				 * 	- Available prescaler = 1~256 >> 31KHz @ 8MHz
				 * 	- internal 1MHz @ 0.65mA in RUN MODE
				 * 	- internal 1MHz @ 0.1mA in IDLE (PRR=0xFF) >>> preferable mode
				 *  - internal RC 128KHz available @ 0.02mA in IDLE	>> risk of bricked chip - clock too low for further programming
				 */

				Ports_Init();
				#if defined (__AVR_ATtiny44A__)
					//TODO ADC_Init();
				#endif

				//set WDT, interrupts
				sei();
				WDT_enable();

				State = PwOFF;
				break;
			}
			case GetLightCondition:
			{
				if(IsDayLight() == true)
				{
					State = FadeToPwOFF;
				}
				else
				{
					LED_ON();
					State = PwOFF;		// go to IDLE state
				}
				break;
			}
			case FadeToPwOFF:
			{
				// PWM Fade LED to Power Off state
				//_delay_ms(2000);
				LED_OFF();
				// Switch to PwOFF
				State = PwOFF;
				break;
			}
			case PwOFF:
			{
				_delay_ms(10);
				sei();
				// default sleep mode
				set_sleep_mode(SLEEP_MODE_IDLE);
//				set_sleep_mode(SLEEP_MODE_STANDBY);
//				set_sleep_mode(SLEEP_MODE_PWR_DOWN);

				if(IsSwitchActive()==false)
				{
					// Candle tilted, WDT OFF, full PowerDown
					LED_OFF();
					WDT_disable();
					INT0_set_low_level();
					set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//					set_sleep_mode(SLEEP_MODE_IDLE);
				}
				sleep_mode();
				// WakeUp from sleep (WDT or TiltSwitch)
				// New State already set in IRQs
				break;
			}
			case UpdateTimer:
			{
//				LED_TOGGLE();
//				_delay_ms(10);
//				LED_TOGGLE();
				//execution triggered every 8 seconds
				if(CurrentSeconds >= TRIGGER_4MINUTES)
				{
					// time to measure actual light conditions
					CurrentSeconds=0;
					State = GetLightCondition;
				}
				else
				{
					// Nothing to be done >>> sleep mode
					State = PwOFF;
				}
				break;
			}
			default:
			{
				break;
			}
		}
	}
}

void Ports_Init(void)		//Initialization of I/O ports
{
	// activate pull-up on all pins to reduce consumption (undefined log. level on inputs)
#if defined (__AVR_ATtiny44A__)
	//TODO adapt
	PORTA = 0x06;
	PORTB = 0xFF;

	DIDR0 = _BV(ADC7D)|_BV(ADC5D)|_BV(ADC4D)|_BV(ADC3D)|_BV(ADC2D);		// Disable digital inputs on Analog Comparator (PB0, PB1 @ AtTiny2313A)
#elif defined (__AVR_ATtiny2313A__)
	PORTA = 0x06;
	PORTB = 0xFF;
	PORTD = 0x7F;

	DIDR = _BV(AIN1D)|_BV(AIN0D);		// Disable digital inputs on Analog Comparator (PB0, PB1 @ AtTiny2313A)
#endif

	LED_Port &=~_BV(LED);				// Set IO state to LOW
	LED_Port_DIR |=_BV(LED);			// Set IO DIR to OUTPUT

	Tilt_SW_Port |=_BV(Tilt_SW);		// activate PULL-UP resistor?
	Tilt_SW_Port_DIR &=~_BV(Tilt_SW);	// Set IO DIR to INPUT

	// Configure external INT0 pin interrupt
	INT0_set_logical_change();
	GIMSK |= _BV(INT0);					// Enable INT0 interrupt

	//turn off not used peripherals
	ACSR |=_BV(ACD);					// turn of Analog comparator permanently
//	DIDR0 |=_BV(AIN1D)|_BV(AIN0D);		// deactivate Digital inputs of PB1, PB0
	PRR |=_BV(PRTIM0)|_BV(PRTIM1)|_BV(PRUSI)|_BV(PRADC);	//turn off timers, ADC, USI, UART

	// LED init blink sequence
	LED_ON();
	_delay_ms(450);
	LED_OFF();
}

bool IsSwitchActive(void)		//When LED candle tilted, return false, otherwise true
{
//	return ((Tilt_SW_Pin & _BV(Tilt_SW)) != 0u);
	if (Tilt_SW_Pin & _BV(Tilt_SW))
	{
		return false;
	}
	else return true;
}

bool IsDayLight(void)			//During night return=0, during day return =1
{
	bool DayLight = false;
	// Disable temporarily LED during measurement
	LED_OFF();		// proper LED ON/OFF state state is recovered in GetLightCondition state
	// Enable Vcc for opto-transistor
	Light_SensorVcc_Port |=_BV(Light_SensorVcc);		// Set IO state to HIGH
	Light_SensorVcc_DIR  |=_BV(Light_SensorVcc);		// Set IO DIR to OUTPUT

	_delay_ms(5);
	// read opto-transistor state
	if ((Light_Sensor_Pin & _BV(Light_Sensor)) == 0u)
	{
		DayLight = true;
	}

	// Disable Vcc for opto-transistor
	Light_SensorVcc_Port &=~_BV(Light_SensorVcc);		// Set IO state to LOW
	Light_SensorVcc_DIR  &=~_BV(Light_SensorVcc);		// Set IO DIR to INPUT, really necessary? TODO maybe output to decrease PWR?

	return DayLight;
}

void WDT_enable(void)
{
	WDT_unlock();			// WDT >> ON
#ifdef test
	WDTCR = _BV(WDIE)|_BV(WDP0)|_BV(WDP1)|_BV(WDP2);	//2 seconds
#else
	WDTCR = _BV(WDIE)|_BV(WDP0)|_BV(WDP3);	//8 seconds
#endif
}

void WDT_disable(void)
{
	WDT_unlock();
	WDTCR = _BV(WDCE);	//WDT OFF
}
