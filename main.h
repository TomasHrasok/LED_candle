/*
 * main.h
 *
 *  Created on: Oct 18, 2015
 *      Author: tomi
 */

#ifndef MAIN_H_
#define MAIN_H_

//#define test		// debug only, shorter timeouts
//#define LED_debug	// LED blink-out the status

#if defined (__AVR_ATtiny44A__)
	/*----- Declaration of Outputs ------*/
//	#define LED						PA6		// (OC1A) LED output pin, 16bit PWM Timer1 output
	#define LED						PA7		// (OC0B) LED output pin, 8bit PWM Timer0 output
	#define LED_Port				PORTA 	// port definition
	#define LED_Port_DIR			DDRA 	// port direction definition

	/*----- Inputs ------*/
	#define Tilt_SW					PB2		// (INT0) to wakeUp MCU from power-down
	#define Tilt_SW_Port 			PORTB	// port definition
	#define Tilt_SW_Pin 			PINB	// Port input definition
	#define Tilt_SW_Port_DIR		DDRB 	// port direction definition

	#define Light_SensorVcc			PA3		// GPIO to supply VCC for opto-transistor
	#define Light_SensorVcc_Port	PORTA	// port definition
	#define Light_SensorVcc_DIR		DDRA 	// port direction definition

	#define Light_Sensor			PA1		// (ADC1)
	#define Light_Sensor_Port		PORTA	// port definition
	#define Light_Sensor_Pin		PINA	// port definition
	#define Light_Sensor_Port_DIR	DDRA 	// port direction definition

	#define WDT_OwerFlow_IRQ		WATCHDOG_vect
	#define INT0_IRQ				EXT_INT0_vect

	#define WDTCR					WDTCSR		// porting AtTiny2313 to AtTiny44A registers

#elif defined (__AVR_ATtiny2313A__)
	// ATtiny2313 does not have ADC!
	/*----- Declaration of Outputs ------*/
	#define LED						PB3		// (OC1A) LED output pin
	#define LED_Port				PORTB 	// port definition
	#define LED_Port_DIR			DDRB 	// port direction definition

	/*----- Inputs ------*/
	#define Tilt_SW					PD2		// (INT0) to wakeUp MCU from power-down
	#define Tilt_SW_Port 			PORTD	// port definition
	#define Tilt_SW_Pin 			PIND	// Port input definition
	#define Tilt_SW_Port_DIR		DDRD 	// port direction definition

	#define Light_SensorVcc			PA1		// GPIO to supply VCC for opto-transistor
	#define Light_SensorVcc_Port	PORTA	// port definition
	#define Light_SensorVcc_DIR		DDRA 	// port direction definition

	#define Light_Sensor			PA0		// GPIO instead of ADC (for testing)
	#define Light_Sensor_Port		PORTA	// port definition
	#define Light_Sensor_Pin		PINA	// port definition
	#define Light_Sensor_Port_DIR	DDRA 	// port direction definition

	#define WDT_OwerFlow_IRQ		WDT_OVERFLOW_vect
	#define INT0_IRQ				INT0_vect

#else
	#warning "device type not defined"
#endif

#ifdef test
	#define LIGHT_CHECK_INTERVAL		2		// check light every cycle
	#define	WDT_TIMEOUT					2		// Watchdog trigger period
	#define DEFAULT_LIGHT_TIME			60		// seconds
#else
	#define LIGHT_CHECK_INTERVAL		120		// 240 seconds = 4 minutes
	#define	WDT_TIMEOUT					8		// Watchdog trigger period
	#define DEFAULT_LIGHT_TIME			5*3600	// 5 hours
//	#define DEFAULT_LIGHT_TIME			1*3600	// 1 hours
#endif //debug

#define INT0_set_logical_change()		MCUCR = (MCUCR & ~(_BV(ISC01)|_BV(ISC00))) | _BV(ISC00);		// INT0 executed on any log change
#define INT0_set_low_level()			MCUCR = (MCUCR & ~(_BV(ISC01)|_BV(ISC00)));			    		// INT0 executed on low level pin state

#define	WDT_SET_IRQ_mode()	WDTCR |= _BV(WDIE);				// (re)activate Interrupt when WatchDog overflow (deactivate WDT reset)
#define WDT_unlock()		WDTCR |= _BV(WDCE) | _BV(WDE);		// (re)activate Interrupt when WatchDog overflow (deactivate WDT reset)

#define LED_OFF()			LED_Port |= _BV(LED);
#define LED_ON()			LED_Port &= ~_BV(LED);
#define LED_TOGGLE()		LED_Port ^= (1 << LED);

typedef enum {Init, GetLightCondition, PwON, FadeToPwOFF, PwOFF, UpdateTimer} tStatus;

#endif /* MAIN_H_ */
