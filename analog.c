/*
 * analog.c
 *
 *  Created on: 03.04.2015
 *      Author: olli
 */

/*
 *	Includes
 */

#include "analog.h"
#include <util/delay.h>
#include "uart.h"

/*
 *	Defines & variables
 */

unsigned int results;

#define DDRC_MASK ( ( 1 << PC3 ) | ( 1 << PC1 ) )

#define clear_mux() ADMUX &= ~(0x1F); //clear all mux values
#define mux_pc0() clear_mux();
#define mux_pc2() clear_mux(); ADMUX |= (1 << MUX1)
#define pc1_on() PORTC |= (1 << PC1)
#define pc1_off() PORTC &= ~(1<< PC1)
#define pc3_on() PORTC |= (1 << PC3)
#define pc3_off() PORTC &= ~(1<< PC3)

#define adc_in_progress() ADCSRA & (1 << ADSC) //1 while adc in progress
#define start_adc() ADCSRA |= (1 << ADSC)	//start adc conversion


/*
 *	Local Functions (Declaration, static)
 */

/*
 *	Global Functions (Definitions)
 */

extern void ANALOG_Init()
{
	//Set IR-LEDs to OUTPUT;
	DDRC |= DDRC_MASK;
	// AVCC as Input Source, 100nF Cap between AREF and AVCC needed!
	ADMUX = 0x00;
	ADMUX |= (1 << REFS0);
	ADMUX &=  ~( 1 << REFS1);
	// Left Adjust Result, LSB and 2.LSB in ADCL
	//Results are less exact but sufficient for our needs
	ADMUX |= (1 << ADLAR);

	// Enable ADC
	ADCSRA |= (1 << ADEN);
	// Perform one conversion, part of initialization
	ADCSRA |= (1 << ADSC);
	while(adc_in_progress())
	{
		; //wait for conversion to complete
	}
}

extern void ANALOG_Service()
{
	//do an check on both ir sensors
	results = 0x00;
	mux_pc0();	//set adc mux to pc0
	pc1_on();	//turn on first ir led
	_delay_us(150); 		//wait for ir diode to settle
	start_adc();
	while(adc_in_progress())
		;
	results = ADCH;		//save in lower byte of results
	pc1_off();
	mux_pc2();
	pc3_on();
	_delay_us(150);
	start_adc();
	while(adc_in_progress())
		;
	results |= (ADCH << 8);
	pc3_off();
}

extern unsigned int ANALOG_GetValues()
{
	ANALOG_Service();
	return results;
}
/*
 *	Local Functions (Definitions)
 */
