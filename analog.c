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

/*
 *	Defines
 */

/*
 *	Local Functions (Declaration, static)
 */

/*
 *	Global Functions (Definitions)
 */

extern void ANALOG_Init()
{
	// AVCC as Input Source, 100nF Cap between AREF and AVCC needed!
	ADMUX |= (1 << REFS0);
	ADMUX &=  ~( 1 << REFS1);
	// Left Adjust Result, LSB and 2.LSB in ADCL
	//Results are less exact but sufficient for our needs
	ADMUX |= (1 << ADLAR);
	// Select ADC7 as Analog Input
	ADMUX |= (1 << MUX1);
	// Enable ADC
	ADCSRA |= (1 << ADEN);
	// Perform one conversion, part of initialisation
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC))
	{
		asm volatile("nop"); //wait for conversion to complete
	}
}
extern void ANALOG_Service();
extern void ANALOG_Trigger();
extern unsigned int ANALOG_GetValues();

/*
 *	Local Functions (Definitions)
 */
