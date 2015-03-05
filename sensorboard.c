/*
 * sensorboard.c
 *	Development Board for tool cabinet sensors
 *
 *
 * Protocoll:
 *		First Byte:
 *			Bit 0-6:	Adress, 0000000 is Broadcast, 1111111 is ResetAll
 *			Bit 7:		1 = write, 0 = write
 *
 * Created: 27.02.2015 00:36:00
 *  Author: Olli
 */ 

/*
 * DEFINES
 */

#define IR_OUT0 PC1
#define IR_IN0 PC0
#define IR_OUT1 PC3
#define IR_IN1 PC2
#define TASTER PB2
#define ADRESS 0x01

#define F_CPU 16000000UL
#define UART_BAUD_RATE 9600


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"

unsigned char new_frame;
unsigned int buffer; //buffer for char
void analog_init();

int main(void)
{
	new_frame = 0;
	DDRC |= (1 << IR_OUT1);
	DDRD |= ((1 << PD5) | (1 << PD6) | (1 << PD7));
	PORTB |= (1 << TASTER);
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU) );
	analog_init();
	sei();
    while(1)
    {
		if(!(uart_getc() & UART_NO_DATA))
		{
			uart_putc(frame_counter);
		}
		/* OLD ADC STUFF */
		/*
		if (!(PINB & (1 << TASTER)))
		{
			PORTD = 0xFF;
			PORTC |= (1 << IR_OUT1);
			ADCSRA |= (1 << ADSC); //start conversion
			while(ADCSRA & (1 << ADSC))
			{
				asm volatile("nop"); //wait for conversion to complete
			}
			uart_puts("\r\nADC: ");
			uart_putc(ADCH);
		} else
		{
			PORTD = 0x00;
			PORTC &= ~( (1 << IR_OUT1));
		}*/
    }
}

void analog_init()
{
		/* AVCC as Input Source, 100nF Cap between AREF and AVCC needed! */
	ADMUX |= (1 << REFS0);
	ADMUX &=  ~( 1 << REFS1);
	/* Left Adjust Result, LSB and 2.LSB in ADCL
	   Results are less exact but sufficient for our needs */
	ADMUX |= (1 << ADLAR);
	/* Select ADC7 as Analog Input */
	ADMUX |= (1 << MUX1);
	/* Enable ADC */
	ADCSRA |= (1 << ADEN);
	/* Perform one conversion, part of initialisation */
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC))
	{
		asm volatile("nop"); //wait for conversion to complete
	}
}