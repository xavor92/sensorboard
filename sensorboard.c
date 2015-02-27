/*
 * sensorboard.c
 *	Development Board for tool cabinet sensors
 *
 * Created: 27.02.2015 00:36:00
 *  Author: Olli
 */ 


#include <avr/io.h>

int main(void)
{
	DDRC |= (1 << PC1);
	DDRD |= ((1 << PD5) | (1 << PD6) | (1 << PD7));
	PORTB |= (1 << PB2);
    while(1)
    {
		if (!(PINB & (1 << PB2)))
		{
			PORTD = 0xFF;
		} else
		{
			PORTD = 0x00;
		}
        //TODO:: Please write your application code 
    }
}