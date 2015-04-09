/*
 * sensorboard.c
 *	Development Board for tool cabinet sensors
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
#define JUMPER PB1
#define ADDRESS 0x34

#define F_CPU 16000000UL
#define UART_BAUD_RATE 9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "timer.h"
#include "dbus.h"
#include "uart.h"
#include "analog.h"



//TODO: Quellcode aufräumen
int main(void)
{
	DDRC |= (1 << PC1);
	PORTB |= (1 << PB2);
	DDRD |= (1 << PD5);
	TIMER_Init();
	dbus_init();
	ANALOG_Init();
	sei();
	while(1)
	{
		dbus_receive();
		if(timerStatus & TIMER_FLAG_MS1)
		{
			timerStatus &= ~TIMER_FLAG_MS1;
			dbus_perform();
		}
		if(timerStatus & TIMER_FLAG_MS100)
		{
			timerStatus &= ~TIMER_FLAG_MS100;
			ANALOG_Service();
		}
		if(timerStatus & TIMER_FLAG_MS1000)
		{
			timerStatus &= ~TIMER_FLAG_MS1000;
//			unsigned int values = ANALOG_GetValues();
//			char* test = (char*) &values;
//			uart_putc(test[0]);
//			uart_putc(test[1]);
		}
	}
}


