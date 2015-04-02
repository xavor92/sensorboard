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



//TODO: Quellcode aufräumen
//TODO: analog messzeit ca 100us bis stabiles signal, also 200us draus machen
int main(void)
{
	DDRC |= (1 << PC1);
	PORTB |= (1 << PB2);
	TIMER_Init();
	dbus_init();
	sei();
	while(1)
	{
		dbus_receive();
		if(timerStatus & TIMER_FLAG_MS1)
		{
			if(!(PINB & (1 << PB2)))
			{
				PORTC |= (1 << PC1);
			} else {
				PORTC &= ~(1 << PC1);
			}
			timerStatus &= ~TIMER_FLAG_MS1;
			dbus_perform();
		}
		if(timerStatus & TIMER_FLAG_MS100)
		{
			timerStatus &= ~TIMER_FLAG_MS100;
		}
		if(timerStatus & TIMER_FLAG_MS1000)
		{
			timerStatus &= ~TIMER_FLAG_MS1000;
		}
	}
	/*DDRD |= ((1 << PD5) | (1 << PD6) | (1 << PD7));
	PORTB |= (1 << TASTER) | (1 << JUMPER);
	//set timer2 to clk/1024 for periodic updates
	TCCR2 = 0b00000111; //normal mode, clk = clkIO/1024
	TIMSK |= (1 << TOV2); //enable interrupt
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU) );
	analog_init();
	sei();
	_delay_ms(5); //wait to recognize a start frame for sure
	if(frame_counter == 0) frame_status = 0;
    while(1)
    {
		receive();
		
		//check crc checksums
		if(frame_status == 99)
		{
			crc_reg = 0xFFFF;
			process_crc(buffer_address);
			process_crc(buffer_port);
			process_crc(buffer_data);
			if( (crc_reg >> 8 == buffer_crch) && ((crc_reg & 0xFF) == buffer_crcl) )
				frame_status = 100;
		}
		
		// Execution of input
		
		if (frame_status == 100)
		{
			perform();
			frame_status = 0;
		}
		
		//Task List Tasks
		if(task_list & 0x01) update_lower();
		if(task_list & 0x02) update_higher();
		
		
		//clear buffer and reset
		if(frame_status != 0 && frame_counter == 0)
		{
			frame_status = 0;
			while(!(uart_getc() && UART_NO_DATA)) //clear receive buffer
			{
				asm volatile("nop"); //wait for conversion to complete
			}
		}
    }*/
}

/*
//receive data in buffer
void receive()
{
	//
	//	Get Address
	//
	if(frame_counter > 0 && frame_status == 0)	//we received sth and are waiting for an address
	{
		buffer_address = uart_getc();
		if(buffer_address == ADDRESS || buffer_address == 0)	//if addressed to slave or broadcast
		{
			frame_status = 1;
		} else 
		{
			frame_status = 101;
		}
	}
		

	 //	Get Port

	if (frame_counter > 1 && frame_status == 1) //were addressed and a new port/command is available
	{
		buffer_port = uart_getc();
		frame_status = 2;
	}
		
	//Get Data
	if (frame_counter > 2 && frame_status == 2)
	{
		buffer_data = uart_getc();
		frame_status = 3;
	}
		
	if (frame_counter > 4 && frame_status == 3) //get and save crc
	{
		buffer_crch = uart_getc();
		buffer_crcl = uart_getc();
		frame_status = 99;
	}
}
*/

/*void perform()
{
	if ( buffer_port & 0b10000000 ) //1 in bit7 -> write
	{
		if ( (buffer_port & 0b0111111) == 0x01) //adding tasks to task list
		{
			task_list |= (buffer_data);
			send(ADDRESS, buffer_port, task_list);
		}
	} else							//read
	{
		if( (buffer_port & 0b0111111) == 0x01) //read task list
		{
			send(ADDRESS, buffer_port, task_list);
		}
		
		if( (buffer_port & 0b0111111) == 0x10) //read lower sensor value
		{
			send(ADDRESS, buffer_port, lower_sensor);
		}
		
		if( (buffer_port & 0b0111111) == 0x11) //read higher sensor value
		{
			send(ADDRESS, buffer_port, higher_sensor);
		}
	}	
}

//analog init script
void analog_init()
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

void send(unsigned char address, unsigned char port, unsigned char data)
{
	crc_reg = 0xFFFF;
	process_crc(address);
	process_crc(port);
	process_crc(data);
	uart_putc(address);
	uart_putc(port);
	uart_putc(data);
	uart_putc(crc_reg >> 8);
	uart_putc(crc_reg & 0xFF);
}


//update lower sensor
void update_lower()
{
	task_list &= ~0x01;
	PORTC |= (1 << IR_OUT0);												// enable IR LED
	_delay_us(50);
	ADMUX &= ~( (1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3)); // set correct MUX
	ADCSRA |= (1 << ADSC); //start conversion
	while(ADCSRA & (1 << ADSC))
	{
		asm volatile("nop"); //wait for conversion to complete
	}
	PORTC &= ~(1 << IR_OUT0);
	lower_sensor = ADCH;
}
*/

/*
//update higher sensor
void update_higher()
{
	task_list &= ~0x02;
	PORTC |= (1 << IR_OUT1);												// enable IR LED
	_delay_us(50);
	ADMUX &= ~( (1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3)); // set correct MUX
	ADMUX |= (1 << MUX1);
	ADCSRA |= (1 << ADSC); //start conversion
	while(ADCSRA & (1 << ADSC))
	{
		asm volatile("nop"); //wait for conversion to complete
	}
	PORTC &= ~(1 << IR_OUT1);
	higher_sensor = ADCH;
}
*/


