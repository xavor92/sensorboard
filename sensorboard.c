/*
 * sensorboard.c
 *	Development Board for tool cabinet sensors
 *
 *
 * Protocoll:
 *		Byte 0: address of Target
 *		Byte 1: Port / Function (Bit 7 = 0 Read, Bit 7 = 1 Write, Bit 0-6 address)
 *		Byte 3+n: CheckSum, High Byte
 *		Byte 4+n: CheckSum, Low Byte
 *		
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
#define ADRESS 7

#define F_CPU 16000000UL
#define UART_BAUD_RATE 9600


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"

void analog_init();
void process_crc(unsigned char new_char);

unsigned char buffer_address, buffer_port, buffer_length;
unsigned char port;
unsigned int crc_reg; //CRC Register

/*
 *	frame_status is the actual mode of the slave, equivalent to the protocol,
  *		0:			Waiting for address
  *		1:			Got address, waiting for Read/write
  *		2:			Got address but not our address/global
  *		10:			Read stuff
  *		11:			Write stuff
 */
unsigned char frame_status; 



int main(void)
{
	frame_status = 99;
	DDRC |= (1 << IR_OUT1);
	DDRD |= ((1 << PD5) | (1 << PD6) | (1 << PD7));
	PORTB |= (1 << TASTER);
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU) );
	analog_init();
	sei();
	//_delay_ms(5); //wait to recognize a start frame
	if(frame_counter == 0) frame_status = 0;
    while(1)
    {
		if(frame_counter > 0 && frame_status == 0)	//we received sth and are waiting for an address
		{
			buffer_address = uart_getc();
			if(buffer_address == ADRESS || buffer_address == 0)	//if addressed to slave or broadcast
			{
				frame_status = 1;
			} else 
			{
				frame_status = 2;
			}
		}
		if (frame_counter > 1 && frame_status == 1) //were addressed and a new port/command is available
		{
			buffer_port = uart_getc();
			if(buffer_port & UART_NO_DATA) PORTD |= (1 << PD5);
			//check for Bit7  (Bit 7 = 0 Read, Bit 7 = 1 Write)
			if (buffer_port & 0b10000000)  //bit 7 set -> write
			{
				//save we want to write
				frame_status = 11;
			} else 
			{
				frame_status = 10;
				//save we want to read
			}
			port = buffer_port & 0b01111111;
		}
		
		if (!(PINB & (1 << TASTER)))
		{
			crc_reg = 0xFFFF;
			process_crc(0x34);
			process_crc(0x00);
			process_crc(0x01);
			process_crc(0xFF);
			process_crc(0x23);
			uart_putc(crc_reg >> 8);
			uart_putc(crc_reg & 0xFF);
			_delay_ms(1000);
		}
		if(frame_status != 0 && frame_counter == 0)
		{
			frame_status = 0;
			while(!(uart_getc() && UART_NO_DATA))
			{
				asm volatile("nop"); //wait for conversion to complete
			}
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

//Function from ftp://www.wickenhaeuser.de/anleitungen/rs485pro.pdf
void process_crc(unsigned char new_char)
{
	unsigned char uci, w;
	crc_reg ^= new_char;		// Einblendung im unteren Byte der CRC
	for(uci = 8; uci; uci--)	// 8 Schleifendurchgänge
	{
		w = crc_reg & 1;
		crc_reg >>=1;
		if(w) crc_reg^=0xA001;
	}
}