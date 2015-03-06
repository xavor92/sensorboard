/*
 * sensorboard.c
 *	Development Board for tool cabinet sensors
 *
 *
 * Protocoll:
 *		Byte 0: address of Target
 *		Byte 1: Port / Function (Bit 7 = 0 Read, Bit 7 = 1 Write, Bit 0-6 address)
 *		Byte 2: Data
 *		Byte 3: CheckSum, High Byte
 *		Byte 4: CheckSum, Low Byte
 *		
 *		CheckSum Generator Beta:
 *			http://www.tutorialspoint.com/compile_c_online.php?PID=0Bw_CjBb95KQMdVBCYTRTbW04ams
 *			May have some Bugs for ex when entering FF, Final output will
 *			be fucked up but checksum is correct.
 *
 *		ToDo/Markers as of 6.3.15
 *			Catch Broadcast Read
 *			Fix FF on Checksum Generator
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
#define JUMPER PB1
#define ADRESS 0x34

#define F_CPU 16000000UL
#define UART_BAUD_RATE 9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"

//receive
void receive();

//initialize ADC
void analog_init();

//process crc_reg with new_char
void process_crc(unsigned char new_char);

unsigned char buffer_address, buffer_port, buffer_data, buffer_crch, buffer_crcl;

unsigned int crc_reg; //CRC Register

/*
 *	frame_status is the actual mode of the slave, equivalent to the protocol,
  *		0:			Waiting for address
  *		1:			Got address, waiting for Read/write + port
  *		2:			Waiting for DataByte
  *		3:			Waiting for CRC16
  *		99:			Got everything, waiting for crc check approval
  *		100:		crc check correct, process data
  *		101:		not the slaves address
 */
unsigned char frame_status;

int main(void)
{
	DDRD |= ((1 << PD5) | (1 << PD6) | (1 << PD7));
	PORTB |= (1 << TASTER) | (1 << JUMPER);
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
		
		/*
		 * Execution of input */
		
		if (frame_status == 100)
		{
			if ( buffer_port & 0b10000000 ) //1 in bit7 -> write
			{
				if( (buffer_port & 0b0111111) == 0)
					PORTD = buffer_data;
			} else							//read
			{
				if( (buffer_port & 0b0111111) == 0)
				{
					buffer_data = PINB;
					crc_reg = 0xFFFF;
					process_crc(buffer_address);
					process_crc(buffer_port);
					process_crc(buffer_data);
					uart_putc(buffer_address);
					uart_putc(buffer_port);
					uart_putc(buffer_data);
					uart_putc(crc_reg >> 8);
					uart_putc(crc_reg & 0xFF);
				}
			}
			
			frame_status = 0;
		}
		
		if(frame_status != 0 && frame_counter == 0)
		{
			frame_status = 0;
			while(!(uart_getc() && UART_NO_DATA)) //clear receive buffer
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

void receive()
{
	/*
	 *	Get Address
	 */ 
	if(frame_counter > 0 && frame_status == 0)	//we received sth and are waiting for an address
	{
		buffer_address = uart_getc();
		if(buffer_address == ADRESS || buffer_address == 0)	//if addressed to slave or broadcast
		{
			frame_status = 1;
		} else 
		{
			frame_status = 101;
		}
	}
		
	/*
	 *	Get Port
	 */
	if (frame_counter > 1 && frame_status == 1) //were addressed and a new port/command is available
	{
		buffer_port = uart_getc();
		frame_status = 2;
	}
		
	/*
	 *	Get Data
 	 */
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

//analog init script
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

//process crc_reg with new_char
void process_crc(unsigned char new_char)
{
	//Function from ftp://www.wickenhaeuser.de/anleitungen/rs485pro.pdf
	unsigned char uci, w;
	crc_reg ^= new_char;		// Einblendung im unteren Byte der CRC
	for(uci = 8; uci; uci--)	// 8 Schleifendurchgänge
	{
		w = crc_reg & 1;
		crc_reg >>=1;
		if(w) crc_reg^=0xA001;
	}
}