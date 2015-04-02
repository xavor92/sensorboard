/*
 * dbus.c
 *
 *  Created on: 02.04.2015
 *      Author: olli
 */

/*
 *	Includes
 */

#include "dbus.h"
#include "uart.h"
#include <stdlib.h>
#include <avr/io.h>
#include <avr/eeprom.h>

/*
 *	Defines & Variables
 */

#define LED_MASK ((1 << PD5) | (1 << PD6) | (1 << PD7))

#define START_REQ 0x3F
#define START_RES 0x2A


static enum Status frameStatus;
static frame frameBuffer;
static unsigned char dbusAddress = DBUS_ADDRESS;


/*
 *	Local Functions (Declaration, static)
 */

static unsigned int calc_crc(frame *framePointer);
static unsigned char send_Frame(frame *framePointer);
static unsigned char fs_get_portd(frame *framePointer);
static unsigned char fs_set_portd(frame *framePointer);

/*
 *	Global Functions (Definitions)
 */

extern void dbus_init()
{
	uart_init(UART_BAUD_SELECT(DBUS_BAUD, F_CPU));
	DDRD |= LED_MASK;
}

extern void dbus_receive()
{
	unsigned int c = uart_getc();
	unsigned char* helper;
	if(!(c & UART_NO_DATA))				//if Data available
	{
		if( ( (c & 0xFF) == START_REQ) || ( (c & 0xFF) == START_RES) )
		{
			frameBuffer.start = (c & 0xFF);
			frameStatus = statusStart;
		} else
		{
			switch(frameStatus)
			{
			case statusStart:
				frameBuffer.address = (c & 0xFF);
				uart_puts("start");
				frameStatus = statusAddress;
				break;
			case statusAddress:
				frameBuffer.function = (c & 0xFF);
				uart_puts("address");
				frameStatus = statusFunction;
				break;
			case statusFunction:
				frameBuffer.data = (c & 0xFF);
				uart_puts("data");
				frameStatus = statusData;
				break;
			case statusData:
				helper = (unsigned char*)&frameBuffer.crc;
				uart_puts("crch");
				helper[1] = (c & 0xFF);
				frameStatus = statusCrcH;
				break;
			case statusCrcH:
				helper = (unsigned char*)&frameBuffer.crc;
				uart_puts("crcl");
				helper[0] = (c & 0xFF);
				frameStatus = statusReceiveComplete;
				break;
			default:
				uart_puts("frame error");
				break;
			}
		}
	}
}

extern enum Status dbus_perform()
{

	if(frameStatus == statusReceiveComplete)
	{
		uart_puts("perform");
		char string[10];
		itoa(calc_crc(&frameBuffer), &string, 16);
		uart_puts(&string);
		if(frameBuffer.crc == calc_crc(&frameBuffer))
		{
			frameStatus = statusCrcCorrect;
			uart_puts("crc ok");
			switch(frameBuffer.function)
			{
				case FUNCTION_GET_PORTD:
					fs_get_portd(&frameBuffer);
					send_Frame(&frameBuffer);
					break;
				case FUNCTION_SET_PORTD:
					fs_set_portd(&frameBuffer);
					send_Frame(&frameBuffer);
					break;
				default:
					frameStatus = statusFunctionUnknown;
					send_Frame(&frameBuffer);
					return frameStatus;
					break;
			}
			frameStatus = statusFrameProcessed;
			return frameStatus;
		} else
		{
			frameStatus = statusCrcWrong;
			return frameStatus;
		}
	}
	return frameStatus;
}

/*
 *	Local Functions (Definitions)
 */

static unsigned int calc_crc(frame *framePointer)
{
	unsigned int crc_reg = 0xFFFF;	//init crc

	//calculate crc of complete frame

	crc_reg ^= framePointer->start;		// Einblendung im unteren Byte der CRC
	for(unsigned char uci = 8; uci; uci--)	// 8 Schleifendurchg�nge
	{
		unsigned char w = crc_reg & 1;
		crc_reg >>=1;
		if(w) crc_reg^=0xA001;
	}

	crc_reg ^= framePointer->address;		// Einblendung im unteren Byte der CRC
	for(unsigned char uci = 8; uci; uci--)	// 8 Schleifendurchg�nge
	{
		unsigned char w = crc_reg & 1;
		crc_reg >>=1;
		if(w) crc_reg^=0xA001;
	}

	crc_reg ^= framePointer->function;		// Einblendung im unteren Byte der CRC
	for(unsigned char uci = 8; uci; uci--)	// 8 Schleifendurchg�nge
	{
		unsigned char w = crc_reg & 1;
		crc_reg >>=1;
		if(w) crc_reg^=0xA001;
	}

	crc_reg ^= framePointer->data;		// Einblendung im unteren Byte der CRC
	for(unsigned char uci = 8; uci; uci--)	// 8 Schleifendurchg�nge
	{
		unsigned char w = crc_reg & 1;
		crc_reg >>=1;
		if(w) crc_reg^=0xA001;
	}

	return crc_reg;
}

static unsigned char send_Frame(frame *framePointer)
{
	if(framePointer->address == dbusAddress)
	{
		unsigned char *helper;
		helper = (unsigned char *)&framePointer->crc;
		framePointer->crc = calc_crc(framePointer);
		uart_putc(framePointer->start);
		uart_putc(framePointer->address);
		uart_putc(framePointer->function);
		uart_putc(framePointer->data);
		uart_putc(helper[1]);
		uart_putc(helper[0]);
	}
	return 0x00;
}

static unsigned char fs_get_portd(frame *framePointer)
{
	framePointer->start = START_RES;
	framePointer->data = PORTD;
	return 0x00;
}

static unsigned char fs_set_portd(frame *framePointer)
{
	PORTD = framePointer->data & LED_MASK;
	framePointer->start = START_RES;
	framePointer->data = 0x00;
	return 0x00;
}
