/*

   Copyright (C) 2018 Stephen Robinson
  
   This file is part of vivepos
  
   vivepos is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 2 of the License, or
   (at your option) any later version.
  
   vivepos is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this code (see the file names COPING).  
   If not, see <http://www.gnu.org/licenses/>.
  
*/

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// TX_BUF_SIZE must be power of 2
#define TX_BUF_SIZE    1024

typedef uint16_t index_t;
static uint8_t g_txBuf[TX_BUF_SIZE];
static index_t g_txReadPos = 0;
static index_t g_txWritePos = 0;

ISR(USART0_DRE_vect)
{
	if(g_txReadPos != g_txWritePos)
	{
		// buffer is not empty, transmit next byte
		USART0.TXDATAL = g_txBuf[g_txReadPos];
		g_txReadPos = (g_txReadPos + 1) & (TX_BUF_SIZE - 1);
	}
	else
	{
		// buffer is empty, turn off interrupt
		USART0.CTRLA &= ~(1 << USART_DREIE_bp);
	}
}

int uartPutChar(char c, FILE* stream)
{
	index_t pos = (g_txWritePos + 1) & (TX_BUF_SIZE - 1);

	// discard is tx buffer is full
	if(pos == g_txReadPos)
		return 0;

	// add data to buffer and enable interrupt to start sending
	g_txBuf[g_txWritePos] = c;
	g_txWritePos = pos;
	USART0.CTRLA |= 1 << USART_DREIE_bp;

	return 0;
}

int uartGetChar(FILE* stream)
{
	return -1;
}

static FILE uartOutput = FDEV_SETUP_STREAM(uartPutChar, NULL, 
                                           _FDEV_SETUP_WRITE);
static FILE uartInput  = FDEV_SETUP_STREAM(NULL, uartGetChar, 
                                           _FDEV_SETUP_READ);

void uartInit()
{
	// 115200
	USART0.BAUDH = 0x02;
	USART0.BAUDL = 0xB6;

	USART0.CTRLB |= (1 << USART_RXEN_bp) | (1 << USART_TXEN_bp);

	stdout = &uartOutput;
	stdin  = &uartInput;
}

