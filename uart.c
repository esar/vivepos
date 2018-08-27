#include <stdio.h>
#include <avr/io.h>


int uartPutChar(char c, FILE* stream)
{
	while((USART0.STATUS & (1 << USART_DREIF_bp)) == 0)
		;
	USART0.TXDATAL = c;

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

