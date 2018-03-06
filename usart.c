#include "defines.h"
#include <avr/io.h>
#include <util/delay.h>

void USART_init(void)
{
  unsigned int bd = (F_CPU / (16UL * UART_BAUD)) - 1;
  UBRR0L = bd & 0xFF;
  UBRR0H = bd >> 8;

  UCSR0B = _BV(TXEN0) | _BV(RXEN0) | _BV(RXCIE0); /* tx/rx enable */
//  UCSRC = 1<<URSEL|1<<UCSZ0|1<<UCSZ1;
  UCSR0C |= /*_BV(UMSEL0) |*/ _BV(UCSZ01) | _BV(UCSZ00);
  //UCSRA = _BV(U2X);
}

void USART_TransmitByte( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void USART_TransmitHex( unsigned char data )
{
	unsigned char h = data>>4;
	char ho = (h < 10) ? (h+'0') : (h+'A'-10);
	unsigned char l = data & 0xF;
	char lo = (l < 10) ? (l+'0') : (l+'A'-10);
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = ho;
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = lo;
}

void USART_TransmitText(char* data)
{
	while (*data != 0)
	{
		/* Wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) );
		/* Put data into buffer, sends the data */
		UDR0 = *data;
		data++;
	}
}

void USART_Transmit(void* p, unsigned long int len)
{
	unsigned char* buff = (unsigned char*)p;
	unsigned long int b;
	for (b = 0; b < len; b++) USART_TransmitByte(buff[b]);
}
