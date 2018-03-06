#include "defines.h"
#include <inttypes.h>
#include <util/delay.h>
#include "comm.h"
#include "usart.h"

static uint8_t comm_send_crc;
static unsigned int comm_send_length;
static unsigned int comm_send_pos;

static int comm_recv_pos;
static uint8_t comm_recv_crc;
static uint8_t comm_recv_error;

volatile uint8_t comm_recv_command;
volatile unsigned int comm_recv_length;
volatile uint8_t recv_buffer[RECV_BUFFER];
volatile uint8_t comm_recv_done;

static void comm_calc_send_crc(uint8_t inbyte)
{
  uint8_t j;
  for (j=0;j<8;j++) 
  {
		uint8_t mix = (comm_send_crc ^ inbyte) & 0x01;
		comm_send_crc >>= 1;
		if (mix) 
			comm_send_crc ^= 0x8C;                  
		inbyte >>= 1;
  }
}

static void comm_calc_recv_crc(uint8_t inbyte)
{
  uint8_t j;
  for (j=0;j<8;j++) 
  {
		uint8_t mix = (comm_recv_crc ^ inbyte) & 0x01;
		comm_recv_crc >>= 1;
		if (mix) 
			comm_recv_crc ^= 0x8C;                  
		inbyte >>= 1;
  }
}

static void comm_send_and_calc(uint8_t data)
{
	comm_calc_send_crc(data);
	USART_TransmitByte(data);
#ifdef SEND_DELAY
	_delay_us(SEND_DELAY);
#endif
}

void comm_start(uint8_t command, unsigned int length)
{
	comm_send_crc = 0;
	comm_send_and_calc('F');
	comm_send_and_calc(command);
	comm_send_and_calc(length & 0xff);
	comm_send_and_calc((length >> 8) & 0xff);
	comm_send_length = length;
	comm_send_pos = 0;

	if (!comm_send_length)
		USART_TransmitByte(comm_send_crc);
}

void comm_send_byte(uint8_t data)
{
	comm_send_and_calc(data);
	comm_send_pos++;
	
	if (comm_send_pos == comm_send_length)
		USART_TransmitByte(comm_send_crc);
}

void comm_proceed(uint8_t data)
{
	if (comm_recv_error && data != 'F') return;
	comm_recv_error = 0;
	if (!comm_recv_pos)
	{
		comm_recv_crc = 0;
		comm_recv_done = 0;
	}
	comm_calc_recv_crc(data);
	unsigned int l = comm_recv_pos-4;
	switch (comm_recv_pos)
	{
		case 0:
			if (data != 'F')
			{
				comm_recv_error = 1;
				comm_start(COMMAND_ERROR_INVALID, 0);
			}
			break;
		case 1:
			comm_recv_command = data;
			break;
		case 2:
			comm_recv_length = data;
			break;
		case 3:
			comm_recv_length |= (uint16_t)data << 8;
			break;
		default:
			if (l >= RECV_BUFFER)
			{
				comm_recv_pos = 0;
				comm_recv_error = 1;
				comm_start(COMMAND_ERROR_OVERFLOW, 0);
				return;
			}			
			else if (l < comm_recv_length)
			{
				recv_buffer[l] = data;				
			} else if (l == comm_recv_length)
			{
				if (!comm_recv_crc)
				{
					comm_recv_done = 1;
				} 
				else
				{
					comm_recv_error = 1;
					comm_start(COMMAND_ERROR_CRC, 0);
				}
				comm_recv_pos = 0;
				return;
			}
			break;
	}
	comm_recv_pos++;
}

void comm_init()
{
	comm_recv_pos = 0;
	comm_recv_done = 0;	
	comm_recv_error = 0;
}
