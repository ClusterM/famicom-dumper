#include "defines.h"
#include <inttypes.h>
#include <util/delay.h>
#include "comm.h"
#include "usart.h"
#include "crc.h"

static uint8_t comm_send_crc;     // CRC of outgoing packet with header
static uint16_t comm_send_length; // size of outgoing data
static uint16_t comm_send_pos;    // how many data sent by app

volatile uint8_t recv_buffer[RECV_BUFFER_SIZE];
static uint16_t comm_recv_pos;    // how many bytes of packet received
static uint8_t comm_recv_crc;
static uint8_t comm_recv_error;
volatile uint8_t comm_recv_command;
volatile uint16_t comm_recv_length;
volatile uint8_t comm_recv_done;

static void comm_send_and_calc(uint8_t data)
{
	comm_send_crc = calc_crc8(comm_send_crc, data);
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
  if (comm_recv_error && data != 'F')
    return;
  comm_recv_error = 0;
  if (!comm_recv_pos)
  {
    comm_recv_crc = 0;
    comm_recv_done = 0;
  }
  comm_recv_crc = calc_crc8(comm_recv_crc, data);
  switch (comm_recv_pos)
  {
  case 0:
    {
      if (data != 'F')
      {
        comm_recv_error = 1;
        comm_start(COMMAND_ERROR_INVALID, 0);
      }
    }
    break;
  case 1:
    {
      comm_recv_command = data;
    }
    break;
  case 2:
    {
      comm_recv_length = data;
    }
    break;
  case 3:
    {
      comm_recv_length |= (uint16_t) data << 8;
    }
    break;
  default:
    {
      uint16_t pos = comm_recv_pos - 4;
      if (pos >= sizeof(recv_buffer))
      {
        comm_recv_pos = 0;
        comm_recv_error = 1;
        comm_start(COMMAND_ERROR_OVERFLOW, 0);
        return;
      } else if (pos < comm_recv_length)
      {
        recv_buffer[pos] = data;
      } else if (pos == comm_recv_length)
      {
        if (!comm_recv_crc)
        {
          comm_recv_done = 1;
        } else
        {
          comm_recv_error = 1;
          comm_start(COMMAND_ERROR_CRC, 0);
        }
        comm_recv_pos = 0;
        return;
      }
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
