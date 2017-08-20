#include "defines.h"
#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include "jtag.h"
#include "jtag_commands.h"

static uint8_t jtag_current_command = 0;
static uint16_t jtag_pos = 0;
static uint32_t jtag_tck_delay_value = 0;
static uint32_t jtag_num_tck = 0;
static uint32_t jtag_usecs = 0;
static uint16_t jtag_multi_count = 0;
static uint16_t jtag_multi_pos = 0;
static uint16_t jtag_multi_multi = 0;

static void tck_delay(uint8_t tms, uint32_t num_tck, uint32_t usecs)
{
	if (tms) 
		PORT |= (1<<TMS_PIN);
	else
		PORT &= ~(1<<TMS_PIN);
	PORT |= 1<<TCK_PIN;
	int i;
	for (i = 0; i < num_tck; i++)
	{
		PORT ^= 1<<TCK_PIN;
		//_delay_us(1);
		PORT ^= 1<<TCK_PIN;
		//_delay_us(1);
	}
	for (i = 0; i < usecs; i++)
		_delay_us(1);
}

static uint8_t pulse_tck(int tms, int tdi, int tdo)
{
	if (tms)
		PORT |= (1<<TMS_PIN);
	else
		PORT &= ~(1<<TMS_PIN);
	if (tdi >= 0)
	{
		if (tdi)
			PORT |= (1<<TDI_PIN);
		else
			PORT &= ~(1<<TDI_PIN);
	}
	PORT &= ~(1<<TCK_PIN);
	//_delay_us(1);
	PORT |= 1<<TCK_PIN;
	//_delay_us(1);
	if (tdo < 0) return 1;
	return ((PORT_PIN >> TDO_PIN) & 1) == tdo;
}

static int jtag_parse_byte(uint8_t data)
{
	int i, tms, tdi, tdo;
	switch (jtag_current_command)
	{
		case 0:
			jtag_current_command = data;
			jtag_pos = 0;
			break;
			
		case JTAG_PULSE_TCK_DELAY:
			if (jtag_pos == 0)
			{
				jtag_num_tck = 0;
				jtag_usecs = 0;
				jtag_tck_delay_value = data;
			} else {
				if ((jtag_pos < 4) && (jtag_tck_delay_value & 0b10)) // num_tck
				{
					jtag_num_tck |= (uint32_t)data << (8*(jtag_pos-1));
				} else if ((jtag_pos < 4) && !(jtag_tck_delay_value & 0b10)) // usecs
				{
					jtag_usecs |= (uint32_t)data << (8*(jtag_pos-1));
				} else {
					jtag_usecs |= (uint32_t)data << (8*(jtag_pos-5));
				}
			}
			if ( ((jtag_tck_delay_value & 0b110) == 0)
				|| (((((jtag_tck_delay_value & 0b110) == 0b100) || (jtag_tck_delay_value & 0b110) == 0b010)) && jtag_pos == 4)
				|| (((jtag_tck_delay_value & 0b110) == 0b110) && jtag_pos == 8)
			)
			{
				tck_delay(jtag_tck_delay_value&1, jtag_num_tck, jtag_usecs);
				jtag_current_command = 0;			
			}
			jtag_pos++;			
			break;
			
		case JTAG_PULSE_TCK_MULTI:
			if (jtag_pos == 0)
			{
				jtag_multi_count = data;
				jtag_multi_pos = 0;
				jtag_multi_multi = 0;
			} else if (jtag_pos == 1)
			{
				jtag_multi_count |= (uint16_t)data << 8;
			} else {
				if (!jtag_multi_multi && !(data & 0x80))
				{
					jtag_multi_multi = data;
				}	else {
					if (!jtag_multi_multi) jtag_multi_multi = 1;
					tms = data&1;
					tdi = (data>>1)&1;
					tdo = -1;
					if (data & (1<<2))
					{
						tdo = (data>>3)&1;
					}
					for (i = 0; i < jtag_multi_multi; i++)
					{
						if (!pulse_tck(tms, tdi, tdo)) return 0;
						jtag_multi_pos++;
					}
					jtag_multi_multi = 0;
				}
				if (jtag_multi_pos >= jtag_multi_count)
					jtag_current_command = 0;
			}
			jtag_pos++;
			break;
			
		/*
		case JTAG_PULSE_TCK:
			tms = data&1;
			tdi = (data>>1)&1;
			tdo = -1;
			if (data & (1<<2))
			{
				tdo = (data>>3)&1;
			}
			if (!pulse_tck(tms, tdi, tdo)) return 0;
			jtag_current_command = 0;
			break;
		*/
		default:
			return 0;
	}
	return 1;
}

void jtag_setup()
{
	PORT &= ~((1<<TMS_PIN) | (1<<TDO_PIN) | (1<<TDI_PIN)) | (1<<TCK_PIN);
	PORT_DDR |= (1<<TMS_PIN) | (1<<TCK_PIN) | (1<<TDI_PIN);
	PORT_DDR &= ~(1<<TDO_PIN);
	jtag_current_command = 0;
}

void jtag_shutdown()
{
	PORT &= ~((1<<TMS_PIN) | (1<<TCK_PIN) | (1<<TDO_PIN) | (1<<TDI_PIN));
	PORT_DDR &= ~((1<<TMS_PIN) | (1<<TCK_PIN) | (1<<TDO_PIN) | (1<<TDI_PIN));
}

int jtag_execute(int count, uint8_t* data)
{
	int i;
	for (i = 0; i < count; i++)
	{
		if (!jtag_parse_byte(data[i]))
		{
			jtag_shutdown();
			return 0;
		}
	}
	return 1;
}
