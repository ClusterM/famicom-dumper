/* Famicom Dumper/Programmer
 *
 * Copyright notice for this file:
 *  Copyright (C) 2016 Cluster
 *  http://clusterrr.com
 *  clusterrr@clusterrr.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "defines.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <inttypes.h>
#include "usart.h"
#include "comm.h"
#include "jtag.h"

#define LED_RED_ON PORTB |= (1<<7)
#define LED_RED_OFF PORTB &= ~(1<<7)
#define LED_GREEN_ON PORTB |= (1<<6)
#define LED_GREEN_OFF PORTB &= ~(1<<6)
#define ROMSEL_HI PORTF |= (1<<1)
#define ROMSEL_LOW PORTF &= ~(1<<1)
#define PHI2_HI PORTF |= (1<<0)
#define PHI2_LOW PORTF &= ~(1<<0)
#define MODE_READ { PORTD = 0xFF; DDRD = 0; }
#define MODE_WRITE DDRD = 0xFF
#define PRG_READ PORTF |= (1<<7)
#define PRG_WRITE PORTF &= ~(1<<7)
#define CHR_READ_HI PORTF |= (1<<5)
#define CHR_READ_LOW PORTF &= ~(1<<5)
#define CHR_WRITE_HI PORTF |= (1<<2)
#define CHR_WRITE_LOW PORTF &= ~(1<<2)

static void (*jump_to_bootloader)(void) = (void*)0xF800;

ISR(USART0_RX_vect)
{
	unsigned char b;
	while (UCSR0A & (1<<RXC0))
	{
		b = UDR0;
		comm_proceed(b);
	}
}

static void phi2_init()
{
	int i = 0x80;
	unsigned char h = PORTF |= (1<<0);
	unsigned char l = PORTF &= ~(1<<0);
	while(i != 0){
		PORTA = l;
		PORTA = h;
		i--;
	}
}

static void set_address(unsigned int address)
{
	unsigned char l = address & 0xFF;
	unsigned char h = address>>8;
	
	PORTA = l;
	PORTC = h;
	
	// PPU /A13
	if ((address >> 13) & 1)
		PORTF &= ~(1<<4);
	else
		PORTF |= 1<<4;
}

static void set_romsel(unsigned int address)
{
	if (address & 0x8000)
	{
		ROMSEL_LOW;
	} else {
		ROMSEL_HI;
	}
}

static unsigned char read_prg_byte(unsigned int address)
{
	MODE_READ;
	PRG_READ;	
	set_address(address);
	PHI2_HI;
	set_romsel(address);
	_delay_us(1);
	return PIND;
}

static unsigned char read_chr_byte(unsigned int address)
{
	MODE_READ;
	PHI2_HI;
	ROMSEL_HI;
	set_address(address);
	CHR_READ_LOW;
	
	_delay_us(1);
	
	uint8_t result = PIND;
	
	CHR_READ_HI;
	return result;
}

static unsigned char read_coolboy_byte(unsigned int address)
{
	MODE_READ;
	PRG_READ;	
	set_address(address);
	PHI2_HI;
	ROMSEL_LOW;
	PORTB |= 1<<TDO_PIN;
	PORTB &= ~(1<<TCK_PIN);
	_delay_us(1);
	return PIND;
}

static void read_prg_send(unsigned int address, unsigned int len)
{
	LED_GREEN_ON;
	comm_start(COMMAND_PRG_READ_RESULT, len);
	while (len > 0)
	{
		comm_send_byte(read_prg_byte(address));
		len--;
		address++;
	}
	set_address(0);
	PHI2_HI;
	ROMSEL_HI;
	LED_GREEN_OFF;
}

static void read_chr_send(unsigned int address, unsigned int len)
{
	LED_GREEN_ON;
	comm_start(COMMAND_CHR_READ_RESULT, len);
	while (len > 0)
	{
		comm_send_byte(read_chr_byte(address));
		len--;
		address++;
	}
	set_address(0);
	PHI2_HI;
	ROMSEL_HI;
	LED_GREEN_OFF;
}

static uint16_t crc16_update(uint16_t crc, uint8_t a)
{
	int i;
	crc ^= a;
	for (i = 0; i < 8; ++i)
	{
		if (crc & 1)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = (crc >> 1);
	}
	return crc;
}

static void read_prg_crc_send(unsigned int address, unsigned int len)
{
	LED_GREEN_ON;
	uint16_t crc = 0;
	read_prg_byte(address);
	while (len > 0)
	{
		unsigned char l = address & 0xFF;
		unsigned char h = address>>8;	
		PORTA = l;
		PORTC = h;
		_delay_us(1);
		crc = crc16_update(crc, PIND);
		len--;
		address++;
	}
	set_address(0);
	PHI2_HI;
	ROMSEL_HI;
	comm_start(COMMAND_PRG_READ_RESULT, 2);
	comm_send_byte(crc & 0xFF);
	comm_send_byte((crc >> 8) & 0xFF);
	LED_GREEN_OFF;
}

static void read_chr_crc_send(unsigned int address, unsigned int len)
{
	LED_GREEN_ON;
	uint16_t crc = 0;
	while (len > 0)
	{
		crc = crc16_update(crc, read_chr_byte(address));
		len--;
		address++;
	}
	set_address(0);
	PHI2_HI;
	ROMSEL_HI;
	comm_start(COMMAND_CHR_READ_RESULT, 2);
	comm_send_byte(crc & 0xFF);
	comm_send_byte((crc >> 8) & 0xFF);
	LED_GREEN_OFF;
}

static void read_coolboy_send(unsigned int address, unsigned int len)
{
	LED_GREEN_ON;
	PORTB |= 1<<TCK_PIN;
	PORTB |= 1<<TDO_PIN;
	DDRB |= 1<<TCK_PIN;
	DDRB |= 1<<TDO_PIN;
	comm_start(COMMAND_PRG_READ_RESULT, len);
	while (len > 0)
	{
		comm_send_byte(read_coolboy_byte(address));
		len--;
		address++;
	}
	set_address(0);
	ROMSEL_HI;
	PORTB |= 1<<TCK_PIN;
	PORTB |= 1<<TDO_PIN;
	
	jtag_shutdown();
	LED_GREEN_OFF;
}

static void write_prg_byte(unsigned int address, uint8_t data)
{
	PHI2_LOW;
	ROMSEL_HI;
	MODE_WRITE;
	PRG_WRITE;
	PORTD = data;
	set_address(address); // PHI2 low, ROMSEL always HIGH
//	_delay_us(1);
	
	PHI2_HI;
	//_delay_us(10);
	set_romsel(address); // ROMSEL is low if need, PHI2 high	
	
	_delay_us(1); // WRITING
	//_delay_ms(1); // WRITING
	
	// PHI2 low, ROMSEL high
	PHI2_LOW;
	_delay_us(1);
	ROMSEL_HI;
	
	// Back to read mode
//	_delay_us(1);
	PRG_READ;
	MODE_READ;
	set_address(0);

	// Set phi2 to high state to keep cartridge unreseted
//	_delay_us(1);
	PHI2_HI;

//	_delay_us(1);
}

static void write_chr_byte(unsigned int address, uint8_t data)
{
	PHI2_LOW;
	ROMSEL_HI;
	MODE_WRITE;
	PORTD = data;	
	set_address(address); // PHI2 low, ROMSEL always HIGH
	//_delay_us(10);
	
	CHR_WRITE_LOW;
		
	_delay_us(1); // WRITING
	//_delay_ms(1); // WRITING
	
	CHR_WRITE_HI;
	
	//_delay_us(1);
	
	MODE_READ;
	set_address(0);
	PHI2_HI;
	
	//_delay_us(1);
}

static void write_prg(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	while (len > 0)
	{
		write_prg_byte(address, *data);
		address++;
		len--;
		data++;
	}
	//_delay_ms(1);
	LED_RED_OFF;
}

static void write_chr(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	while (len > 0)
	{
		write_chr_byte(address, *data);
		address++;
		len--;
		data++;
	}
	//_delay_ms(1);
	LED_RED_OFF;
}

static void write_prg_flash_command(unsigned int address, uint8_t data)
{
	write_prg_byte(address | 0x8000, data);
}

static void write_coolboy_flash_command(unsigned int address, uint8_t data)
{
	PORTB |= 1<<TCK_PIN;
	PORTB |= 1<<TDO_PIN;
	ROMSEL_HI;
	PRG_READ;	
	set_address(address);
	MODE_WRITE;
	PORTD = data;
	PHI2_HI;
	ROMSEL_LOW;
	_delay_us(1);
	
	PORTB &= ~(1<<TDO_PIN);
	
	_delay_us(1);

	PORTB |= 1<<TDO_PIN;
	set_address(0);
	ROMSEL_HI;
	MODE_READ;
}

static void write_chr_flash_command(unsigned int address, uint8_t data)
{
	write_chr_byte(address, data);
}

static int write_prg_flash_byte(unsigned int address, uint8_t data)
{
	write_prg_flash_command(0x0000, 0xF0);
	write_prg_flash_command(0x0555, 0xAA);
	write_prg_flash_command(0x02AA, 0x55);
	write_prg_flash_command(0x0555, 0xA0);
	
	write_prg_flash_command(address, data);
	_delay_us(50);

	int timeout = 0;
	uint8_t res, last_res = 0;
	while (timeout < 200)
	{
		res = read_prg_byte(address | 0x8000);
		ROMSEL_HI;
		if (res == last_res && last_res == data) break;
		last_res = res;
 		_delay_us(50);
		timeout++;
	}
	//PHI2_LOW;
	ROMSEL_HI;
	set_address(0);
	
	return timeout < 10;
}


static int write_chr_flash_byte(unsigned int address, uint8_t data)
{
	write_chr_flash_command(0x0000, 0xF0);
	write_chr_flash_command(0x0555, 0xAA);
	write_chr_flash_command(0x02AA, 0x55);
	write_chr_flash_command(0x0555, 0xA0);
	write_chr_flash_command(address, data);

	int timeout = 0;
	while (read_chr_byte(address | 0x8000) != data && timeout < 10)
	{
		_delay_us(100);
		timeout++;
	}
	set_address(0);
	PHI2_LOW;
	ROMSEL_HI;
	
	return timeout < 10;
}

static int erase_prg_flash()
{
	LED_RED_ON;
	write_prg_flash_command(0x0000, 0xF0);
	write_prg_flash_command(0x0AAA, 0xAA);
	write_prg_flash_command(0x0555, 0x55);
	write_prg_flash_command(0x0AAA, 0x80);
	write_prg_flash_command(0x0AAA, 0xAA);
	write_prg_flash_command(0x0555, 0x55);
	write_prg_flash_command(0x0AAA, 0x10);
	
	int timeout = 0;
	while ((read_prg_byte(0x8000) != 0xFF) && (timeout < 300000))
	{
		_delay_ms(1);
		timeout++;
	}
	set_address(0);
	PHI2_HI;
	ROMSEL_HI;

	LED_RED_OFF;
	return timeout < 300000;
}

static int erase_coolboy_sector()
{
	LED_RED_ON;
	PORTB |= 1<<TCK_PIN;
	PORTB |= 1<<TDO_PIN;
	DDRB |= 1<<TCK_PIN;
	DDRB |= 1<<TDO_PIN;
	ROMSEL_HI;

	write_coolboy_flash_command(0x0000, 0xF0);
	write_coolboy_flash_command(0x0AAA, 0xAA);
	write_coolboy_flash_command(0x0555, 0x55);
	write_coolboy_flash_command(0x0AAA, 0x80);
	write_coolboy_flash_command(0x0AAA, 0xAA);
	write_coolboy_flash_command(0x0555, 0x55);
	write_coolboy_flash_command(0x0000, 0x30);	
	
	int timeout = 0;
	uint8_t debug;
	while (((debug = read_coolboy_byte(0x8000)) != 0xFF) && (timeout < 3000))
	{
		//comm_start(0xFF, 1);
		//comm_send_byte(debug);
		_delay_ms(1);
		timeout++;
	}
	
	set_address(0);
	ROMSEL_HI;

	jtag_shutdown();
	LED_RED_OFF;
	return timeout < 3000;
}

static int erase_coolgirl_sector()
{
	LED_RED_ON;
	write_prg_flash_command(0x0000, 0xF0);
	write_prg_flash_command(0x0AAA, 0xAA);
	write_prg_flash_command(0x0555, 0x55);
	write_prg_flash_command(0x0AAA, 0x80);
	write_prg_flash_command(0x0AAA, 0xAA);
	write_prg_flash_command(0x0555, 0x55);
	write_prg_flash_command(0x0000, 0x30);
	
	int timeout = 0;
	uint8_t debug;
	while (((debug = read_prg_byte(0x8000)) != 0xFF) && (timeout < 3000))
	{
		ROMSEL_HI;
		//comm_start(0xFF, 1);
		//comm_send_byte(debug);
		_delay_ms(1);
		timeout++;
	}	
	//comm_start(0xFF, 1);
	//comm_send_byte(debug);
	set_address(0);
	PHI2_HI;
	ROMSEL_HI;

	LED_RED_OFF;
	return timeout < 3000;
}

static int erase_chr_flash()
{
	LED_RED_ON;
	write_chr_flash_command(0x0000, 0xF0);
	write_chr_flash_command(0x0555, 0xAA);
	write_chr_flash_command(0x02AA, 0x55);
	write_chr_flash_command(0x0555, 0x80);
	write_chr_flash_command(0x0555, 0xAA);
	write_chr_flash_command(0x02AA, 0x55);
	write_chr_flash_command(0x0555, 0x10);
	
	int timeout = 0;
	while ((read_chr_byte(0) != 0xFF) && (timeout < 10000))
	{
		_delay_ms(1);
		timeout++;
	}
	set_address(0);
	PHI2_HI;
	ROMSEL_HI;

	LED_RED_OFF;
	return timeout < 10000;
}

static int write_prg_flash(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	int ok = 1;
	while (len > 0)
	{
		if (!write_prg_flash_byte(address, *data))
		{
			ok = 0;
			break;
		}
		address++;
		len--;
		data++;
	}
	LED_RED_OFF;
	return ok;
}

static int write_coolboy(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	PORTB |= 1<<TCK_PIN;
	PORTB |= 1<<TDO_PIN;
	DDRB |= 1<<TCK_PIN;
	DDRB |= 1<<TDO_PIN;
	ROMSEL_HI;
	uint8_t ok = 1;
	while (len > 0)
	{
		
		//uint8_t count = len > 16 ? 16 : len;
		uint8_t count = 0;
		uint8_t* d = data;
		unsigned int a = address;
		unsigned int address_base = a & 0xFFE0;
		while (len > 0 && ((a & 0xFFE0) == address_base))
		{
			if (*d != 0xFF) count++;
			a++;
			len--;
			d++;
		}

		if (count)
		{
			//write_prg_flash_command(0x0000, 0xF0);
			write_coolboy_flash_command(0x0AAA, 0xAA);
			write_coolboy_flash_command(0x0555, 0x55);
			write_coolboy_flash_command(0x0000, 0x25);
			write_coolboy_flash_command(0x0000, count-1);

			while (count > 0)
			{
				if (*data != 0xFF)
				{
					write_coolboy_flash_command(address, *data);
					count--;
				}
				address++;
				data++;
			}
		
			write_coolboy_flash_command(0x0000, 0x29);
			_delay_us(10);

			long int timeout = 0;
			uint8_t res, last_res = 0;
			while (timeout < 100000)
			{
				res = read_coolboy_byte((address-1) | 0x8000);
				ROMSEL_HI;
				if (res == last_res && last_res == *(data-1)) break;
				last_res = res;
				_delay_us(10);
				timeout++;
			}
			if (timeout >= 100000)
			{
				ok = 0;
				break;
			}
		}
		
		address = a;
		data = d;
	}
	ROMSEL_HI;
	jtag_shutdown();
	LED_RED_OFF;
	return ok;
}

static int write_coolgirl(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	ROMSEL_HI;
	uint8_t ok = 1;
	while (len > 0)
	{
		
		//uint8_t count = len > 16 ? 16 : len;
		uint8_t count = 0;
		uint8_t* d = data;
		unsigned int a = address;
		unsigned int address_base = a & 0xFFE0;
		while (len > 0 && ((a & 0xFFE0) == address_base))
		{
			if (*d != 0xFF)
				count++;
			a++;
			len--;
			d++;
		}

		if (count)
		{
			write_prg_flash_command(0x0000, 0xF0);
			write_prg_flash_command(0x0AAA, 0xAA);
			write_prg_flash_command(0x0555, 0x55);
			write_prg_flash_command(0x0000, 0x25);
			write_prg_flash_command(0x0000, count-1);

			while (count > 0)
			{
				if (*data != 0xFF)
				{
					write_prg_flash_command(address, *data);
					count--;
				}
				address++;
				data++;
			}
		
			write_prg_flash_command(0x0000, 0x29);
			_delay_us(10);

			long int timeout = 0;
			uint8_t res, last_res = 0;
			while (timeout < 100000)
			{
				res = read_prg_byte((address-1) | 0x8000);
				//comm_start(0xFF, 1);
				//comm_send_byte(res);
				ROMSEL_HI;
				if (res == last_res && last_res == *(data-1)) break;
				last_res = res;
				_delay_us(10);
				timeout++;
			}
			//comm_start(0xFF, 1);
			//comm_send_byte(res);
			if (timeout >= 100000)
			{
				ok = 0;
				break;
			}
		}

		address = a;
		data = d;
	}
	ROMSEL_HI;
	LED_RED_OFF;
	return ok;
}

static int write_chr_flash(unsigned int address, unsigned int len, uint8_t* data)
{
	LED_RED_ON;
	if (address >= 0x8000) address -= 0x8000;
	int ok = 1;
	while (len > 0)
	{
		if (!write_chr_flash_byte(address, *data))
		{
			ok = 0;
			break;
		}
		address++;
		len--;
		data++;
	}
	LED_RED_OFF;
	return ok;
}

void get_mirroring()
{
	comm_start(COMMAND_MIRRORING_RESULT, 4);
	LED_GREEN_ON;
	set_address(0);
	_delay_us(1);
	comm_send_byte((PINE >> 2) & 1);
	set_address(1<<10);
	_delay_us(1);
	comm_send_byte((PINE >> 2) & 1);
	set_address(1<<11);
	_delay_us(1);
	comm_send_byte((PINE >> 2) & 1);
	set_address((1<<10) | (1<<11));
	_delay_us(1);
	comm_send_byte((PINE >> 2) & 1);
	set_address(0);
}

static void init_ports()
{
	DDRB |= (1 << 6) | (1 << 7); // LEDS
	DDRF = 0b10110111; // CPU R/W, IRQ, PPU /RD, PPU /A13, CIRAM /CE, PPU /WR, /ROMSEL, PHI2
	PORTF = 0b11111111; // CPU R/W, IRQ, PPU /RD, PPU /A13, CIRAM /CE, PPU /WR, /ROMSEL, PHI2	
	DDRE &= ~(1<<2); // CIRAM A10
	PORTE |= 1<<2; // CIRAM A10
	MODE_READ;
	set_address(0);
	DDRA = 0xFF; // Address low	
	DDRC = 0xFF; // Address high
}

static void reset_phi2()
{
	LED_RED_ON;
	LED_GREEN_ON;
	PHI2_LOW;
	ROMSEL_HI;
	_delay_ms(100);
	PHI2_HI;
	LED_RED_OFF;
	LED_GREEN_OFF;
}

int main (void)
{
	sei();
	USART_init();
	init_ports();
	jtag_shutdown();

	LED_RED_OFF;
	LED_GREEN_OFF;	
	
	comm_init();
	comm_start(COMMAND_PRG_STARTED, 0);

	uint16_t address;
	uint16_t length;
	
	unsigned long int t = 0;
	char led_down = 0;
	int led_bright = 0;
	
	while (1)
	{
		TCCR1A |= (1<<COM1C1) | (1<<COM1B1) | (1<<WGM10);
		TCCR1B |= (1<<CS10);
		if (t++ >= 10000)
		{
			if (!led_down)
			{
				led_bright++;
				if (led_bright >= 110) led_down = 1;
			} else {
				led_bright--;
				if (!led_bright) led_down = 0;
			}
			if (led_bright >= 100) OCR1B = led_bright - 100;
			if (led_down)
			{
				int led_bright2 = 110-led_bright;
				if (led_bright2 <= 20)
				{
					if (led_bright2 > 10) led_bright2 = 20 - led_bright2;
					OCR1C = led_bright2*2;
				}
			}
			t = 0;
		}
		
		if (comm_recv_done)
		{
			comm_recv_done = 0;
			t = led_down = led_bright = 0;
			TCCR1A = OCR1B = OCR1C = 0;
			
			switch (comm_recv_command)
			{
				case COMMAND_PRG_INIT:
					comm_start(COMMAND_PRG_STARTED, 0);
					break;
					
				case COMMAND_PRG_READ_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					read_prg_send(address, length);
					break;

				case COMMAND_PRG_CRC_READ_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					read_prg_crc_send(address, length);
					break;

				case COMMAND_PRG_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					write_prg(address, length, (uint8_t*)&recv_buffer[4]);
					comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;

				case COMMAND_COOLBOY_READ_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					read_coolboy_send(address, length);
					break;

				case COMMAND_PHI2_INIT:
					phi2_init();
					comm_start(COMMAND_PHI2_INIT_DONE, 0);
					break;

				case COMMAND_RESET:
					reset_phi2();
					comm_start(COMMAND_RESET_ACK, 0);
					break;
					
				case COMMAND_PRG_FLASH_ERASE_REQUEST:
					if (erase_prg_flash())
						comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;

				case COMMAND_PRG_FLASH_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					if (write_prg_flash(address, length, (uint8_t*)&recv_buffer[4]))
						comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;
					
				case COMMAND_COOLBOY_ERASE_REQUEST:
					if (erase_coolboy_sector())
						comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;

				case COMMAND_COOLBOY_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					if (write_coolboy(address, length, (uint8_t*)&recv_buffer[4]))
						comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;
					
				case COMMAND_COOLGIRL_ERASE_SECTOR_REQUEST:
					if (erase_coolgirl_sector())
						comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;

				case COMMAND_COOLGIRL_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					if (write_coolgirl(address, length, (uint8_t*)&recv_buffer[4]))
						comm_start(COMMAND_PRG_WRITE_DONE, 0);
					break;
					
				case COMMAND_CHR_INIT:
					comm_start(COMMAND_CHR_STARTED, 0);
					break;
					
				case COMMAND_CHR_READ_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					read_chr_send(address, length);
					break;

				case COMMAND_CHR_CRC_READ_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					read_chr_crc_send(address, length);
					break;

				case COMMAND_CHR_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					write_chr(address, length, (uint8_t*)&recv_buffer[4]);
					comm_start(COMMAND_CHR_WRITE_DONE, 0);
					break;

				case COMMAND_MIRRORING_REQUEST:
					get_mirroring();
					break;

				/*
				case COMMAND_EPROM_PREPARE:
					write_eprom_prepare();
					break;
				
				case COMMAND_CHR_EPROM_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					write_eprom(address, length, (uint8_t*)&recv_buffer[4]);
					comm_start(COMMAND_CHR_WRITE_DONE, 0);
					break;
				*/
					
				case COMMAND_CHR_FLASH_ERASE_REQUEST:
					if (erase_chr_flash())
						comm_start(COMMAND_CHR_WRITE_DONE, 0);
					break;

				case COMMAND_CHR_FLASH_WRITE_REQUEST:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
					if (write_chr_flash(address, length, (uint8_t*)&recv_buffer[4]))
						comm_start(COMMAND_CHR_WRITE_DONE, 0);
					break;
					
				case COMMAND_JTAG_SETUP:
					jtag_setup();
					comm_start(COMMAND_JTAG_RESULT, 1);
					comm_send_byte(1);
					break;
					
				case COMMAND_JTAG_SHUTDOWN:
					jtag_shutdown();
					comm_start(COMMAND_JTAG_RESULT, 1);
					comm_send_byte(1);
					break;
				
				case COMMAND_JTAG_EXECUTE:
					address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
					comm_start(COMMAND_JTAG_RESULT, 1);
					LED_RED_ON;
					comm_send_byte(jtag_execute(address, (uint8_t*) &recv_buffer[2]));
					LED_RED_OFF;
					break;
				case COMMAND_BOOTLOADER:
					cli();
					MCUCSR = 0;
					jump_to_bootloader();
			}
		}		
	}
}
