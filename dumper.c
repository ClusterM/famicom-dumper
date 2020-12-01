/* Famicom Dumper/Programmer
 *
 * Copyright notice for this file:
 *  Copyright (C) 2020 Cluster
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
#include "dumper.h"
#include "crc.h"

static void (*jump_to_bootloader)(void) = (void*)0xF800;
uint16_t flash_buffer_mask = 0xFFC0;

ISR(USART0_RX_vect)
{
  unsigned char b;
  while (UCSR0A & (1<<RXC0))
  {
    b = UDR0;
    comm_proceed(b);
  }
}

static void set_flash_buffer_size(uint16_t value)
{
  // Set maximum number of bytes in multi-byte program
  uint8_t bit_value = 0;
  while (value > 1)
  {
    value >>= 1;
    bit_value++;
  }
  flash_buffer_mask = 0xFFFF << bit_value;
}

static void set_address(uint16_t address)
{
  unsigned char l = address & 0xFF;
  unsigned char h = (address >> 8) & 0xFF;
  
  PORTA = l;
  PORTC = h;
  
  // PPU /A13
  if ((address >> 13) & 1)
    PORTF &= ~(1<<4);
  else
    PORTF |= 1<<4;
}

static inline void set_romsel(uint16_t address)
{
  if (address & 0x8000)
  {
    ROMSEL_LOW;
  } else {
    ROMSEL_HI;
  }
}

static inline void set_coolboy_rd(uint16_t address)
{
  if (address & 0x8000)
  {
    COOLBOY_PORT |= 1<<COOLBOY_WR_PIN;
    COOLBOY_PORT &= ~(1<<COOLBOY_RD_PIN);
  } else {
    COOLBOY_PORT |= 1<<COOLBOY_WR_PIN;
    COOLBOY_PORT |= 1<<COOLBOY_RD_PIN;
  }
}

static inline void set_coolboy_wr(uint16_t address)
{
  if (address & 0x8000)
  {
    COOLBOY_PORT &= ~(1<<COOLBOY_WR_PIN);
    COOLBOY_PORT |= 1<<COOLBOY_RD_PIN;
  } else {
    COOLBOY_PORT |= 1<<COOLBOY_WR_PIN;
    COOLBOY_PORT |= 1<<COOLBOY_RD_PIN;
  }
}

static unsigned char read_prg_byte(uint16_t address)
{
  PHI2_LOW;
  ROMSEL_HI;
  MODE_READ;
  PRG_READ;  
  set_address(address);
  PHI2_HI;
  set_romsel(address); // set /ROMSEL low if need
  set_coolboy_rd(address); // COOLBOY's /oe low if need
  _delay_us(1);
  uint8_t result = PIND;
  ROMSEL_HI;
  set_coolboy_rd(0);
  return result;
}

static unsigned char read_chr_byte(uint16_t address)
{
  PHI2_LOW;
  ROMSEL_HI;
  MODE_READ;
  set_address(address);
  CHR_READ_LOW;
  _delay_us(1);
  uint8_t result = PIND;
  CHR_READ_HI;
  PHI2_HI;
  return result;
}

static void read_prg_send(uint16_t address, uint16_t len)
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
  LED_GREEN_OFF;
}

static void read_chr_send(uint16_t address, uint16_t len)
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
  LED_GREEN_OFF;
}

static void read_prg_crc_send(uint16_t address, uint16_t len)
{
  LED_GREEN_ON;
  uint16_t crc = 0;
  MODE_READ;
  PRG_READ;  
  PHI2_HI;
  set_romsel(address); // set /ROMSEL low if need
  while (len > 0)
  {
    PORTA = address & 0xFF;
    PORTC = (address >> 8) & 0xFF;
    _delay_us(1);
    crc = calc_crc16(crc, PIND);
    len--;
    address++;
  }
  ROMSEL_HI;
  set_address(0);
  comm_start(COMMAND_PRG_READ_RESULT, 2);
  comm_send_byte(crc & 0xFF);
  comm_send_byte((crc >> 8) & 0xFF);
  LED_GREEN_OFF;
}

static void read_chr_crc_send(uint16_t address, uint16_t len)
{
  LED_GREEN_ON;
  uint16_t crc = 0;
  while (len > 0)
  {
    crc = calc_crc16(crc, read_chr_byte(address));
    len--;
    address++;
  }
  set_address(0);
  comm_start(COMMAND_CHR_READ_RESULT, 2);
  comm_send_byte(crc & 0xFF);
  comm_send_byte((crc >> 8) & 0xFF);
  LED_GREEN_OFF;
}

static void write_prg_byte(uint16_t address, uint8_t data)
{
  PHI2_LOW;
  ROMSEL_HI;
  MODE_WRITE;
  PRG_WRITE;
  PORTD = data;
  set_address(address); // PHI2 low, ROMSEL always HIGH
  _delay_us(1);
  
  PHI2_HI;
  set_romsel(address); // ROMSEL is low if need, PHI2 high  
  set_coolboy_wr(address); // COOLBOY's /we is low if need
  
  _delay_us(1); // WRITING
  
  // PHI2 low, ROMSEL high
  PHI2_LOW;
  ROMSEL_HI;
  set_coolboy_wr(0);
  
  // Back to read mode
  _delay_us(1);
  PRG_READ;
  MODE_READ;
  set_address(0);

  // Set phi2 to high state to keep cartridge unreseted
  PHI2_HI;
}

static void write_chr_byte(uint16_t address, uint8_t data)
{
  PHI2_LOW;
  ROMSEL_HI;
  MODE_WRITE;
  PORTD = data;  
  set_address(address); // PHI2 low, ROMSEL always HIGH
  CHR_WRITE_LOW;
    
  _delay_us(1); // WRITING
  
  CHR_WRITE_HI;
  
  MODE_READ;
  set_address(0);
  PHI2_HI;
}

static void write_prg(uint16_t address, uint16_t len, uint8_t* data)
{
  LED_RED_ON;
  while (len > 0)
  {
    write_prg_byte(address, *data);
    address++;
    len--;
    data++;
  }
  LED_RED_OFF;
}

static void write_chr(uint16_t address, uint16_t len, uint8_t* data)
{
  LED_RED_ON;
  while (len > 0)
  {
    write_chr_byte(address, *data);
    address++;
    len--;
    data++;
  }
  LED_RED_OFF;
}

static inline void write_prg_flash_command(uint16_t address, uint8_t data)
{
  write_prg_byte(address | 0x8000, data);
}

static void erase_flash_sector()
{
  LED_RED_ON;
  write_prg_flash_command(0x0000, 0xF0);
  write_prg_flash_command(0x0AAA, 0xAA);
  write_prg_flash_command(0x0555, 0x55);
  write_prg_flash_command(0x0AAA, 0x80);
  write_prg_flash_command(0x0AAA, 0xAA);
  write_prg_flash_command(0x0555, 0x55);
  write_prg_flash_command(0x0000, 0x30);
  
  uint8_t res;
  int16_t last_res = -1;
  TCNT1 = 0;
  // waiting for result
  while (1)
  {
    if (TCNT1 >= 23437) // 3 seconds
    {
      // timeout
      comm_start(COMMAND_FLASH_ERASE_TIMEOUT, 0);
      break;
    }
    res = read_prg_byte(0x8000);
    if ((last_res == -1) || ((res != (last_res & 0xFF))))
    {
      // in progress
      last_res = res;
      continue;
    }
    // done
    if (res == 0xFF)
    {
       // ok
       comm_start(COMMAND_PRG_WRITE_DONE, 0);
       break;
    } else {
       // error
       comm_start(COMMAND_FLASH_ERASE_ERROR, 1);
       comm_send_byte(res);
       break;
    }
  }
  LED_RED_OFF;
}

static void write_flash(uint16_t address, uint16_t len, uint8_t* data)
{
  LED_RED_ON;
  while (len > 0)
  {
    uint8_t count = 0;
    uint8_t* d = data;
    uint16_t a = address;
    uint16_t last_address;
    uint8_t last_data;
    uint16_t address_base = a & flash_buffer_mask;
    while ((len > 0) && ((a & flash_buffer_mask) == address_base))
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
          last_address = address;
          last_data = *data;
          count--;
        }
        address++;
        data++;
      }
    
      write_prg_flash_command(0x0000, 0x29);

      TCNT1 = 0;
      // waiting for result
      while (1)
      {
        if (TCNT1 >= 7812) // 1 second
        {
          // timeout
          comm_start(COMMAND_FLASH_WRITE_TIMEOUT, 0);
          LED_RED_OFF;
          return;
        }
        uint8_t read_1 = read_prg_byte(last_address | 0x8000);
        uint8_t read_2 = read_prg_byte(last_address | 0x8000);
        uint8_t read_3 = read_prg_byte(last_address | 0x8000);
        if (((read_1 ^ read_2) & (1 << 6)) && ((read_2 ^ read_3) & (1 << 6)))
        {
          if (read_1 & (1 << 1))
          {
            comm_start(COMMAND_FLASH_WRITE_ERROR, 3);
            comm_send_byte(read_1);
            comm_send_byte(read_2);
            comm_send_byte(read_3);
            LED_RED_OFF;
            return;
          } else if (read_1 & (1 << 5)) {
            comm_start(COMMAND_FLASH_WRITE_TIMEOUT, 3);
            comm_send_byte(read_1);
            comm_send_byte(read_2);
            comm_send_byte(read_3);
            LED_RED_OFF;
            return;
          }
        } else {
          read_1 = read_prg_byte(last_address | 0x8000);
          read_2 = read_prg_byte(last_address | 0x8000);
          if (read_1 == read_2 && read_2 == last_data)
            break; // ok
        }
      }
    }

    address = a;
    data = d;
  }
  comm_start(COMMAND_PRG_WRITE_DONE, 0);
  LED_RED_OFF;
}

static uint8_t transfer_fds_byte(uint8_t *output, uint8_t input, uint8_t *end_of_head)
{
  TCNT1 = 0;
  while (!IRQ_FIRED)
  {
    // waiting for interrupt
    // timeout 5 secs
    if (TCNT1 >= 39060)
    {
      write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_RESET); // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
  }
  if (output)
    *output = read_prg_byte(FDS_DATA_READ);
  write_prg_byte(FDS_DATA_WRITE, input); // clear interrupt
  uint8_t status = read_prg_byte(FDS_DISK_STATUS);
  if (end_of_head)
    *end_of_head |= (status >> 6) & 1;
  TCNT1 = 0;
  while (IRQ_FIRED)
  {
    // is interrupt flag cleared?
    // timeout 5 secs
    if (TCNT1 >= 39060)
    {
      write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_RESET); // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
  }
  return 1;
}

static uint8_t read_fds_block_send(uint16_t length, uint8_t send, uint8_t *crc_ok, uint8_t *end_of_head, uint16_t *file_size, uint32_t gap_delay)
{
  uint8_t data;
  uint8_t status;
  uint32_t b;

  write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON); // motor on without transfer
  if (gap_delay < 30000)
    DELAY_CLOCK(gap_delay);
  else
    DELAY_KILO_CLOCK(gap_delay / 1000);
  if (send)
  {
    LED_GREEN_ON;
    comm_start(COMMAND_FDS_READ_RESULT_BLOCK, length + 2);
  }
  // start transfer, enable IRQ
  write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON);
  for (b = 0; b < length; b++)
  {
    if (!transfer_fds_byte(&data, 0, end_of_head))
      return 0;
    if (file_size)
    {
      if (b == 13)
        *file_size |= data;
      else if (b == 14)
        *file_size |= data << 8;
    }
    if (send)
      comm_send_byte(data);
  }
  if (!transfer_fds_byte(0, 0, end_of_head))
    return 0;
  write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON | FDS_CONTROL_CRC); // enable CRC control
  if (!transfer_fds_byte(0, 0, end_of_head))
    return 0;
  status = read_prg_byte(FDS_DISK_STATUS);
  *crc_ok &= ((status >> 4) & 1) ^ 1;
  *end_of_head |= (status >> 6) & 1;
  if (send)
  {
    comm_send_byte(*crc_ok); // CRC check result
    comm_send_byte(*end_of_head); // end of head meet?
  }
  LED_GREEN_OFF;
  return 1; // success
}

static uint8_t write_fds_block(uint8_t *data, uint16_t length, uint32_t gap_delay)
{
  uint8_t end_of_head = 0;
  LED_RED_ON;
  write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON); // motor on without transfer
  read_prg_byte(FDS_DRIVE_STATUS); // check if disk is inserted
  write_prg_byte(FDS_CONTROL, FDS_CONTROL_WRITE | FDS_CONTROL_MOTOR_ON); // enable writing without transfer
  if (gap_delay < 30000)
    DELAY_CLOCK(gap_delay);
  else
    DELAY_KILO_CLOCK(gap_delay / 1000);
  write_prg_byte(FDS_DATA_WRITE, 0x00); // write $00
  // start transfer, enable IRQ
  write_prg_byte(FDS_CONTROL, FDS_CONTROL_WRITE | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON);
  transfer_fds_byte(0, 0x80, &end_of_head);  // write $80
  while (length)
  {
    if (end_of_head)
    {
      write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_RESET); // reset, stop
      comm_start(COMMAND_FDS_END_OF_HEAD, 0);
      return 0;
    }
    if (!transfer_fds_byte(0, *data, &end_of_head))
    {
      write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_RESET); // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
    data++;
    length--;
  }
  if (!transfer_fds_byte(0, 0xFF, &end_of_head))
  {
    write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_RESET); // reset, stop
    comm_start(COMMAND_FDS_TIMEOUT, 0);
    return 0;
  }
  if (end_of_head)
  {
    write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_RESET); // reset, stop
    comm_start(COMMAND_FDS_END_OF_HEAD, 0);
    return 0;
  }
  write_prg_byte(FDS_CONTROL, FDS_CONTROL_WRITE | FDS_CONTROL_MOTOR_ON | FDS_CONTROL_TRANSFER_ON | FDS_CONTROL_IRQ_ON | FDS_CONTROL_CRC);  // enable CRC control
  DELAY_CLOCK(FDS_WRITE_CRC_DELAY);
  TCNT1 = 0;
  while (1)
  {
    uint8_t status = read_prg_byte(FDS_DRIVE_STATUS);
    if (!(status & 2))
      break; // ready
    // timeout 1 sec
    if (TCNT1 >= 7812)
    {
      write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_RESET); // reset, stop
      comm_start(COMMAND_FDS_TIMEOUT, 0);
      return 0;
    }
  }  
  LED_RED_OFF;
  return 1;
}

static void fds_transfer(uint8_t block_read_start, uint8_t block_read_count, uint8_t block_write_count, uint8_t *block_write_ids, uint16_t *write_lengths,
    uint8_t *write_data)
{
  uint8_t crc_ok = 1;
  uint8_t end_of_head = 0;
  uint8_t current_block = 0;
  uint8_t current_writing_block = 0;

  write_prg_byte(FDS_IRQ_CONTROL, 0x00); // disable timer IRQ
  write_prg_byte(FDS_MASTER_IO, 0x01); // enable disk registers
  write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_RESET); // reset
  uint8_t ram_adapter_connected = 1;
  write_prg_byte(FDS_EXT_WRITE, 0x00); // Ext. connector
  write_prg_byte(0x0000, 0xFF); // To prevent open bus read
  if ((read_prg_byte(FDS_EXT_READ) & 0x7F) != 0x00)
    ram_adapter_connected = 0;
  write_prg_byte(FDS_EXT_WRITE, 0xFF); // Ext. connector
  write_prg_byte(0x0000, 0x00); // To prevent open bus read
  if ((read_prg_byte(FDS_EXT_READ) & 0x7F) != 0x7F)
    ram_adapter_connected = 0;
  if (!ram_adapter_connected)
  {
    comm_start(COMMAND_FDS_NOT_CONNECTED, 0);
    return;
  }
  if (read_prg_byte(FDS_DRIVE_STATUS) & 1)
  {
    comm_start(COMMAND_FDS_DISK_NOT_INSERTED, 0);
    return;
  }
  DELAY_KILO_CLOCK(916500 / 1000); // ~916500 cycles
  write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON); // monor on, unreset
  DELAY_KILO_CLOCK(268500 / 1000); // ~268500 cycles
  write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_RESET); // reset
  write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_MOTOR_ON); // monor on, unreset
  // waiting until drive is rewinded
  TCNT1 = 0;
  uint8_t secs = 0;
  do
  {
    // timeout 15 secs
    if (TCNT1 >= 7812)
    {
      TCNT1 = 0;
      secs++;
      if (secs >= 15)
      {
        write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_RESET); // reset, stop
        comm_start(COMMAND_FDS_TIMEOUT, 0);
        return;
      }
    }
  } while (read_prg_byte(FDS_DRIVE_STATUS) & 2);

  // disk info block
  if (block_write_count && (current_block == block_write_ids[current_writing_block]))
  {
    // gap delay while writing = ~28300 bits = (~28300 / 8)bits * ~165cycles = ~583687.5
    uint16_t write_length = write_lengths[current_writing_block];
    if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BEFORE_FIRST_BLOCK))
      return;
    write_data += write_length;
    current_writing_block++;
    block_write_count--;
  } else
  {
    // gap delay while reading = ~486974 cycles
    if (!read_fds_block_send(56, (current_block >= block_read_start) && block_read_count, &crc_ok, &end_of_head, 0, FDS_READ_GAP_BEFORE_FIRST_BLOCK))
      return;
  }
  if ((current_block >= block_read_start) && block_read_count)
    block_read_count--;
  current_block++;

  if (crc_ok && !end_of_head && (block_read_count || block_write_count))
  {
    // file amount block
    if (block_write_count && (current_block == block_write_ids[current_writing_block]))
    {
      uint16_t write_length = write_lengths[current_writing_block];
      if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BETWEEN_BLOCKS))
        return;
      write_data += write_length;
      current_writing_block++;
      block_write_count--;
    } else
    {
      if (!read_fds_block_send(2, (current_block >= block_read_start) && block_read_count, &crc_ok, &end_of_head, 0, FDS_READ_GAP_BETWEEN_BLOCKS))
        return;
    }
    if ((current_block >= block_read_start) && block_read_count)
      block_read_count--;
    current_block++;
  }

  while (crc_ok && !end_of_head && (block_read_count || block_write_count))
  {
    // file header block
    uint16_t file_size = 0; // size of the next file
    if (block_write_count && (current_block == block_write_ids[current_writing_block]))
    {
      uint16_t write_length = write_lengths[current_writing_block];
      if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BETWEEN_BLOCKS))
        return;
      write_data += write_length;
      current_writing_block++;
      block_write_count--;
    } else
    {
      if (!read_fds_block_send(16, (current_block >= block_read_start) && block_read_count, &crc_ok, &end_of_head, &file_size, FDS_READ_GAP_BETWEEN_BLOCKS))
        return;
    }
    if ((current_block >= block_read_start) && block_read_count)
      block_read_count--;
    current_block++;

    if (crc_ok && !end_of_head && (block_read_count || block_write_count))
    {
      // file data block
      if (block_write_count && (current_block == block_write_ids[current_writing_block]))
      {
        uint16_t write_length = write_lengths[current_writing_block];
        if (!write_fds_block(write_data, write_length, FDS_WRITE_GAP_BETWEEN_BLOCKS))
          return;
        write_data += write_length;
        current_writing_block++;
        block_write_count--;
      } else
      {
        if (!read_fds_block_send(file_size + 1, (current_block >= block_read_start) && block_read_count, &crc_ok, &end_of_head, 0, FDS_READ_GAP_BETWEEN_BLOCKS))
          return;
      }
      if ((current_block >= block_read_start) && block_read_count)
        block_read_count--;
      current_block++;
    }
  }

  write_prg_byte(FDS_CONTROL, FDS_CONTROL_READ | FDS_CONTROL_RESET); // reset, stop

  _delay_ms(50);
  if (current_writing_block && !block_write_count && !block_read_count)
  {
    comm_start(COMMAND_FDS_WRITE_DONE, 0);
    return;
  }
  comm_start(COMMAND_FDS_READ_RESULT_END, 0);
}

static void get_mirroring()
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
  COOLBOY_DDR |= (1<<COOLBOY_RD_PIN) | (1<<COOLBOY_WR_PIN);
  COOLBOY_PORT |= (1<<COOLBOY_RD_PIN) | (1<<COOLBOY_WR_PIN);
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

  LED_RED_OFF;
  LED_GREEN_OFF;  
  
  comm_init();
  comm_start(COMMAND_PRG_STARTED, 0);

  uint32_t address;
  uint32_t length;
  
  unsigned long int t = 0;
  char led_down = 0;
  int led_bright = 0;
  
  while (1)
  {
    // PWM for leds
    TCCR1A |= (1<<COM1C1) | (1<<COM1B1) | (1<<WGM10);
    TCCR1B = 1<<CS10; // no prescaler
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
      // Just timer without PWM for timeouts
      TCCR1A = OCR1B = OCR1C = 0;
      TCCR1B = (1<<CS12) | (1<<CS10); // /1024 prescaler
      
      switch (comm_recv_command)
      {
        case COMMAND_PRG_INIT:
          comm_start(COMMAND_PRG_STARTED, 5);
          comm_send_byte(PROTOCOL_VERSION);
          comm_send_byte((SEND_BUFFER_SIZE - 4) & 0xFF);
          comm_send_byte(((SEND_BUFFER_SIZE - 4) >> 8) & 0xFF);
          comm_send_byte((RECV_BUFFER_SIZE - 4) & 0xFF);
          comm_send_byte(((RECV_BUFFER_SIZE - 4) >> 8) & 0xFF);
          break;
          
        case COMMAND_COOLBOY_READ_REQUEST:
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

        case COMMAND_RESET:
          reset_phi2();
          comm_start(COMMAND_RESET_ACK, 0);
          break;
          
        case COMMAND_FLASH_ERASE_SECTOR_REQUEST:
        case COMMAND_COOLBOY_ERASE_SECTOR_REQUEST:
          erase_flash_sector();
          break;

        case COMMAND_FLASH_WRITE_REQUEST:
        case COMMAND_COOLBOY_WRITE_REQUEST:
          address = recv_buffer[0] | ((uint16_t)recv_buffer[1]<<8);
          length = recv_buffer[2] | ((uint16_t)recv_buffer[3]<<8);
          write_flash(address, length, (uint8_t*)&recv_buffer[4]);
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

        case COMMAND_FDS_READ_REQUEST:
          fds_transfer(recv_buffer[0], recv_buffer[1], 0, 0, 0, 0);
          break;

        case COMMAND_FDS_WRITE_REQUEST:
          fds_transfer(0, 0, recv_buffer[0], (uint8_t*) &recv_buffer[1], (uint16_t*) &recv_buffer[1 + recv_buffer[0]],
            (uint8_t*) &recv_buffer[1 + recv_buffer[0] + recv_buffer[0] * 2]);
          break;        
        
        case COMMAND_MIRRORING_REQUEST:
          get_mirroring();
          break;
          
        case COMMAND_SET_FLASH_BUFFER_SIZE:
          set_flash_buffer_size(recv_buffer[0] | ((uint16_t) recv_buffer[1] << 8));
          comm_start(COMMAND_SET_VALUE_DONE, 0);
          break;

        case COMMAND_BOOTLOADER:
          cli();
          MCUCSR = 0;
          jump_to_bootloader();
      }

      LED_GREEN_OFF;
      LED_RED_OFF;
    }    
  }
}
