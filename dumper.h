#ifndef _DUMPER_H_
#define _DUMPER_H_

#define CONCAT(a, b) a ## b
#define COOLBOY_OUTPORT(name) CONCAT(PORT, name)
#define COOLBOY_DDRPORT(name) CONCAT(DDR, name)
#define COOLBOY_INPORT(name) CONCAT(PIN, name)
#define COOLBOY_PORT COOLBOY_OUTPORT(COOLBOY_GPIO_PORT)
#define COOLBOY_DDR COOLBOY_DDRPORT(COOLBOY_GPIO_PORT)
#define COOLBOY_PIN COOLBOY_INPORT(COOLBOY_GPIO_PORT)

#define IRQ_FIRED (!(PINF & (1<<6)))

#define FDS_IRQ_CONTROL 0x4022
#define FDS_MASTER_IO 0x4023
#define FDS_DATA_WRITE 0x4024
#define FDS_CONTROL 0x4025
#define FDS_EXT_WRITE 0x4026
#define FDS_DISK_STATUS 0x4030
#define FDS_DATA_READ 0x4031
#define FDS_DRIVE_STATUS 0x4032
#define FDS_EXT_READ 0x4033

#define FDS_CONTROL_MOTOR_ON 0b00000001
#define FDS_MOTOR_OFF 0b00000010
#define FDS_CONTROL_READ 0b00000100
#define FDS_CONTROL_WRITE 0b00000000
#define FDS_CONTROL_CRC 0b00010000
#define FDS_CONTROL_TRANSFER_ON 0b01000000
#define FDS_CONTROL_IRQ_ON 0b10000000

#define FDS_READ_GAP_BEFORE_FIRST_BLOCK 486974
#define FDS_WRITE_GAP_BEFORE_FIRST_BLOCK 580000
#define FDS_READ_GAP_BETWEEN_BLOCKS 9026
#define FDS_WRITE_GAP_BETWEEN_BLOCKS 17917
#define FDS_WRITE_CRC_DELAY 897

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

#define DELAY_CLOCK(t) _delay_us(t * 1000000 / 1789773)
#define DELAY_KILO_CLOCK(t) _delay_ms(t * 1000000 / 1789773)

#endif
