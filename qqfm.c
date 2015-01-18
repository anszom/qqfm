/*
	qqfm: tiny FM transmitter dongle based on the Sony-Erricson MMR-070 
		
	This code is executed on the Atmega32 chip on MMR-070.

	Author: Andrzej Szombierski <qq@kuku.eu.org>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "ns741.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "i2c.h"

#include <util/crc16.h>

// device configuration, stored in EEPROM
// these values can be changed via the audio-UART interface
uint32_t config[] EEMEM = {
	95000, 	// transmitter frequency
	3, 	// transmitter power
	1, 	// stereo
	0, 	// mute
	1	// volume boost
};

// post-initialization sequence, stored in EEPROM
// these values can be changed via the audio-UART interface
uint8_t i2c_init_seq[128] EEMEM =
{
//	reg,  value
	0xff, 0xff,  // 0xff, 0xff is a terminator
		     // the rest is just an example
	0x00, 0x02, 
	0x01, 0x83, 
	0x02, 0x0A, 
	0x03, 0x00,
	0x04, 0x00, 
	0x05, 0x00, 
	0x06, 0x00, 
	0x07, 0x7E,
	0x08, 0x0E, 
	0x09, 0x08, 
	0x0a, 0x3F, 
	0x0b, 0x2A,
	0x0c, 0x0C, 
	0x0d, 0xE6, 
	0x0e, 0x3F, 
	0x0f, 0x70,
	0x10, 0x0A, 
	0x11, 0xE4, 
	0x12, 0x00, 
	0x13, 0x42,
	0x14, 0xC0, 
	0x15, 0x41, 
	0x16, 0xF4,
	0x02, 0x0B,
	0x15, 0x11,
	0x07, 0x7E,
	0x08, 0x0E,
	0x02, 0xCA,
	0x01, 0x81,
	0x0E, 0x3F,
	0x0F, 0x70,
	0x10, 0x0A,
	0x11, 0xE4,
	0x12, 0x00,
	0x13, 0x42,
	0x14, 0xC0,
	0xff, 0xff,
};     

// UART communications at 3200 baud
// 3200 is a nice divisor of 48000
#define BAUD 3200
#include "util/setbaud.h"

// for debugging
//#define DEBUG_FLASH
//#define DEBUG_BIT

// we are using the watchdog timer to implement soft-reboot
// see http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
#include <avr/wdt.h>

// disable the WDT at boot, not sure if atmega32 does it by default
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void)
{
	MCUSR = 0;
	wdt_disable();
	return;
}

// buffer for data received via uart
// every message has the following format:
// "qqfm" (synchronization sequence, not stored in this buffer)
// 0: cmd 
// 1: addr 
// 2345: val val val val 
// 67: crc crc
uint8_t msg[8];

void handle_msg()
{
	// process received message
	uint16_t crc = 0;
	uint8_t i;
	for(i=0;i<sizeof(msg)-2;i++)
		crc = _crc16_update(crc, msg[i]);

	if(msg[6] == (crc&255) && msg[7] == (crc>>8)) {
		// crc match
		uint8_t cmd = msg[0];
		uint8_t addr = msg[1];
		
		if(cmd == 1 && addr < sizeof(config)/sizeof(config[0])) {
			// write config value	
			uint32_t val = msg[2] |( (uint32_t)msg[3]<<8) | ((uint32_t)msg[4]<<16) |((uint32_t)msg[5]<<24);
			eeprom_write_dword(&config[addr], val);
			eeprom_busy_wait();
			uint32_t check = eeprom_read_dword(&config[addr]);
			if(check != val)
				return;

		} else if(cmd == 2) {
			// send i2c raw
			i2c_send(addr, msg[2]);

		} else if(cmd == 3) {
			// no-op

		} else if(cmd == 4) {
			// reset
			_delay_ms(100);
			wdt_enable(WDTO_15MS);
			for(;;){}

		} else if(cmd == 5 && addr < sizeof(i2c_init_seq)) {
			static uint8_t last_seq = 0xff;
			uint8_t seq = msg[3];
			// write raw-i2c-init-sequence value

			if(seq == 0 || seq == last_seq+1) {
				// init
				last_seq = seq;
				eeprom_write_byte(&i2c_init_seq[addr], msg[2]);
				eeprom_busy_wait();

				if(msg[4] == 0xff) {
					// final byte -> ack
					last_seq = 0xff;
				} else {
					// no ack
					return;
				}
			} else {
				// ignore
				return;
			}

		} else
			return;

		// acknowledge
		PORTD &= ~(1<<PD7);
		_delay_ms(100);
		PORTD |= (1<<PD7);
		_delay_ms(100);
	}

	// do nothing if the crc doesn't match
}

int main( void )
{
	_delay_ms(100);	
	DDRD = (1<<PD7);

	// enable led
	PORTD=0;

	// start the transmitter
	uint32_t freq = eeprom_read_dword(&config[0]);
	ns741_init();
	ns741_set_frequency(freq);
	ns741_txpwr(eeprom_read_dword(&config[1]));
	ns741_power(1);
	ns741_stereo(eeprom_read_dword(&config[2]));
	ns741_rds(0);
	ns741_mute(eeprom_read_dword(&config[3]));
	ns741_boost(eeprom_read_dword(&config[4]));

	{
		// post-init sequence
		uint8_t i;
		for(i=0;i<sizeof(i2c_init_seq);i+=2) {
			uint8_t blk[2];
			if(blk[0] == 0xff && blk[1] == 0xff)
				break;

			PORTD ^= (1<<PD7);
			eeprom_read_block(blk, i2c_init_seq+i, 2);
			i2c_send(blk[0], blk[1]);
			PORTD ^= (1<<PD7);
		}
	}

#ifdef DEBUG_FLASH
	// flash led continuously
	for(;;){
		_delay_ms(100);
		PORTD ^= (1<<PD7);
	}
#endif

#ifdef DEBUG_BIT
	// dump the current rx bit over spi
	DDRB |= (1<<PB6);
	SPCR = (1<<SPE);
	for(;;) {
		uint8_t tmp = (PIND&1)+'0';
		if(SPSR & (1<<SPIF)) {
			SPDR=tmp;
		}
	}
#endif

	// setup uart
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
	UCSRB = (1<<RXEN);
	UCSRC = (1<<URSEL)|(3<<UCSZ0);

	uint8_t s0=0, s1=0, s2=0, s3=0;
	int8_t sync = -1;

	// disable led
	PORTD = (1<<PD7);

	// we produce a pulsing pattern on the status LED when idle
	uint8_t seq[256] = { 0,0,0,1,2,3,5,7,9,12,15,18,21,25,28,32,37,41,46,51,56,61,67,72,78,84,90,96,102,108,114,121,127,133,139,146,152,158,164,170,176,181,187,192,198,203,208,213,217,221,225,229,233,236,239,242,245,247,249,251,252,253,254,254,254,254,254,253,252,251,249,247,245,242,240,236,233,230,226,222,217,213,208,203,198,193,187,182,176,170,164,158,152,146,140,134,127,121,115,109,102,96,90,84,79,73,67,62,56,51,46,42,37,33,29,25,21,18,15,12,9,7,5,3,2,1 };
	// For some reason I assumed that the LED is not connected to a PWM channel, 
	// so I'm running a poor man's pwm emulation, which is "good enough for me".
	// Obviously you can use the real PWM instead.
	uint8_t counter = 0, value = 0;
	uint16_t seqi=0;

	// This whole loop could be replaced with an interrupt-based program, but I
	// was too lazy to change it (if it works, don't fix it).

	while(1) {
		// poor man's pwm
		counter++;
		if(counter < value)
			PORTD &= ~(1<<PD7);
		else
			PORTD |= (1<<PD7);

		// pulsing pattern
		if(counter == 0)
			value = seq[(seqi++ >> 4) & 0xff];

		// spi debug output
		if(SPSR & (1<<SPIF)) 
			SPDR = freq;

		// uart byte received
		if(UCSRA & (1<<RXC)) {
			uint8_t byte = UDR;
			// s0 s1 s2 s3 make a 4-byte fifo buffer 
			s0=s1;
			s1=s2;
			s2=s3;
			s3=byte;

			// sync < 0 means that we are looking for the message header ("qqfm")
			// sync >= 0 means that we are in the process of receiving a message
			if(sync >= 0) 
				msg[sync++] = byte;

			if(s0 == 'q' && s1 == 'q' && s2 == 'f' && s3 == 'm') 
				sync=0;

			if(sync == sizeof(msg)) {
				// full message received, handle it
				handle_msg();
				sync = -1;
			}
		}
	}
}
