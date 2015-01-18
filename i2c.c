#include <util/delay.h>
#include <avr/io.h>
#include "i2c.h"

uint8_t slave_addr;

int i2c_init(uint8_t addr)
{
	DDRC |= (1<<PC0) | (1<<PC1);	//SDA und SCL
	
	TWBR = 0x17;
	TWCR = (1<<TWEN);// | (1<<TWIE);

	slave_addr = addr<<1;
	i2c_send(0,0); // init
}

void i2c_send(uint8_t reg, uint8_t byte)
{
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);	//START
	
	while(!(TWCR & (1<<TWINT)));
	
	TWDR = slave_addr | 0x00;
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	
	TWDR = reg;
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	
	TWDR = byte;
	TWCR = (1<<TWINT) | (1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	_delay_ms(2);
}

void i2c_writeData(const uint8_t *data, uint8_t size)
{
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);	//START
	
	while(!(TWCR & (1<<TWINT)));
	
	TWDR = slave_addr | 0x00;
	TWCR = (1<<TWINT) | (1<<TWEN);

	while(size > 0) {	
		while(!(TWCR & (1<<TWINT)));
	
		TWDR = *data;
		TWCR = (1<<TWINT) | (1<<TWEN);
		++data;
		--size;
		_delay_ms(1);
	}
	
	while(!(TWCR & (1<<TWINT)));
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	_delay_ms(1);
}
