CFLAGS=-Os -DF_CPU=3686400 -mmcu=atmega32
LDFLAGS=-mmcu=atmega32

all: qqfm.hex qqfm.eehex

qqfm.hex: qqfm.elf
	avr-objcopy -O ihex $< $@ -j .text -j .data
qqfm.eehex: qqfm.elf
	avr-objcopy -O ihex $< $@ -j .eeprom --change-section-lma .eeprom=0 --no-change-warnings

qqfm.elf: qqfm.o ns741.o i2c.o
	avr-gcc $(LDFLAGS) -o $@ $^

%.o: %.c *.h
	avr-gcc $(CFLAGS) -c -o $@ $<
	
