MCU=atmega32
CC=avr-gcc
OBJCOPY=avr-objcopy
# optimize for size:
CFLAGS=-g -mmcu=$(MCU) -Wall -Wstrict-prototypes -Os -ffunction-sections -mcall-prologues

all: synth.hex
synth.bin : synth.out 
	$(OBJCOPY) -R .eeprom -O binary synth.out synth.bin 
synth.hex : synth.out 
	$(OBJCOPY) -R .eeprom -O ihex synth.out synth.hex 
synth.out : synth.o nanomidi.o
	$(CC) $(CFLAGS) -o synth.out -Wl,-Map,synth.map,--gc-sections synth.o nanomidi.o
synth.o : synth.c
	$(CC) $(CFLAGS) -Os -c synth.c
nanomidi.o : nanomidi.c
	$(CC) $(CFLAGS) -Os -c nanomidi.c

# erase the AVR before it is programmed
load: synth.hex
	uisp -dlpt=/dev/parport0 --erase  -dprog=bsd
	uisp -dlpt=/dev/parport0 --upload if=synth.hex -dprog=bsd  -v=3 --hash=32

clean:
	rm -f *.o *.map *.out
