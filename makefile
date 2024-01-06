# makefile for AVR projects
# revision history
#	Date		Author			Revision
#	2/14/22		D. McLaughlin	initial release 
# 	2/15/22		D. McLaughlin	updated with corrections (thanks S. Kaza)
#	3/30/22		D. McLaughlin	updated for use with Sparkfun Pocket Programmer
# 	4/4/22		D. McLaughlin	updated for multiple source files. 

# Specify the com port (eg, com3 or /dev/tty.usbmodem 1121) for Arduino Uno or usb for usbtiny
SERIALPORT = usb
# Specify the name(s) of your source code file(s) here:
SOURCEFILES = 304finalbuild.c SSD1306.c i2c.c
# Use 1000000 for a new ATmega328P IC; use 16000000 for Arduino Uno
CLOCKSPEED = 16000000	
# Use usbtiny for the Sparkfun Pocket Programmer; Arduino for Arduino Uno
PROGRAMMER = usbtiny

begin:	main.hex

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size --format=avr --mcu=atmega328p main.elf

main.elf: $(SOURCEFILES)
	avr-gcc -Wall -Os -DF_CPU=$(CLOCKSPEED) -mmcu=atmega328p -o main.elf $(SOURCEFILES)

flash:	begin
	avrdude -c $(PROGRAMMER) -b 115200 -P $(SERIALPORT) -p atmega328p -U flash:w:main.hex:i