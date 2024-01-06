D. McLaughlin Instructions for Use:
4/5/22 

This source code is based on the example found here:
https://audiodiwhy.blogspot.com/2021/06/embedded-c-avr-getting-oled-display.html
the SSD1306 libraries i2c.c, i2c.h, SSD1306.c, and SSD1306.h were downloaded from
https://github.com/Preston-Sundar/AVR-OLED-SSD1306-IIC-DRIVER
The file called README.md is from the author of this code.

I created oldtest.c as a simple application, tested it succesfully using Makefile 
with bare 328p on breadboard and Sparkfun Pocket Programmer

Read through the comments in SSD1306.c to learn what functions are available. 


To build this code, create a  project folder that contains the following 4 files, which constitute
the library for the ic2 interface and the SSD1306 OLED display:
i2c.c
i2c.h
SSD1306.c
SSD1306.h

Also the project folder needs the source code application with main():
oldtest.c  --- this is my exameple code

And be sure to include the makefile (no extension, just makefile)

To compile: type make from Command Prompt (Windows) or Terminal (macOS)
To flash to microcontroller: type make flash

