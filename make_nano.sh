avr-gcc -mmcu=atmega328 -Os main_nano.c -o main_nano.o
avr-objcopy -j .text -j .data -O ihex main_nano.o main_nano.hex
avrdude -c arduino -P /dev/ttyUSB0 -b57600 -p m328p -U flash:w:main_nano.hex
