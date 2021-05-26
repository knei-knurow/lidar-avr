ROOT := ${CURDIR}

MMCU := atmega328p
CC := avr-gcc
CFLAGS := -std=c99 -mmcu=$(MMCU) -Wall

flash: main.hex
	avrdude -c usbasp -p m328p -e -U flash:w:main.hex

main.hex: main.elf
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

main.elf: main.c
	$(CC) $(CFLAGS) -Os  -o main.elf main.c usart/usart.c mpu6050/mpu6050.c mpu6050/i2c.c -DF_CPU=16000000UL

clean:
	rm -rf *.hex *.elf *.o
