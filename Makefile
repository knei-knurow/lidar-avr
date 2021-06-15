ROOT := ${CURDIR}

MMCU := atmega328p
CC := avr-gcc
CFLAGS := -std=c99 -mmcu=$(MMCU) -Wall
INCLUDES := -I$(ROOT)/mpu6050 -I$(ROOT)/usart

flash: main.hex
	avrdude -c usbasp -p m328p -e -U flash:w:main.hex

main.hex: main.elf
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

main.elf: main.o usart.o mpu6050.o i2cmaster.o mpu6050dmp6.o
	$(CC) $(CFLAGS) -Os -o main.elf main.o usart.o mpu6050.o mpu6050dmp6.o i2cmaster.o -DF_CPU=16000000UL

main.o: main.c
	$(CC) $(CFLAGS) -Os -c main.c -DF_CPU=16000000UL $(INCLUDES)

usart.o: usart/usart.c
	$(CC) $(CFLAGS) -Os -c usart/usart.c -DF_CPU=16000000UL $(INCLUDES)

mpu6050.o: mpu6050/mpu6050.c
	$(CC) $(CFLAGS) -Os -c mpu6050/mpu6050.c -DF_CPU=16000000UL $(INCLUDES)

mpu6050dmp6.o: mpu6050/mpu6050dmp6.c
	$(CC) $(CFLAGS) -Os -c mpu6050/mpu6050dmp6.c -DF_CPU=16000000UL $(INCLUDES)

i2cmaster.o: i2cmaster/i2cmaster.c
	$(CC) $(CFLAGS) -Os -c i2cmaster/i2cmaster.c -DF_CPU=16000000UL $(INCLUDES)


clean:
	rm -rf *.hex *.elf *.o

