flash: main.hex
	avrdude -c usbasp -p m328p -e -U flash:w:main.hex

main.hex: main.elf
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

main.elf: main.c
	avr-gcc -mmcu=atmega328p -Wall -Os -o main.elf main.c mpu6050/mpu6050.c mpu6050/i2c.c -DF_CPU=16000000UL

clean:
	rm -rf *.hex *.elf *.o
