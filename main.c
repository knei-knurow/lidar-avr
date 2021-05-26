/**
 * This program is a proxy between the rover's main computer
 * and the lidar servo.
 *
 * MCU: ATMega328 (Arduino Uno)
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "mpu6050/mpu6050.h"

#define MIN_DUTY 1600
#define MAX_DUTY 4400

#define FRAME_LENGTH 17

void led_init() {
  DDRB = DDRB | (1 << PB5);
}

void led_blink(unsigned time) {
  static unsigned cnt = 0;
  if (cnt == 0) {
    PORTB = PORTB ^ (1 << PB5);  // Debug LED blink
  }
  cnt = (cnt + 1) % time;
}

void usart_write_byte(uint8_t byte) {
  // Wait for empty transmit buffer.
  while (!(UCSR0A & (1 << UDRE0))) {
  };
  // UCSR0A = USART Control and Status Register 0 A.
  // UDRE0 = USART Data Register Empty 0.

  UDR0 = byte;
  // UDR0 = USART Data Register 0.
}

void acc_init() {
  mpu6050_start();
}

void usart_write_acc_frame(uint8_t* frame, int frame_length) {
  for (int i = 0; i < frame_length; i++) {
    usart_write_byte(frame[i]);
  }
}

int main(void) {
  TCCR1A |= (1 << WGM11);                 // Set Fast-PWM mode 1/2
  TCCR1B |= (1 << WGM12) | (1 << WGM13);  // Set Fast-PWM mode 2/2
  TCCR1A |= (1 << COM1A1);                // Set non-inverting PWM mode
  DDRB |= (1 << PB1);                     // Set PORTB1 to be output (we use later for OCR1A pin )
  OCR1A = MIN_DUTY;                       // Set PWM duty
  ICR1 = 39999;  // Set PWM period and prescaler (period = 20ms; prescaler = 8)
  TCCR1B |= (1 << CS11);

  UBRR0 = 103;                              // Set USART baudrate to 9600 bps
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);    // Enable USART receiver and transmitter
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);  // Set USART frame to be 8 bits
  UCSR0B |= (1 << RXCIE0);  // Enable interrupt to fire when USART receives data receives data

  // DDRB = Data Direction Register for port B.
  // Setting PIN5 on PORTB to 1. 1 means it is an output pin.

  sei();  // Enable global interrupts

  led_init();

  uint8_t frame[FRAME_LENGTH];
  while (1) {
    for (int i = MIN_DUTY; i <= MAX_DUTY; i++) {
      _delay_ms(10);

      frame[0] = 'L';
      frame[1] = 'D';
      frame[2] = '-';
      mpu6050_read_gyro_X(3 + frame + 0);
      mpu6050_read_gyro_Y(3 + frame + 2);
      mpu6050_read_gyro_Z(3 + frame + 4);
      mpu6050_read_accel_X(3 + frame + 6);
      mpu6050_read_accel_Y(3 + frame + 8);
      mpu6050_read_accel_Z(3 + frame + 10);
      frame[15] = 0xA;
      frame[16] = 0xD;

      usart_write_acc_frame(frame, FRAME_LENGTH);

      // UDR0 = v[0]; it was here before - delete in the future

      OCR1A = i;

      led_blink(50);
    }
  }
}

ISR(USART_RX_vect) {
  static uint8_t byteNumber = 0;
  static uint8_t value8LSB = 0;
  static uint8_t value8MSB = 0;

  static uint8_t crc = 0;

  uint8_t input = UDR0;

  switch (byteNumber) {
    case 0:
      if (input == 'L') {
        crc = input;

        byteNumber++;
      } else {
        // TODO: handle error gracefully?
        byteNumber = 0;
        break;
      }
      break;
    case 1:
      if (input == 'D') {
        crc ^= input;

        byteNumber++;
      } else {
        // TODO: handle error gracefully?
        byteNumber = 0;
      }
      break;
    case 2:
      if (input == '+') {
        crc ^= input;

        byteNumber++;
      } else {
        // TODO: handle error gracefully?
        byteNumber = 0;
      }
      break;
    case 3:
      value8MSB = input;
      crc ^= input;

      byteNumber++;
      break;
    case 4:
      value8LSB = input;
      crc ^= input;

      byteNumber++;
      break;
    case 5:
      if (input == '#') {
        byteNumber++;
        crc ^= input;
      }
      break;
    case 6:
      if (input == crc) {
        // OK
        byteNumber = 0;
        crc = 0;
      } else {
        // TODO: handle error gracefully?
      }
      break;
  }

  uint16_t receivedPWMDuty = (value8MSB << 8) + value8LSB;

  uint16_t calculatedPWMDuty = (input * (MAX_DUTY - MIN_DUTY)) / 255 + MIN_DUTY;

  if (receivedPWMDuty >= MIN_DUTY && receivedPWMDuty <= MAX_DUTY) {
    OCR1A = receivedPWMDuty;  // Set TOP to calculated PWM duty.
  }

  // TODO: Fix (only 8LSB bytes are sent)
  // UDR0 = receivedPWMDuty;  // Send back what we got.
}
