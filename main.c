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
#include "usart/usart.h"

#define MIN_DUTY 1600
#define MAX_DUTY 4400

#define FRAME_LENGTH 17

// Creates a frame with latest data from the accelerometer and writes
// it to buffer.
//
// Buffer must be of length FRAME_LENGTH.
void acc_create_frame(uint8_t* buffer) {
  buffer[0] = 'L';
  buffer[1] = 'D';
  buffer[2] = '-';

  // The following lines are copied from mpu6050_getRawData(...) function.
  uint8_t acc_buffer[14];  // Buffer for gyroscope (8B), temperature (2B) and accelerometer (8B)
  mpu6050_readBytes(MPU6050_RA_ACCEL_XOUT_H, 14, acc_buffer);  // Fill the acc_buffer

  buffer[3] = acc_buffer[0];    // accel X (high)
  buffer[4] = acc_buffer[1];    // accel X (low)
  buffer[5] = acc_buffer[2];    // accel Y (high)
  buffer[6] = acc_buffer[3];    // accel Y (low)
  buffer[7] = acc_buffer[4];    // accel Z (high)
  buffer[8] = acc_buffer[5];    // accel Z (low)
  buffer[9] = acc_buffer[8];    // gyro X (high)
  buffer[10] = acc_buffer[9];   // gyro X (low)
  buffer[11] = acc_buffer[10];  // gyro Y (high)
  buffer[12] = acc_buffer[11];  // gyro Y (low)
  buffer[13] = acc_buffer[12];  // gyro Z (high)
  buffer[14] = acc_buffer[13];  // gyro Z (low)

  buffer[15] = '#';
  buffer[16] = 'S';  // TODO: CNC checksum
}

int main(void) {
  // PWM
  TCCR1A |= (1 << WGM11);                 // Set Fast-PWM mode 1/2
  TCCR1B |= (1 << WGM12) | (1 << WGM13);  // Set Fast-PWM mode 2/2
  TCCR1A |= (1 << COM1A1);                // Set non-inverting PWM mode
  DDRB |= (1 << PB1);                     // Set PORTB1 to be output (we use later for OCR1A pin )
  OCR1A = MIN_DUTY;                       // Set PWM duty
  ICR1 = 39999;  // Set PWM period and prescaler (period = 20ms; prescaler = 8)
  TCCR1B |= (1 << CS11);

  // USART
  UBRR0 = 103;                              // Set USART baudrate to 9600 bps
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);    // Enable USART receiver and transmitter
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);  // Set USART frame to be 8 bits
  UCSR0B |= (1 << RXCIE0);  // Enable interrupt to fire when USART receives data receives data

  // DDRB = Data Direction Register for port B
  // Setting PIN5 on PORTB to 1. 1 means it is an output pin

  // Interrupts
  sei();  // Enable global interrupts

  // LED
  DDRB = DDRB | (1 << PB5);  // Initialize the on-board LED to help with debugging

  // Accelerometer MPU6050
  uint8_t acc_test = mpu6050_testConnection();
  mpu6050_init();

  mpu6050_init();

  uint8_t frame[FRAME_LENGTH];
  while (1) {
    for (int duty = MIN_DUTY; duty <= MAX_DUTY; duty += 5) {
      _delay_ms(25);

      acc_create_frame(frame);                 // Create a frame with accelerometer output
      usart_write_frame(frame, FRAME_LENGTH);  // Write accelerometer output to USART

      OCR1A = duty;  // Set PWM TOP to duty

      if (duty % 50 == 0) {
        PORTB ^= (1 << PB5);  // Blink on-board LED to help with debugging
      }
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
    OCR1A = receivedPWMDuty;  // Set PWM TOP to received PWM duty
  }
}
