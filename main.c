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

#define MIN_DUTY 1000  // is too low for testing purposed
#define MAX_DUTY 4400

#define FRAME_LENGTH 18
#define FRAME_SERVO_LEN 8

// Calculates checksum
uint8_t calculate_checksum(uint8_t* buffer, unsigned size) {
  if (size == 0)
    return 0;
  uint8_t checksum = buffer[0];
  for (unsigned i = 1; i < size; i++) {
    checksum ^= buffer[i];
  }
  return checksum;
}

// Creates a frame with latest data from the accelerometer and writes
// it to buffer.
//
// Buffer must be of length FRAME_LENGTH.
void acc_create_frame(uint8_t* buffer) {
  buffer[0] = 'L';
  buffer[1] = 'D';
  buffer[2] = 12;  // data part length
  buffer[3] = '+';

  // The following lines are copied from mpu6050_getRawData(...) function.
  uint8_t acc_buffer[14];  // Buffer for gyroscope (8B), temperature (2B) and accelerometer (8B)
  mpu6050_readBytes(MPU6050_RA_ACCEL_XOUT_H, 14, acc_buffer);  // Fill the acc_buffer

  buffer[4] = acc_buffer[0];    // accel X (high)
  buffer[5] = acc_buffer[1];    // accel X (low)
  buffer[6] = acc_buffer[2];    // accel Y (high)
  buffer[7] = acc_buffer[3];    // accel Y (low)
  buffer[8] = acc_buffer[4];    // accel Z (high)
  buffer[9] = acc_buffer[5];    // accel Z (low)
  buffer[10] = acc_buffer[8];   // gyro X (high)
  buffer[11] = acc_buffer[9];   // gyro X (low)
  buffer[12] = acc_buffer[10];  // gyro Y (high)
  buffer[13] = acc_buffer[11];  // gyro Y (low)
  buffer[14] = acc_buffer[12];  // gyro Z (high)
  buffer[15] = acc_buffer[13];  // gyro Z (low)

  buffer[16] = '#';
  buffer[17] = calculate_checksum(buffer, FRAME_LENGTH - 1);
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
  UBRR0 = 51;                               // Set USART baudrate to 19200 bps
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

  int led_count = 0;
  uint8_t frame[FRAME_LENGTH];
  while (1) {
    // acc_create_frame(frame);                 // Create a frame with accelerometer output
    // usart_write_frame(frame, FRAME_LENGTH);  // Write accelerometer output to USART

    if (led_count++ % 50 == 0) {
      PORTB ^= (1 << PB5);
    }
  }
}

ISR(USART_RX_vect) {
  static uint8_t byte_number = 0;
  static uint8_t crc = 0;
  static uint8_t frame[FRAME_SERVO_LEN];
  static uint8_t frame_ready = 0;

  uint8_t input = UDR0;

  if (byte_number == 0 && input == 'L') {
    crc = input;
  } else if ((byte_number == 1 && input == 'D') || (byte_number == 2 && input == 2) ||
             (byte_number == 3 && input == '+') || (byte_number == 4) || (byte_number == 5) ||
             (byte_number == 6 && input == '#')) {
    crc ^= input;
  } else if (byte_number == 7) {
    frame_ready = 1;
  } else {
    byte_number = 0;
    frame_ready = 0;
    // bad frame
  }

  frame[byte_number] = input;
  byte_number++;

  if (frame_ready && crc == frame[FRAME_SERVO_LEN - 1]) {
    byte_number = 0;
    frame_ready = 0;

    uint16_t pwm = (frame[4] << 8) + frame[5];

    usart_write_byte(pwm / 100);

    if (pwm >= MIN_DUTY && pwm <= MAX_DUTY) {
      OCR1A = pwm;  // Set PWM TOP to received PWM duty
    }

    frame_ready = 0;
  }
}
