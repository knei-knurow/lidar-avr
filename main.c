/**
 * This program is a proxy between the rover's main computer
 * and the lidar servo and IMU.
 *
 * MCU: ATMega328 (Arduino Uno)
 */
#include "frames/frames.h"
#include "lidar-avr.h"

int main(void) {
  // PWM
  TCCR1A |= (1 << WGM11);                 // Set Fast-PWM mode 1/2
  TCCR1B |= (1 << WGM12) | (1 << WGM13);  // Set Fast-PWM mode 2/2
  TCCR1A |= (1 << COM1A1);                // Set non-inverting PWM mode
  DDRB |= (1 << PB1);                     // Set PORTB1 to be output (we use later for OCR1A pin )
  OCR1A = START_DUTY;                     // Set PWM duty
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
  DDRB |= (1 << PB5);  // Initialize the on-board LED to help with debugging
  int led_count = 0;

  // Data frame
  uint8_t frame[64];

// Accelerometer MPU6050
#if MPU_TYPE == 6050
#if MPU6050_GETATTITUDE == 0
  init_raw_mpu6050();
#elif MPU6050_GETATTITUDE == 1
  init_mahony_mpu6050();
#elif MPU6050_GETATTITUDE == 2
  init_dmp_mpu6050(frame);
#endif  // MPU6050_GETATTITUDE
#elif MPU_TYPE == 9250
  init_raw_mpu9250(frame);
#endif  // MPU_TYPE

  while (1) {
#if MPU_TYPE == 6050
    // Read MPU6050
#if MPU6050_GETATTITUDE == 0
    read_raw_mpu6050(frame);  // requires about 10 ms
#elif MPU6050_GETATTITUDE == 1
    read_mahony_mpu6050(frame);
#elif MPU6050_GETATTITUDE == 2
    read_dmp_mpu6050(frame);
#endif  // MPU6050_GETATTITUDE
#elif MPU_TYPE == 9250
    read_raw_mpu9250(frame);
#endif  // MPU_TYPE

    // Update LED
    if (led_count++ % 20 == 0) {
      PORTB ^= (1 << PB5);
    }
  }
}