/**
 * This program is a proxy between the rover's main computer
 * and the lidar servo.
 *
 * MCU: ATMega328 (Arduino Uno)
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define MIN_DUTY 1600
#define MAX_DUTY 4400

int main(void) {
  TCCR1A |= (1 << WGM11);                 // Set Fast-PWM mode 1/2
  TCCR1B |= (1 << WGM12) | (1 << WGM13);  // Set Fast-PWM mode 2/2
  TCCR1A |= (1 << COM1A1);                // Set non-inverting PWM mode
  DDRB |= (1 << PB1);                     // Set PORTB1 to be output (we use later for OCR1A pin )
  OCR1A = MIN_DUTY;                       // Set PWM duty
  ICR1 = 39999;  // Set PWM period and prescaler (period = 20ms; prescaler = 8)
  TCCR1B |= (1 << CS11);

  UBRR0 = 103;                              // Set USART baudrate to 9600 bps
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);    // Enable USART receiver
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);  // Set USART frame to be 8 bits
  UCSR0B |= (1 << RXCIE0);  // Enable interrupt to fire when USART receives data receives data

  // DDRB = Data Direction Register for port B.
  // Setting PIN5 on PORTB to 1. 1 means it is an output pin.

  sei();  // Enable global interrupts

  while (1) {
    for (int i = MIN_DUTY; i <= MAX_DUTY; i++) {
      _delay_ms(10);
      // OCR1A = i;
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

  // TODO: Fix (only 4LSB bytes are sent) (suggestion: use - instead of +)
  UDR0 = receivedPWMDuty;  // Send back what we got.
}
