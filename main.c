#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define MIN_DUTY 1600
#define MAX_DUTY 4400

int main(void) {
  // Timer1 PWM servo HS332HD
  TCCR1A |= (1 << WGM11); // Set Fast-PWM mode.
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  TCCR1A |= (1 << COM1A1); // Set non-inverting mode.
  DDRB |= (1 << PB1);      // Set PORTB1 to be output (we use later for OCR1A pin ).
  OCR1A = MIN_DUTY;        // Set PWM duty.
  ICR1 = 39999;            // Set PWM period and prescaler (period = 20ms; prescaler = 8).
  TCCR1B |= (1 << CS11);

  // USART
  UBRR0 = 103;                             // Baudrate 9600 bps
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);   // Enable reciever
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01); // 8 bits of data
  UCSR0B |= (1 << RXCIE0);                 // Interrupt recieve

  // DDRB = Data Direction Register for port B.
  // Setting PIN5 on PORTB to 1. 1 means it is an output pin.

  sei(); // Enable global interrupts

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
      // handle error gracefully
      byteNumber = 0;
      break;
    }
    break;
  case 1:
    if (input == 'D') {
      crc ^= input;

      byteNumber++;
    } else {
      // handle error gracefully
      byteNumber = 0;
    }
    break;
  case 2:
    if (input == '+') {
      crc ^= input;

      byteNumber++;
    } else {
      // handle error gracefully
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
    }
    break;
  }

  uint16_t receivedPWMDuty = (value8MSB << 8) + value8LSB;

  uint16_t calculatedPWMDuty = (input * (MAX_DUTY - MIN_DUTY)) / 255 + MIN_DUTY;

  if (receivedPWMDuty >= MIN_DUTY && receivedPWMDuty <= MAX_DUTY) {
    OCR1A = receivedPWMDuty; // Set TOP to calculated PWM duty.
  }

  // TODO: Fix (only 4LSB bytes are sent) (suggestion: use - instead of +)
  UDR0 = receivedPWMDuty; // Send back what we got.
}
