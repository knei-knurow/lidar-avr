#include <avr/io.h>
#include <util/delay.h>

int main(void) {
  DDRB |= (1 << PB5);

  // Set Fast-PWM mode.
  TCCR2A |= (1 << WGM20);
  TCCR2A |= (1 << WGM21);

  // Set inverting mode.
  TCCR2A |= (1 << COM2A1);

  // Set DDRB to be output (we use it for OC2 pin for compare registers later).
  DDRB |= (1 << PB3);

  // Set PWM duty.
  OCR2A = 127;

  // Enable prescaler and set to 1 (no prescaling).
  TCCR2B |= (1 << CS21);

  // DDRB = Data Direction Register for port B.
  // Setting PIN5 on PORTB to 1. 1 means it is an output pin.

  while (1) {
    _delay_ms(1000);

    PORTB ^= (1 << PB5);
  }
}
