#include <avr/io.h>
#include <util/delay.h>

int main(void) {
  DDRB = DDRB | (1 << PB0);
  // DDRB = Data Direction Register for port B.
  // Setting PIN1 on PORTB to 1. 1 means it is an output pin.

  while (1) {
    _delay_ms(1000);
    PORTB = PORTB ^ (1 << PB0);
  }
}
