#include <avr/io.h>
#include <util/delay.h>

#define MIN_DUTY 1600
#define MAX_DUTY 4400

int main(void) {
  // Set Fast-PWM mode.
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << WGM13);

  // Set non-inverting mode.
  TCCR1A |= (1 << COM1A1);

  // Set DDRB to be output (we use it for OC2 pin for compare registers later).
  DDRB |= (1 << PB1);

  // Set PWM duty.
  OCR1A = MAX_DUTY;

  // Set PWM period and prescaler (period = 20ms; prescaler = 8).
  ICR1 = 39999;
  TCCR1B |= (1 << CS11);

  // DDRB = Data Direction Register for port B.
  // Setting PIN5 on PORTB to 1. 1 means it is an output pin.

  while (1) {
    for (int i = MIN_DUTY; i <= MAX_DUTY; i++) {
      _delay_ms(10);
      OCR1A = i;
    }
  }
}
