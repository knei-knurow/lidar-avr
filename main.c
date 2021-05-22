#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define MIN_DUTY 1600
#define MAX_DUTY 4400

int main(void) {
  // Timer1 PWM servo HS332HD
  TCCR1A |= (1 << WGM11);  // Set Fast-PWM mode.
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  TCCR1A |= (1 << COM1A1);  // Set non-inverting mode.
  DDRB |= (1 << PB1);       // Set DDRB to be output (we use it for OC2 pin for
                            // compare registers later).
  OCR1A = MIN_DUTY;         // Set PWM duty.
  ICR1 = 39999;  // Set PWM period and prescaler (period = 20ms; prescaler = 8).
  TCCR1B |= (1 << CS11);

  // USART
  UBRR0 = 103;                              // Baudrate 9600 bps
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);    // Enable reciever
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);  // 8 bits of data
  UCSR0B |= (1 << RXCIE0);                  // Interrupt recieve

  // DDRB = Data Direction Register for port B.
  // Setting PIN5 on PORTB to 1. 1 means it is an output pin.

  sei();  // Enable global interrupts

  while (1) {
    for (int i = MIN_DUTY; i <= MAX_DUTY; i++) {
      _delay_ms(10);
      OCR1A = i;
    }
  }
}

ISR(USART_RX_vect) {
  uint8_t inputValue = UDR0;
  uint16_t calculated = (inputValue * (MAX_DUTY - MIN_DUTY)) / 255 + MIN_DUTY;

  // TODO: Fix (only 4LSB bytes are sent)
  OCR1A = calculated;  // Set TOP to calculated PWM duty.
  UDR0 = calculated;   // Send back the calculated PWM duty.
}
