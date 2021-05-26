#include "usart.h"
#include <avr/io.h>

void usart_write_byte(uint8_t byte) {
  // Wait until transmit buffer is empty
  while (!(UCSR0A & (1 << UDRE0))) {
  };

  UDR0 = byte;
}

void usart_write_frame(uint8_t* frame, unsigned int frame_length) {
  for (int i = 0; i < frame_length; i++) {
    usart_write_byte(frame[i]);
  }
}
