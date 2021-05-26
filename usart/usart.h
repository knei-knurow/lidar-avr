#include <stdint.h>

// Waits until the USART transmit buffer is empty, then sends the byte.
void usart_write_byte(uint8_t byte);

// Writes frame byte-by-byte to USART.
void usart_write_frame(uint8_t* frame, unsigned int frame_length);
