/**
 * This program is a proxy between the rover's main computer
 * and the lidar servo and IMU.
 *
 * MCU: ATMega328 (Arduino Uno)
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart/usart.h"

// select MPU (MPU-6050, MPU-9250)
#define MPU_TYPE 9250
#if MPU_TYPE == 6050
#include "mpu6050/mpu6050.h"
#elif MPU_TYPE == 9250
#include "mpu9250/mpu9250.h"
#include "mpu9250/twi.h"
#endif

#define MIN_DUTY 500  // is too low for testing purposed
#define MAX_DUTY 5000
#define START_DUTY ((MAX_DUTY - MIN_DUTY) / 2 + MIN_DUTY)

#define FRAME_SERVO_LEN 8

// Converts float to 4 bytes
void float_to_bytes(uint8_t* buffer, float v) {
  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = v;
  memcpy(buffer, thing.bytes, 4);
}

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

#if MPU_TYPE == 6050
// Creates a frame with latest data from the accelerometer (6DoF) and writes
// it to buffer.
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
  buffer[17] = calculate_checksum(buffer, 17);
}
#endif  // MPU_TYPE == 6050

// Creates a frame with quaternion and writes it to the buffer.
void acc_create_frame_quat(uint8_t* frame_buffer, float* qw, float* qx, float* qy, float* qz) {
  frame_buffer[0] = 'L';
  frame_buffer[1] = 'Q';
  frame_buffer[2] = 16;  // data part length
  frame_buffer[3] = '+';

  float_to_bytes(frame_buffer + 4, *qw);
  float_to_bytes(frame_buffer + 8, *qx);
  float_to_bytes(frame_buffer + 12, *qy);
  float_to_bytes(frame_buffer + 16, *qz);

  frame_buffer[20] = '#';
  frame_buffer[21] = calculate_checksum(frame_buffer, 21);
}

// Create frame which can be ignored and can be uses for testing purposes.
void frame_create_debug(uint8_t* frame_buffer, uint8_t v0, uint8_t v1) {
  frame_buffer[0] = 'L';
  frame_buffer[1] = '?';
  frame_buffer[2] = 2;  // data part length
  frame_buffer[3] = '+';
  frame_buffer[4] = v0;
  frame_buffer[5] = v1;
  frame_buffer[6] = '#';
  frame_buffer[7] = calculate_checksum(frame_buffer, 7);
}

#if MPU_TYPE == 6050
#if MPU6050_GETATTITUDE == 0
// Initializes mpu6050 to grab raw data: accel X Y Z, gyro X Y Z
void init_raw_mpu6050() {
  mpu6050_init();
}

// Grab raw data: accel X Y Z, gyro X Y Z
void read_raw_mpu6050(uint8_t* frame) {
  acc_create_frame(frame);
  usart_write_frame(frame, 18);
}
#elif MPU6050_GETATTITUDE == 2
// Initializes mpu6050 to grab raw DMP data: QW QX QY QZ
// Please DO NOT use this one, I have wasted 10 hours with it.
void init_dmp_mpu6050(uint8_t* frame) {
  mpu6050_init();
  uint8_t dmp_ok = mpu6050_dmpInitialize();
  mpu6050_dmpEnable();

  frame_create_debug(frame, dmp_ok, 0);  // Create debug frame with MPU6050 initialization
  usart_write_frame(frame, 8);           // Send MPU6050 initialization status
}

// Grab raw DMP data: QW QX QY QZ
// Please DO NOT use this one, I have wasted 10 hours with it.
void read_dmp_mpu6050(uint8_t* frame) {
  double qw = 1, qx = 0, qy = 0, qz = 0;
  if (mpu6050_getQuaternionWait(&qw, &qx, &qy, &qz)) {
    acc_create_frame_quat(frame, &qw, &qx, &qy, &qz);
    usart_write_frame(frame, 22);
  }
}
#elif MPU6050_GETATTITUDE == 1
void init_mahony_mpu6050() {
  mpu6050_init();
  _delay_ms(50);
}

void read_mahony_mpu6050(uint8_t* frame) {
  double qw = 1, qx = 0, qy = 0, qz = 0;
  mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
  acc_create_frame_quat(frame, &qw, &qx, &qy, &qz);
  usart_write_frame(frame, 22);

  _delay_ms(10);  // TODO: Remove if possible
}
#endif  // MPU6050_GETATTITUDE
#endif  // MPU_TYPE

#if MPU_TYPE == 9250
void acc9dof_create_frame(uint8_t* buffer, int16_t* accel, int16_t* gyro, int16_t* mag) {
  buffer[0] = 'L';
  buffer[1] = 'M';
  buffer[2] = 18;  // data part length
  buffer[3] = '+';

  buffer[4] = accel[0] >> 8;    // accel X (high)
  buffer[5] = accel[0] & 0xff;  // accel X (low)
  buffer[6] = accel[1] >> 8;    // accel Y (high)
  buffer[7] = accel[1] & 0xff;  // accel Y (low)
  buffer[8] = accel[2] >> 8;    // accel Z (high)
  buffer[9] = accel[2] & 0xff;  // accel Z (low)
  buffer[10] = gyro[0] >> 8;    // gyro X (high)
  buffer[11] = gyro[0] & 0xff;  // gyro X (low)
  buffer[12] = gyro[1] >> 8;    // gyro Y (high)
  buffer[13] = gyro[1] & 0xff;  // gyro Y (low)
  buffer[14] = gyro[2] >> 8;    // gyro Z (high)
  buffer[15] = gyro[2] & 0xff;  // gyro Z (low)
  buffer[16] = mag[0] >> 8;     // mag X (high)
  buffer[17] = mag[0] & 0xff;   // mag X (low)
  buffer[18] = mag[1] >> 8;     // mag Y (high)
  buffer[19] = mag[1] & 0xff;   // mag Y (low)
  buffer[20] = mag[2] >> 8;     // mag Z (high)
  buffer[21] = mag[2] & 0xff;   // mag Z (low)

  buffer[22] = '#';
  buffer[23] = calculate_checksum(buffer, 23);
}

void init_raw_mpu9250(uint8_t* frame) {
  frame_create_debug(frame, 'A', 'A');
  usart_write_frame(frame, 8);

  twi_init();

  frame_create_debug(frame, 'B', 'B');
  usart_write_frame(frame, 8);

  mpu9250_setup();

  frame_create_debug(frame, 'C', 'C');
  usart_write_frame(frame, 8);
}

void read_raw_mpu9250(uint8_t* frame) {
  int16_t accel[3];
  int16_t gyro[3];
  int16_t mag[3];

  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    readAccelData(accel);
    readGyroData(gyro);
    readMagData(mag);

    acc9dof_create_frame(frame, accel, gyro, mag);
    usart_write_frame(frame, 24);
  }
}

#endif  // MPU_TYPE

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
    if (pwm >= MIN_DUTY && pwm <= MAX_DUTY) {
      OCR1A = pwm;  // Set PWM TOP to received PWM duty
    }
    frame_ready = 0;
  }
}
