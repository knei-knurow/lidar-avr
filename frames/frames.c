#include "frames.h"

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
#endif  // MPU_TYPE == 9250