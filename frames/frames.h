#include <stdint.h>
#include "../lidar-avr.h"

// Converts float to 4 bytes
void float_to_bytes(uint8_t* buffer, float v);

// Calculates checksum
uint8_t calculate_checksum(uint8_t* buffer, unsigned size);

// Creates a frame with quaternion and writes it to the buffer.
void acc_create_frame_quat(uint8_t* frame_buffer, float* qw, float* qx, float* qy, float* qz);

// Create frame which can be ignored and can be uses for testing purposes.
void frame_create_debug(uint8_t* frame_buffer, uint8_t v0, uint8_t v1);

#if MPU_TYPE == 6050
// Creates a frame with latest data from the accelerometer (6DoF) and writes
// it to buffer.
void acc_create_frame(uint8_t* buffer);
#endif  // MPU_TYPE == 6050

#if MPU_TYPE == 9250
void acc9dof_create_frame(uint8_t* buffer, int16_t* accel, int16_t* gyro, int16_t* mag);
#endif  // MPU_TYPE == 6050