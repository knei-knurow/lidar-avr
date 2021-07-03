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

#if MPU_TYPE == 6050
#if MPU6050_GETATTITUDE == 0
void init_raw_mpu6050();
void read_raw_mpu6050(uint8_t* frame);
#elif MPU6050_GETATTITUDE == 2
void init_dmp_mpu6050(uint8_t* frame);
void read_dmp_mpu6050(uint8_t* frame);
#elif MPU6050_GETATTITUDE == 1
void init_mahony_mpu6050();
void read_mahony_mpu6050(uint8_t* frame);
#endif  // MPU6050_GETATTITUDE
#endif  // PU_TYPE == 6050

#if MPU_TYPE == 9250
void init_raw_mpu9250(uint8_t* frame);
void read_raw_mpu9250(uint8_t* frame);
#endif  // MPU_TYPE == 9250