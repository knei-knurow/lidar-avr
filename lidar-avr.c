#include "lidar-avr.h"

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
void init_raw_mpu9250(uint8_t* frame) {
  frame_create_debug(frame, 'A', '0');
  usart_write_frame(frame, 8);

  twi_init();

  frame_create_debug(frame, 'B', '0');
  usart_write_frame(frame, 8);

  mpu9250_setup(frame);
}

void read_raw_mpu9250(uint8_t* frame) {
  // raw data
  int16_t accel[3];
  int16_t gyro[3];
  int16_t mag[3];

  // scaled and biased data
  float accelf[3];
  float gyrof[3];
  float magf[3];

  // scales
  float accel_scale, gyro_scale, mag_scale;
  getAres(&accel_scale);
  getGres(&gyro_scale);
  getMres(&mag_scale);

  // biases
  float accel_bias[3], gyro_bias[3], mag_bias[3];
  getAccelBias(accel_bias, accel_bias + 1, accel_bias + 2);
  getGyroBias(gyro_bias, gyro_bias + 1, gyro_bias + 2);
  getMagBias(mag_bias, mag_bias + 1, mag_bias + 2);

  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    readAccelData(accel);
    accelf[0] = (float)accel[0] * accel_scale + accel_bias[0];
    accelf[1] = (float)accel[1] * accel_scale + accel_bias[1];
    accelf[2] = (float)accel[2] * accel_scale + accel_bias[2];

    readGyroData(gyro);
    gyrof[0] = (float)gyro[0] * gyro_scale + gyro_bias[0];
    gyrof[1] = (float)gyro[1] * gyro_scale + gyro_bias[1];
    gyrof[2] = (float)gyro[2] * gyro_scale + gyro_bias[2];

    readMagData(mag);
    magf[0] = (float)mag[0] * mag_scale + mag_bias[0];
    magf[1] = (float)mag[1] * mag_scale + mag_bias[1];
    magf[2] = (float)mag[2] * mag_scale + mag_bias[2];

    acc9dof_create_frame_float(frame, accelf, gyrof, magf);
    usart_write_frame(frame, 42);
  }
}
#endif  // MPU_TYPE

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
