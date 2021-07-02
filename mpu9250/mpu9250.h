
/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */

#ifndef avb_mpu9250
#define avb_mpu9250

#include <avr/io.h>
#include <util/delay.h>

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "mpu9250_registers.h"
#include "uart.h"
#include "twi.h"

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);

// Using the MSENSR-9250 breakout board, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 1
#if ADO
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif  

#define AHRS true         // set to false for basic data read
//#define SerialDebug true   // set to true to get Serial output for debugging

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError  M_PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
#define GyroMeasDrift  M_PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define Kp 10.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

// Specify sensor full scale
extern uint8_t Gscale;	// = GFS_250DPS;
extern uint8_t Ascale;	// = AFS_2G;
extern uint8_t Mscale;  // = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
extern uint8_t Mmode;	     // = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

extern float magBias[3];
extern float magScale[3];

extern float magCalibration[3];		// = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
extern float gyroBias[3];		// = {0, 0, 0};
extern float accelBias[3];		// = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output

extern float deltat;	// = 0;
extern float sum; 	//= 0;        // integration interval for both filter schemes

extern float beta;		//= 0.8660254 * GyroMeasError; //sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
extern float zeta;		// = 0.8660254 * GyroMeasDrift; //sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

void mpu9250_setup(void);

/*---------------------------------- Lokale Funktionen -----------------------------------------*/
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

uint8_t readByte_Debug(uint8_t address, uint8_t subAddress);
void readBytes_Debug(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

void magcalMPU9250(float * dest1, float * dest2);
void getMres(void);
void getGres(void);
void getAres(void);
void readAccelData(int16_t * destination);
void getMres(void);
void readMagData(int16_t * destination);
int16_t readTempData(void);
void initAK8963(float * destination);
void readGyroData(int16_t * destination);
void initMPU9250(void);
void MPU9250SelfTest(float * destination);
void calibrateMPU9250(float * dest1, float * dest2);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float * quaternionBuffer);
/*----------------------------------------------------------------------------------------------*/


#endif
