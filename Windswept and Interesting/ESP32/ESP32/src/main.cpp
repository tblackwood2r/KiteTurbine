//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Version: Beta 1.2

/*
 * 
 * If you are using this for an academic or scholarly project, please credit me in any presentations or publications:
 *
 * Nicholas Rehm
 * Department of Aerospace Engineering
 * University of Maryland
 * College Park 20742
 * Email: nrehm@umd.edu
 *
 */
 
//========================================================================================================================//

//CREDITS + SPECIAL THANKS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Skeleton code for reading and initializing MPU6050 borrowed from:
https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS

MPU9250 implementation based on MPU9250 library by
brian.taylor@bolderflight.com
http://www.bolderflight.com


Thank you to:

RcGroups 'jihlein' - IMU implementation overhaul + SBUS implementation

*/

//========================================================================================================================//

//Rods Diasy Kite Turbine changes
/*
This is going to be an ESP32 with an MPU9250 (was MPU6050) placed inside the fuselage of a Daisy kite turbine blade to assess the exact position of rotation asap for control of the PTO
Comms are going to be over wifi UDP 
*/
/*
 *  This bit of sketch initiates wifi to send data over UDP on a ESP32 device
 *
 */

//REQUIRED LIBRARIES (included with download in main sketch folder)
#include <Arduino.h>

#include <WiFi.h>
#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication

#include <WiFiMulti.h>
#include <InfluxDbClient.h>

// WiFi network name and password:
//const char * networkName = "BTHub6-3JFG";
//const char * networkPswd = "YwVU63rdLiTP";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.1.132"; // Set this to the IP of the recieving computer so the board knows who to look at
const int udpPort = 3333;

//Are we currently connected?
boolean connected = false;

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//Uncomment only one IMU
//#define USE_MPU6050_I2C //default
#define USE_MPU9250_SPI  //MPU6050 worked great but This time going for speedy data 

//Uncomment only one full scale gyro range (deg/sec)
//#define GYRO_250DPS //default
//#define GYRO_500DPS
//#define GYRO_1000DPS 
#define GYRO_2000DPS // rod changed up from default 250DPS because kite blade can spin at > 1000DPS

//Uncomment only one full scale accelerometer range (G's)
//#define ACCEL_2G //default
//#define ACCEL_4G
//#define ACCEL_8G
#define ACCEL_16G  //rod changed up to this 16G max blades may be going about 10g with radius ~2.5m at 180rpm

//========================================================================================================================//

// --------------------- Tom Additions ------------------------------

WiFiMulti wifiMulti;

#define DEVICE "ESP32"

// // --------------- Tom's House ---------------
// #define WIFI_SSID "BTHub6-3JFG"  // WiFi AP SSID
// #define WIFI_PASSWORD "YwVU63rdLiTP"  // WiFi password
// // InfluxDB  server url. Don't use localhost, always server name or ip address.
// // E.g. http://192.168.1.48:8086 (In InfluxDB 2 UI -> Load Data -> Client Libraries), 
// #define INFLUXDB_URL "http://192.168.1.132:8086"

// // --------------- Rod's House ---------------
// #define WIFI_SSID "BTHub6-SQ53"
// #define WIFI_PASSWORD "7aVYUX9aWQwG"
// #define INFLUXDB_URL "http://192.168.1.218:8086"

// // --------------- Rod's Phone ---------------
// #define WIFI_SSID "roddysphone"
// #define WIFI_PASSWORD "caferod1"
// #define INFLUXDB_URL "http://192.168.187.83:8086"

// --------------- Office's Wifi ---------------
#define WIFI_SSID "JLP Internet"
#define WIFI_PASSWORD "shetl4ndwebsdesign"
#define INFLUXDB_URL "http://192.168.40.2:8086"

// InfluxDB 2 server or cloud API authentication token (Use: InfluxDB UI -> Load Data -> Tokens -> <select token>)
#define INFLUXDB_TOKEN "my-token"

// InfluxDB 2 organization id (Use: InfluxDB UI -> Settings -> Profile -> <name under tile> )
#define INFLUXDB_ORG "my-org"

// InfluxDB 2 bucket name (Use: InfluxDB UI -> Load Data -> Buckets)
#define INFLUXDB_BUCKET "my-bucket"

// InfluxDB client instance
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
// InfluxDB client instance for InfluxDB 1
//InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DB_NAME);

// Data point
Point sensor("wifi_status");

// ------------------------------------------------------------------

#if defined USE_MPU6050_I2C
  #include <MPU6050.h>
  MPU6050 mpu6050;
#elif defined USE_MPU9250_SPI
  #include <MPU9250.h>
 MPU9250 mpu9250(SPI,14);  // Line used to read     MPU9250 mpu9250(SPI2,36); but that threw up exceptions on compiling
#else
  #error No MPU defined... 
#endif

//The udp library class
WiFiUDP udp;

//========================================================================================================================//

//Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
  #define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
  #define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
  #define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS                                                        
  #define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
  #define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
  #define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
  #define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
  #define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G
#endif
  
#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.2;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2) going for 9250 this time - thanks for comments here
float B_gyro = 0.17;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)  going for 9250 this time - thanks for comments here
float B_mag = 1.0;        //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these  --  going for 9250 this time - thanks for comments here
float MagErrorX = -7.16;
float MagErrorY = 48.03;
float MagErrorZ = 20.09;
float MagScaleX = 0.99;
float MagScaleY = 1.02;
float MagScaleZ = 0.99;

//Controller parameters (take note of defaults before modifying!): Just deleted them all for this project

//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          

//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup    // That's pin 2 GPIO 2 on ESP32

//========================================================================================================================//

//DECLARE GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float q0 = 1.0f; //initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//

//HELPER FUNCTIONS

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void IMUinit() {
  //DESCRIPTION: Initialize IMU
  /*
   * Don't worry about how this works
   */
  #if defined USE_MPU6050_I2C
    Wire.begin();
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
    
    mpu6050.initialize();
    
    if (mpu6050.testConnection() == false) {
      Serial.println("MPU6050 initialization unsuccessful");
      Serial.println("Check MPU6050 wiring or try cycling power");
      while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
    
  #elif defined USE_MPU9250_SPI
    int status = mpu9250.begin();    

    if (status < 0) {
      Serial.println("MPU9250 initialization unsuccessful");
      Serial.println("Check MPU9250 wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu9250.setGyroRange(GYRO_SCALE);
    mpu9250.setAccelRange(ACCEL_SCALE);
    mpu9250.setMagCalX(MagErrorX, MagScaleX);
    mpu9250.setMagCalY(MagErrorY, MagScaleY);
    mpu9250.setMagCalZ(MagErrorZ, MagScaleZ);
    mpu9250.setSrd(0); //sets gyro and accel read to 1khz, magnetometer read to 100hz
  #endif
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;

  #if defined USE_MPU6050_I2C
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  #elif defined USE_MPU9250_SPI
    mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
  #endif

 //Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

  //Magnetometer
  MagX = MgX/6.0; //uT
  MagY = MgY/6.0;
  MagZ = MgZ/6.0;
  //Correct the outputs with the calculated error values
  MagX = (MagX - MagErrorX)*MagScaleX;
  MagY = (MagY - MagErrorY)*MagScaleY;
  MagZ = (MagZ - MagErrorZ)*MagScaleZ;
  //LP filter magnetometer data
  MagX = (1.0 - B_mag)*MagX_prev + B_mag*MagX;
  MagY = (1.0 - B_mag)*MagY_prev + B_mag*MagY;
  MagZ = (1.0 - B_mag)*MagZ_prev + B_mag*MagZ;
  MagX_prev = MagX;
  MagY_prev = MagY;
  MagZ_prev = MagZ;
}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    #if defined USE_MPU6050_I2C
      mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    #elif defined USE_MPU9250_SPI
      mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
    #endif
    
    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //compute angles
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float mholder;

  //use 6DOF algorithm if MPU6050 is being used
  #if defined USE_MPU6050_I2C 
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  #endif
  
  //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  }

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    //Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    //Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  //compute angles - NWU
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void calibrateAttitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
   * to boot. 
   */
  //Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
}

void calibrateMagnetometer() {
  #if defined USE_MPU9250_SPI 
    float success;
    Serial.println("Beginning magnetometer calibration in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Rotate the IMU about all axes until complete.");
    Serial.println(" ");
    success = mpu9250.calibrateMag();
    if(success) {
      Serial.println("Calibration Successful!");
      Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
      Serial.print("float MagErrorX = ");
      Serial.print(mpu9250.getMagBiasX_uT());
      Serial.println(";");
      Serial.print("float MagErrorY = ");
      Serial.print(mpu9250.getMagBiasY_uT());
      Serial.println(";");
      Serial.print("float MagErrorZ = ");
      Serial.print(mpu9250.getMagBiasZ_uT());
      Serial.println(";");
      Serial.print("float MagScaleX = ");
      Serial.print(mpu9250.getMagScaleFactorX());
      Serial.println(";");
      Serial.print("float MagScaleY = ");
      Serial.print(mpu9250.getMagScaleFactorY());
      Serial.println(";");
      Serial.print("float MagScaleZ = ");
      Serial.print(mpu9250.getMagScaleFactorZ());
      Serial.println(";");
      Serial.println(" ");
      Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
    }
    else {
      Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
    }
  
    while(1); //halt code so it won't enter main loop until this function commented out
  #endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
  while(1); //halt code so it won't enter main loop until this function commented out
}

void loopBlink() {
  //DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(2, blinkAlternate); //pin 13 is built in LED // We've moved to ESP32 so it's pin 2
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired   now on pin 2 esp32
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(2, LOW);
    delay(downTime);
    digitalWrite(2, HIGH);
    delay(upTime);
  }
}

void printGyroData() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX: "));
    Serial.print(GyroX);
    Serial.print(F(" GyroY: "));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ: "));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX: "));
    Serial.print(AccX);
    Serial.print(F(" AccY: "));
    Serial.print(AccY);
    Serial.print(F(" AccZ: "));
    Serial.println(AccZ);
  }
}

void printMagData() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("MagX: "));
    Serial.print(MagX);
    Serial.print(F(" MagY: "));
    Serial.print(MagY);
    Serial.print(F(" MagZ: "));
    Serial.println(MagZ);
  }
}

void printRollPitchYaw() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll: "));
    Serial.print(roll_IMU);
   Serial.print(F(" pitch: "));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw: "));
    Serial.println(yaw_IMU);
  }
}

void printLoopRate() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt = "));
    Serial.println(dt*1000000.0);
  }
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//

void printToSerial(){
  Serial.println(String(roll_IMU) + "," + String(pitch_IMU) + "," + String(yaw_IMU) + "," + String(AccX) + "," + String(AccY) + "," + String(AccZ));
}

class MyProtocol{
  public:
  MyProtocol(Stream& s):serial(s){}
  void send(){
    Serial.println(String(roll_IMU, 2) + "," + String(pitch_IMU) + "," + String(yaw_IMU) + "," + String(AccX) + "," + String(AccY) + "," + String(AccZ));
  }
  
  // void sendInflux(){
  //   // Store measured value into point%.2f
  //   sensor.clearFields();
    
  //   // Report RSSI of currently connected network as per original example
  //   sensor.addField("rssi", WiFi.RSSI());
    
  //   // Report Acceleration on X, Y, and Z axis
  //   sensor.addField("accX", AccX);
  //   sensor.addField("accY", AccY);
  //   sensor.addField("accZ", AccZ);

  //   // Report the roll, pitch, and yaw of device
  //   sensor.addField("roll", roll_IMU);
  //   sensor.addField("pitch", pitch_IMU);
  //   sensor.addField("yaw", yaw_IMU);

  //   //   Print what are we exactly writing
  //   Serial.print("Writing: ");
  //   Serial.println(client.pointToLineProtocol(sensor));
    
  //   // If no Wifi signal, try to reconnect it
  //   if (wifiMulti.run() != WL_CONNECTED) {
  //     Serial.println("Wifi connection lost");
  //   }
  //   // Write point
  //   if (!client.writePoint(sensor)) {
  //     Serial.print("InfluxDB write failed: ");
  //     Serial.println(client.getLastErrorMessage());
  //   }
  // }
  private:
    Stream& serial;
};

MyProtocol printDat(Serial);

void writeSensorVals(){
  // Store measured value into point%.2f
  sensor.clearFields();
  
  // Report RSSI of currently connected network as per original example
  sensor.addField("rssi", WiFi.RSSI());
  
  // Report Acceleration on X, Y, and Z axis
  sensor.addField("accX", AccX);
  sensor.addField("accY", AccY);
  sensor.addField("accZ", AccZ);

  // Report the roll, pitch, and yaw of device
  sensor.addField("roll", roll_IMU);
  sensor.addField("pitch", pitch_IMU);
  sensor.addField("yaw", yaw_IMU);

  //   Print what are we exactly writing
  Serial.print("Writing: ");
  Serial.println(client.pointToLineProtocol(sensor));
  
  // If no Wifi signal, try to reconnect it
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("Wifi connection lost");
  }
  // Write point
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
}

void setupWiFi(){
  // Connect WiFi
  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  
  // Add constant tags - only once
  sensor.addTag("device", DEVICE);
  sensor.addTag("SSID", WiFi.SSID());

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
}
//=========================================================================================//

//Controller:  don't need those variables

//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
//  Serial.begin(500000); //usb serial
  Serial.begin(115200);
  // Serial.begin(9600);
  delay(3000); //3 second delay for plugging in battery before IMU calibration begins, feel free to comment this out to reduce boot time
//
  //Initialize all pins
  pinMode(2, OUTPUT); //pin 13 LED blinker on board, do not modify // rod did modify this ... it's pin 2 GPIO2 on an esp32

  //Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration
  digitalWrite(2, HIGH);

  delay(10);

   //Indicate here with 3 quick blinks
  setupBlink(9,160,70); //numBlinks, upTime (ms), downTime (ms)

   //Initialize IMU communication
  IMUinit();

  delay(10);

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level
  calculate_IMU_error();

  delay(100);

  //Warm up the loop
  calibrateAttitude(); //helps to warm up IMU and Madgwick filter before finally entering main loop
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

  //If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  //calibrateMagnetometer(); //generates magentometer error and scale factors

// --------------------- Tom Additions ------------------------------ 
  // setupWiFi();
// ------------------------------------------------------------------
}

//========================================================================================================================//
//                                                       MAIN LOOP                                                        //                           
//========================================================================================================================//
                                                  
void loop() {
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  //Get vehicle state
  getIMUdata(); //pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)

  // --------------------- Tom Additions ------------------------------ 
  // printToSerial();
  printDat.send();
  // writeSensorVals();
  // ------------------------------------------------------------------
  //Regulate loop rate
  loopRate(2000); //do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}




//=========================================================================================//

