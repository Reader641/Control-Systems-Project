/*  Arduino Based RC Gryo Trim Tab PID control system
    A closed loop PID ccontrol system using roll readings from MPU6050 Gyro/Accel
    to actuate servos linked to trim tabs on the back of the boat.
    Keeps the boat from rocking, corrects listing.

    MPU6050 library
    mpu6050/examples/MPU6050_DMP6_using_DMP_V6v12
*/

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

//Including libraries
#include "Packets.h"
#include <Servo.h>
#include <PID_v2.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"

//Channels
#define TRIM_TAB_LEFT 5
#define TRIM_TAB_RIGHT 6

//ACK payload to transmit
IMUDataPacket imuData;

//MPU6050
MPU6050 mpu;

//Servos
Servo trimTabLeftServo;
Servo trimTabRightServo;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//Velocity Calculation
float velocity[2] = {0, 0};
float currAccel[2];
float prevAccel[2] = {0, 0};
unsigned long prevTime = millis();
unsigned long currTime;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

unsigned int trimTabLeftValue;
unsigned int trimTabRightValue;

// Specify the links and initial tuning parameters
double Kp = 2.145, Ki = 0, Kd = 0;
PID_v2 myPIDLeft(Kp, Ki, Kd, PID::Direct);
PID_v2 myPIDRight(Kp, Ki, Kd, PID::Direct);


void setup() 
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(115200);

  Serial.print("Help");

  // initialize device
  mpu.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // load and configure the DMP
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println("DMP ready!");
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    // read the FIFO Buffer
    readFifoBuffer();
    currTime = millis();

    // Yaw, Pitch & Roll angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ypr[0] *= 180 / M_PI;
    ypr[1] *= 180 / M_PI;
    ypr[2] *= 180 / M_PI;
  } 
  else 
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }  

  //Attach channels
  trimTabLeftServo.attach(TRIM_TAB_LEFT);
  trimTabRightServo.attach(TRIM_TAB_RIGHT);

  myPIDLeft.Start(ypr[2],  // input
  50,                      // current output
              0);                   // setpoint

  myPIDRight.Start(ypr[2],  // input
  130,                      // current output
              0);                   // setpoint
  myPIDRight.SetOutputLimits(170, 170);
  myPIDLeft.SetOutputLimits(10, 100);
  myPIDLeft.SetControllerDirection(REVERSE);
  resetTrimTabs();
}         

void loop() 
{

  // if programming failed, don't try to do anything
  if (!dmpReady) 
  {
    Serial.println("DMP not ready");
    return;
  }

  // read the FIFO Buffer
  readFifoBuffer();
  currTime = millis();

  // Yaw, Pitch & Roll angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
  ypr[0] *= 180 / M_PI;
  ypr[1] *= 180 / M_PI;
  ypr[2] *= 180 / M_PI;

  imuData.ypr[0] = ypr[0];
  imuData.ypr[1] = ypr[1];
  imuData.ypr[2] = ypr[2];
  imuDataDisplay();
  controlTrimTabs();
}


void readFifoBuffer() 
{
  // Clear the buffer so as we can get fresh values
  // The sensor is running a lot faster than our sample period
  mpu.resetFIFO();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
}

void controlTrimTabs()
{
  trimTabLeftValue = map(myPIDLeft.Run(ypr[2]), 0, 180,  100, 10);
  trimTabRightValue = map(myPIDRight.Run(ypr[2]), 0, 180, 80, 170);
  trimTabLeftServo.write(trimTabLeftValue); //Flat = 50, Lower = 100, Upper = 10
  trimTabRightServo.write(trimTabRightValue); // Flat 130, Lower = 80, Upper = 170
  // resetTrimTabs(); //Comment out the above 4 lines and uncomment this one to turn off PID control
}

void resetTrimTabs()
{
  trimTabLeftServo.write(50); //Flat = 50, Lower = 100, Upper = 10
  trimTabRightServo.write(130); // Flat 130, Lower = 80, Upper = 170
}

/**/ //Uncomment to allow print out
void imuDataDisplay()
{
  // Serial.println("IMU Data");
  // Serial.print("Yaw -  ");
  // Serial.println(imuData.ypr[0]);
  // Serial.print("Pitch -  ");
  // Serial.println(imuData.ypr[1]);
  Serial.print(imuData.time);
  Serial.print( ", ");
  Serial.println(imuData.ypr[2]);

}
/**/
