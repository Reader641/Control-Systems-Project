/*  Arduino Based RC Receiver with Trim Tab PID control system
    MPU6050 3-Axis Gyroscope & 3-Axis Accelerometer
    Gets IMU Data by polling every loop
    IMU Data packets are sent back to TX using ACK payloads
    Mode 1 - Adjust setpoints for PID control system
    Mode 2 - Adjust pitch control system parameters
    Mode 3 - Adjust roll control system parameters
    Mode 4 - Adjust trimtabs postion manually
    Failsafe mode hard coded

    Examples used -
    RF24 library
    examples/AcknowledgementPayloads/AcknowledgementPayloads.ino

    MPU6050 library
    mpu6050/examples/MPU6050_DMP6_using_DMP_V6v12
*/

/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
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
#include <EEPROM.h>
#include <RF24.h>
#include <Servo.h>
#include <ESC.h>
#include <PID_v2.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"

//Channels
#define THROTTLE 9
#define STEER 6
#define TRIM_TAB_LEFT 3
#define TRIM_TAB_RIGHT 5

RF24 radio(8,10); //nRF24L01 (CE = 8, CSN = 10)

byte address[][6] = {"TX_01", "RX_01"};

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

//Data packets to receive
ControlPacket controlData;

//ACK payload to transmit
IMUDataPacket imuData;

//MPU6050
MPU6050 mpu;

//Servos
Servo rudderServo;
Servo trimTabLeftServo;
Servo trimTabRightServo;
int Offset = 40;

//ESC
ESC motorESC (THROTTLE, 1000, 2000, 1500);   //Foward and backward control

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

PID_v2 pitchPID(0, 0, 0, PID::Direct);
// PID_v2 rollPID(0, 0, 0, PID::Direct);

unsigned int throttleValue;
unsigned int steerValue;
unsigned int trimTabLeftValue;
unsigned int trimTabRightValue;


void setup() 
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(115200);

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

  // if (!radio.begin())
  // {
  //   Serial.println("Radio is not responding");
  // }
  // radio.setPALevel(RF24_PA_MAX, 1);
  // radio.enableDynamicPayloads();
  // radio.enableAckPayload();
  // radio.openWritingPipe(address[1]);
  // radio.openReadingPipe(1, address[0]);
  // radio.setDataRate(RF24_250KBPS);
  // radio.startListening();                     //Start receiving
  resetData();                                //Set control data to defaults

  //Attach channels
  rudderServo.attach(STEER);
  trimTabLeftServo.attach(TRIM_TAB_LEFT);
  trimTabRightServo.attach(TRIM_TAB_RIGHT);
  motorESC.arm();

  // rollPID.Start(ypr[2], 127, controlData.setPointData.rollSetPoint);
  pitchPID.Start(ypr[1], 90, controlData.setPointData.pitchSetPoint);
  pitchPID.SetOutputLimits(50, 110);
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

  // initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  currAccel[0] = aaWorld.x;         //Accelration in the x axis
  currAccel[1] = aaWorld.y;         //Accelration in the y axis
  velocity[0] += (currAccel[0] + prevAccel[0]) / 2 * (currTime - prevTime) / 1000; //1000 added to get the same units
  velocity[1] += (currAccel[1] + prevAccel[1]) / 2 * (currTime - prevTime) / 1000;
  prevAccel[0] = currAccel[0];
  prevAccel[1] = currAccel[1];
  
  imuData.velocity[0] = velocity[0];
  imuData.velocity[1] = velocity[1];
  imuData.acceleration[0] = aaWorld.x;
  imuData.acceleration[1] = aaWorld.y;
  imuData.time = currTime;

  prevTime = currTime;

  // if(radio.available())    // If transmission successful
  // {
  //   Serial.println("Packet Received");
  //   radio.read(&controlData, sizeof(controlData));
  //   lastReceiveTime = millis();       //Time we received data
  //   radio.writeAckPayload(1, &imuData, sizeof(imuData));
  //   controlDataDisplay();
  //   imuDataDisplay();
  // }

  currentTime = millis();

  if (currentTime - lastReceiveTime > 100)
  {
    Serial.println("Entering Fail Safe Mode!");
    failSafe();
  }


  //Steer
  //Trim
  if (controlData.steer > 127)
  {
    steerValue = controlData.steerTrim + (controlData.steer - 127);
  }
  if (controlData.steer < 127)
  {
    steerValue = controlData.steerTrim - (127 - controlData.steer);
  }

  //Range
  controlData.steerRange = map(controlData.steerRange, 0, 255, 0, 25);
  steerValue = map(controlData.steer, 0, 255, (10 + controlData.steerRange), (90 - controlData.steerRange));
 
  rudderServo.write(steerValue);   //Set the rudder angle

  //Throttle
  //Trim
  if (controlData.throttle > 127)
  {
    throttleValue = controlData.throttleTrim + (controlData.throttle - 127);
  }
  if (controlData.throttle < 127)
  {
    throttleValue = controlData.throttleTrim - (127 - controlData.throttle);
  }

  //Range
  controlData.throttleRange = map(controlData.throttleRange, 0, 255, 0, 25);
  throttleValue = map(controlData.throttle, 0, 255, (1000 + controlData.throttleRange), (2000 - controlData.throttleRange));
  

  #ifdef DYNAMIC_GAIN
    // Dynamic gain setting
    // stick center is 1 and at both end-points = 0 , like this curve /\
    // so stabilization magnitude is max at centerstick and zero at full stick
    rollStabMagnitude  = (500 - fabs(constrain((float)TinyCppmReader::width_us(ROLL_CHANNEL) - 1500, -500, 500))) / 500.0f  ;
    gyro_roll_input *= rollStabMagnitude;
    pitchStabMagnitude  = (500 - fabs(constrain((float)TinyCppmReader::width_us(PITCH_CHANNEL) - 1500, -500, 500))) / 500.0f  ;
    gyro_pitch_input *= pitchStabMagnitude;
  #endif // end dynamic gain

  //Limit
    switch (controlData.throttleLimit)
  {
    case 1:
      {
        throttleValue = constrain(throttleValue, 1000, 2000);
        break;
      }
    case 2:
      {
        throttleValue = constrain(throttleValue, 1100, 1900);
        break;
      }
    case 3:
      {
        throttleValue = constrain(throttleValue, 1200, 1800);
        break;
      }
  }
  
  motorESC.speed(throttleValue);    //Set the motor speed

  //TrimTabs
  //Read the control packet depending on mode
  switch (controlData.mode) 
  {
    case 1:
    {
      pitchPID.Setpoint(controlData.setPointData.pitchSetPoint);
      // rollPID.Setpoint(controlData.setPointData.rollSetPoint);
      controlTrimTabs();
      break;
    }

    case 2:
    {
      pitchPID.SetTunings(controlData.pidData.KdPitch, controlData.pidData.KiPitch, controlData.pidData.KdPitch);
      controlTrimTabs();
      break;
    }

    case 3:
    {
      // rollPID.SetTunings(controlData.pidData.KdRoll, controlData.pidData.KiRoll, controlData.pidData.KdRoll);
      controlTrimTabs();
      break;
    }

    case 4:
    {
      trimTabLeftValue = map(controlData.manualData.trimTabLeft, 0, 255, 0, 180);
      trimTabRightValue = map(controlData.manualData.trimTabRight, 0, 255, 0, 180);
      trimTabLeftServo.write(trimTabLeftValue);
      trimTabRightServo.write(trimTabRightValue);
    }
    default:
    break;                                                  //Default case does nothing

  };

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

void failSafe()
{
  // controlData.steer = 127;     
  // controlData.steerRange = 0;
  // controlData.throttle = 0;
  // controlData.throttleTrim = 0;
  // controlData.throttleRange = 0;
  // controlData.throttleLimit = 0;
  controlData.mode = 1;
  controlData.pidData.KpPitch = 2;
  controlData.pidData.KiPitch = 0;
  controlData.pidData.KdPitch = 0;
  // controlData.pidData.KpRoll = 0;
  // controlData.pidData.KiRoll = 0;
  // controlData.pidData.KdRoll = 0;
  // controlData.setPointData.rollSetPoint = 0;
  controlData.setPointData.pitchSetPoint = 0;
}

void resetData()
{
  controlData.steer = 127;     
  controlData.steerRange = 0;
  controlData.throttle = 0;
  controlData.throttleTrim = 0;
  controlData.throttleRange = 0;
  controlData.throttleLimit = 0;
  controlData.mode = 1;
}

void controlTrimTabs()
{
  imuDataDisplay();
  //Dynamic Gain??

  //Pitch and Roll Mixing
  // double rollOutput = rollPID.Run(ypr[1]);
  int pitchOutput = map(ypr[1], -20, 20, 50, 110);
  // trimTabLeftValue =   rollOutput - pitchOutput;
  // trimTabRightValue =  rollOutput + pitchOutput;


  trimTabLeftServo.write(pitchOutput - Offset);
  trimTabRightServo.write(pitchOutput);
}

/**/ //Uncomment to allow print out
void imuDataDisplay()
{
  Serial.println("IMU Data");
  Serial.print("Velocity X-  ");
  Serial.println(imuData.velocity[0]);
  Serial.print("Velocity Y-  ");
  Serial.println(imuData.velocity[1]);
  Serial.print("Acceleration X-  ");
  Serial.println(imuData.acceleration[0]);
  Serial.print("Acceleration Y-  ");
  Serial.println(imuData.acceleration[1]);
  Serial.print("Yaw -  ");
  Serial.println(imuData.ypr[0]);
  Serial.print("Pitch -  ");
  Serial.println(imuData.ypr[1]);
  Serial.print("Roll -  ");
  Serial.println(imuData.ypr[2]);
  Serial.print("Time -  ");
  Serial.println(imuData.time);
}
/**/

/**/  //Uncomment to allow print out
void controlDataDisplay()
{
  Serial.println("Control Data");
  Serial.print("Steer -  ");
  Serial.println(controlData.steer);
  Serial.print("SteerTrim -  ");
  Serial.println(controlData.steerTrim);
  Serial.print("SteerRange -  ");
  Serial.println(controlData.steerRange);
  Serial.print("Throttle -  ");
  Serial.println(controlData.throttle);
  Serial.print("ThrottleTrim -  ");
  Serial.println(controlData.throttleTrim);
  Serial.print("ThrottleRange -  ");
  Serial.println(controlData.throttleRange);
  Serial.print("Mode -  ");
  Serial.println(controlData.mode);
  Serial.print("Throttle Limit -  ");
  Serial.println(controlData.throttleLimit);
}
/**/