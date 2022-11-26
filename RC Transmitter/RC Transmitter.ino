/*  Arduino Based RC Transceiver with Trim Tab PID control system
    MPU6050 3-Axis Gyroscope & 3-Axis Accelerometer
    IMU Data packets are sent back to TX using ACK payloads
    Mode 1 - Adjust setpoint for PID control system (Low signal LED brightness) (Gamma Adjusted)
    Mode 2 - Adjust trimtabs postion manually (Medium signal LED brightness)
    Mode 3 - Adjust control system parameters (Max signal LED brightness)
*/

//LED 1 (Power Led)
//sw 1 (Power Switch)

//Including libraries
#include "Packets.h"
#include "Pitches.h"
#include <EEPROM.h>
#include <RF24.h>
#include "UpDownCycleButton.h"
#include "PushEventButton.h"

//Ananlog Input Pot pins
//Throttle
#define throttlePin A7      //pot1
#define throttleTrimPin A6  //pot3
#define throttleRangePin A5 //pot5

//Steering
#define steerPin A4         //pot2
#define steerTrimPin A3     //pot4
#define steerRangePin A2    //pot6

//Digital Input Pins
#define throttleLimitPin A0    //sw2
#define btn1Pin 2              //btn1
#define btn2Pin 3              //btn2
#define btn3Pin 4              //btn3
#define btn4Pin 5              //btn4

//Mode Button (Mode 1  Set point control, Mode 2  Control System Adjust, Mode 3  Manual Trim Tab Control)
#define modeSwitchPin 7     //sw3

//Digital Output Pins
//LEDs
#define signalLEDPin A1       //LED2
#define modeLEDPin 6          //LED3

//Buzzer
#define buzzerPin 8           //buzz1

//EEROM storage addresses for user input values
#define setPointAddress 0       //Takes up 2 bytes
#define pidAddress 2            //Takes up 3 bytes
#define manualAddress 5         //Takes up 2 bytes
#define modeAddress 7           //Takes up 1 byte
#define throttleLimitAddress 8  //Takes up 1 byte

RF24 radio(10,9); //nRF24L01 (CE = 10, CSN = 9)

byte address[][6] = {"TX_01", "RX_02"};

//Data packets to transmit
IMUDataPacket imuData;
ControlPacket controlData;
ManualAdjustmentPacket manualData;    
PIDAdjustmentPacket pidData;
SetPointAdjustmentPacket setPointData;

//Setup resettable buttons that increment or decremnt a value each time they are pressed. Once the max value is reached they automatically reset to the intial value.
//The value can also be rest to the intial value programmically or with a long press. 
UpDownCycleButton modeSwitch(modeSwitchPin, &controlData.mode, UpDownCycleButton::ONEBASED, UpDownCycleButton::INCREMENT, 3);

//Set Point Control (Use these when in Mode 1) (The incrementing buttons and decrementing buttons operate on the same value)
UpDownCycleButton rollIncBtn(btn1Pin, &setPointData.rollSetPoint, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 180);
UpDownCycleButton pitchIncBtn(btn2Pin, &setPointData.pitchSetPoint, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 180);
UpDownCycleButton rollDecBtn(btn3Pin, &setPointData.rollSetPoint, UpDownCycleButton::ZEROBASED, UpDownCycleButton::DECREMENT, 180);
UpDownCycleButton pitchDecBtn(btn4Pin, &setPointData.pitchSetPoint, UpDownCycleButton::ZEROBASED, UpDownCycleButton::DECREMENT, 180);

//Control System (Use these in Mode 2) (The incrementing buttons and decrementing buttons buttons when the direction button is pressed)
UpDownCycleButton kpBtn(btn1Pin, &pidData.Kp, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 10);
UpDownCycleButton kiBtn(btn2Pin, &pidData.Ki, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 10);
UpDownCycleButton kdBtn(btn3Pin, &pidData.Kd, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 10);
PushEventButton directionBtn(btn4Pin, PushEventButton::CAPTURERELEASE);     //This button returns true when it pressed and released, false otherwise.

//Trim Tabs (Use these when in Mode 3) (The (The incrementing buttons and decrementing buttons operate on the same value)
UpDownCycleButton leftTrimTabUpBtn(btn1Pin, &manualData.trimTabLeft, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 20);
UpDownCycleButton leftTrimTabDownBtn(btn2Pin, &manualData.trimTabLeft, UpDownCycleButton::ZEROBASED, UpDownCycleButton::DECREMENT, 20);
UpDownCycleButton rightTrimTabUpBtn(btn3Pin, &manualData.trimTabRight, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 20);
UpDownCycleButton rightTrimTabDownBtn(btn4Pin, &manualData.trimTabRight, UpDownCycleButton::ZEROBASED, UpDownCycleButton::DECREMENT, 20);

//Throttle Limit
UpDownCycleButton throttleLimitSwitch(throttleLimitPin, &controlData.throttleLimit, UpDownCycleButton::ONEBASED, UpDownCycleButton::INCREMENT, 3);

void setup() {
  Serial.begin(57600);

  if (!radio.begin())
  {
    Serial.println("Radio is not responding");
  }
  radio.setPALevel(RF24_PA_MAX, 1);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
  radio.setDataRate(RF24_250KBPS);

  radio.stopListening();

  //Inputs
  pinMode(btn1Pin, INPUT_PULLUP);
  pinMode(btn2Pin, INPUT_PULLUP);
  pinMode(btn3Pin, INPUT_PULLUP);
  pinMode(btn4Pin, INPUT_PULLUP);
  pinMode(modeSwitchPin, INPUT_PULLUP);
  pinMode(throttleLimitPin, INPUT_PULLUP);

  //Outputs
  pinMode(buzzerPin, OUTPUT);
  pinMode(signalLEDPin, OUTPUT);
  pinMode(modeLEDPin, OUTPUT);

  delay(20);      //Short delay to prevent noise from skewing analog reading on other pins

  //Default values for control packet, map into 1 Byte values
  controlData.steer = map(analogRead(steerPin), 0, 1023, 0, 255);         
  controlData.steerTrim = map(analogRead(steerTrimPin), 0, 1023, 0, 255);     
  controlData.steerRange = map(analogRead(steerRangePin), 0, 1023, 0, 255);
  controlData.throttle = map(analogRead(throttlePin), 0, 1023, 0, 255); 
  controlData.throttleTrim = map(analogRead(throttleTrimPin), 0, 1023, 0, 255);
  controlData.throttleRange = map(analogRead(throttleRangePin), 0, 1023, 0, 255);
  controlData.mode = EEPROM[modeAddress];                      //Get saved value from EEPROM
  controlData.throttleLimit = EEPROM[throttleLimitAddress];    //Get saved value from EEPROM                   
  
  //Set values to default if EEPROM is not written
  if (controlData.throttleLimit == 255)
  {
    controlData.throttleLimit = 1; 
  }
  if (controlData.mode == 255)
  {
    controlData.mode = 1;
  }

  //Read saved values for setpoint adjustment mode (1)
  EEPROM.get(setPointAddress, setPointData);

  //Set values to default if EEPROM is not written
  if (setPointData.pitchSetPoint == 255)
  {
    setPointData.pitchSetPoint = 0;
  }
  if (setPointData.rollSetPoint == 255)
  {
    setPointData.rollSetPoint = 0;
  }

  //Read saved values for adjustment mode (2)
  EEPROM.get(manualAddress, manualData);

  //Set values to default if EEPROM is not written
  if (manualData.trimTabLeft == 255)
  {
    manualData.trimTabLeft = 0;
  }
  if (manualData.trimTabRight == 255)
  {
    manualData.trimTabRight = 0;
  }

  //Read saved values for adjustment mode (3)
  EEPROM.get(pidAddress, pidData);

    //Set values to default if EEPROM is not written
  if (pidData.Kp == 255)
  {
    pidData.Kp = 0;
  }
  if (pidData.Ki == 255)
  {
    pidData.Ki = 0;
  }
  if (pidData.Kd == 255)
  {
    pidData.Kd = 0;
  }
}


void loop() 
{
  //Read anlog inputs values and map 
  controlData.steer = map(analogRead(steerPin), 0, 1023, 0, 255);       
  controlData.steerTrim = map(analogRead(steerTrimPin), 0, 1023, 0, 255);     
  controlData.steerRange = map(analogRead(steerRangePin), 0, 1023, 0, 255);
  controlData.throttle = map(analogRead(throttlePin), 0, 1023, 0, 255); 
  controlData.throttleTrim = map(analogRead(throttleTrimPin), 0, 1023, 0, 255);
  controlData.throttleRange = map(analogRead(throttleRangePin), 0, 1023, 0, 255);
  if (throttleLimitSwitch.updateValue())
  {
    tone(buzzerPin, NOTE_G5, 20);
  }

  //Read all digital inputs depending on mode
  if (modeSwitch.updateValue())
  {
    tone(buzzerPin, NOTE_D5, 20);
  }
  switch (controlData.mode) 
  {
    case 1:
    {
      if(rollIncBtn.updateValue())                                      //Get values from the buttons
      {
        tone(buzzerPin, NOTE_G5, 20);
      }
      
      if(pitchIncBtn.updateValue())
      {
        tone(buzzerPin, NOTE_E4, 20);
      }

     if(rollDecBtn.updateValue())
      {
        tone(buzzerPin, NOTE_G5, 20);
      }

      if (pitchDecBtn.updateValue())
      {
        tone(buzzerPin, NOTE_E4, 20);
      }

      Serial.println("Set Point Data");
      Serial.print("Roll Set Point -  ");
      Serial.println(setPointData.rollSetPoint);
      Serial.print("Pitch Set Point -  ");
      Serial.println(setPointData.pitchSetPoint);

      //Update setPointData in control packet
      controlData.setPointData = setPointData;

      //Save values for setpoint adjustment mode in EEPROM
      EEPROM.put(setPointAddress, setPointData);

      //Dim the mode LED to 5 while in mode 1
      analogWrite(modeLEDPin, 5);  
      break;
    }

    case 2:
    {
      if(leftTrimTabUpBtn.updateValue())                                      //Get values from the buttons
      {
        tone(buzzerPin, NOTE_G5, 20);
      }
      
      if(rightTrimTabUpBtn.updateValue())
      {
        tone(buzzerPin, NOTE_G5, 20);
      }

     if(leftTrimTabDownBtn.updateValue())
      {
        tone(buzzerPin, NOTE_E4, 20);
      }

      if (rightTrimTabDownBtn.updateValue())
      {
        tone(buzzerPin, NOTE_E4, 20);
      }
      Serial.println("Manual Data");
      Serial.print("Trim Tab Left -  ");
      Serial.println(manualData.trimTabLeft);
      Serial.print("Trim Tab Right -  ");
      Serial.println(manualData.trimTabRight);

      //Update manualData in control packet
      controlData.manualData = manualData;

      //Save values for manual adjustment mode in EEPROM
      EEPROM.put(manualAddress, manualData);

      //Dim the mode LED to 36 while in mode 2
      analogWrite(modeLEDPin, 36);
      break;
    }

    case 3:
    {
      if (directionBtn.pushed())                                //Check state of direction button
      {
        kpBtn.changeDirection();
        kiBtn.changeDirection();
        kdBtn.changeDirection();
        tone(buzzerPin, kpBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4, 20);
      }

      if(kpBtn.updateValue())
      {
        tone(buzzerPin, kpBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4 , 20);
      }

     if(kiBtn.updateValue())
      {
        tone(buzzerPin, kpBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4 , 20);
      }

      if (kdBtn.updateValue())
      {
        tone(buzzerPin, kpBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4 , 20);
      }
      Serial.println("PID Data");
      Serial.print("Kp -  ");
      Serial.println(pidData.Kp);
      Serial.print("Ki -  ");
      Serial.println(pidData.Ki);
      Serial.print("Kd -  ");
      Serial.println(pidData.Kd);
      
      //Update pidData in control packet
      controlData.pidData = pidData;

      //Save values in for PID adjustemnt mode in EEPROM 
      EEPROM.put(pidAddress, pidData);

      //Dim the mode LED to 255 while in mode 3
      analogWrite(modeLEDPin, 255);
      break;
    }
    default:
    break;                                                  //Default case does nothing

  };

  EEPROM.update(modeAddress, controlData.mode);
  EEPROM.update(throttleLimitAddress, controlData.throttleLimit);

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

  unsigned long startTime = micros();                              //Get the time when transmisson starts
  bool report = radio.write(&controlData, sizeof(controlData));   //Transmit the control data packet and get status
  unsigned long endTime = micros();                               //Get the time when transmission ends
  if(report)    // If transmission successful
  {
    Serial.println("Transmission Sucecssful!");
    Serial.print("Time to transmit = ");
    Serial.print(endTime - startTime);  // print the timer result
    Serial.println(" us.");
    digitalWrite(signalLEDPin, HIGH);    //Light signal LED if control data transmitted
    if (radio.available())
    {
      radio.read(&imuData, sizeof(imuData));
        Serial.println("IMU Data");
        Serial.print("Velocity -  ");
        Serial.println(imuData.velocity);
        Serial.print("Acceleration -  ");
        Serial.println(imuData.acceleration);
        Serial.print("Pitch -  ");
        Serial.println(imuData.pitch);
        Serial.print("Yaw -  ");
        Serial.println(imuData.yaw);
        Serial.print("Roll -  ");
        Serial.println(imuData.roll);
        Serial.print("Time -  ");
        Serial.println(imuData.time);
      digitalWrite(signalLEDPin, LOW);  //Turn off signal LED if imu data (ACK payload) received
    }
    else
    {
      Serial.println("ACK Payload not received (Empty).");
    }
  }
  else
  {
    Serial.println("Transmission failed or timed out.");
  }
}