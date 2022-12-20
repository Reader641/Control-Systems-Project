/*  Arduino Based RC Transmitter with Trim Tab PID control system
    MPU6050 3-Axis Gyroscope & 3-Axis Accelerometer
    IMU Data packets are sent back to TX using ACK payloads
    Mode 1 - Adjust setpoint for PID control system (Low signal LED brightness) (Gamma Adjusted)
    Mode 2 - Adjust pitch control system parameters (Medium-Low signal LED brightness)
    Mode 3 - Adjust roll control system parameters  (Medium-High signal LED brightness)
    Mode 4 - Adjust trimtabs postion manually (Max signal LED brightness)

*/

//Simulator doesn't implement NRF24l01
//Comment out radio.write lines when using simulator to speed up.
//LED 1 (Power Led)
//sw 1 (Power Switch)

//Including libraries
#include "Packets.h"
#include "Pitches.h"
#include <EEPROM.h>
#include <RF24.h>
#include "UpDownCycleButton.h"
#include "PushEventButton.h"

//Analog Input Pot pin
//Throttle
#define THROTTLE_PIN A7      //pot1
#define THROTTLE_TRIM_PIN A6  //pot3
#define THROTTLE_RANGE_PIN A5 //pot5

//Steering
#define STEER_PIN A4         //pot2
#define STEER_TRIM_PIN A3     //pot4
#define STEER_RANGE_PIN A2    //pot6

//Digital Input Pins
#define BTN_1_PIN 2              //btn1
#define BTN_2_PIN 3              //btn2
#define BTN_3_PIN 4              //btn3
#define BTN_4_PIN 5              //btn4

//Mode Button (Mode 1  Set point control, Mode 2  Control System Adjust, Mode 3  Manual Trim Tab Control)
#define MODE_PIN 6     //sw3
#define THROTTLE_LIMIT_PIN 7    //sw2

//Digital Output Pins
//LEDs
#define SIGNAL_LED_PIN A1       //LED2
#define MODE_LED_PIN A0          //LED3

//Buzzer
#define BUZZER_PIN 8           //buzz1

//EEROM storage addresses for user input values
#define SET_POINT_ADDRESS 0       //Takes up 2 bytes
#define PID_ADDRESS 2            //Takes up 6 bytes
#define MANUAL_ADDRESS 8         //Takes up 2 bytes
#define MODE_ADDRESS 9           //Takes up 1 byte
#define THROTTLE_LIMIT_ADDRESS 9  //Takes up 1 byte

RF24 radio(10, 9); //nRF24L01 (CE = 10, CSN = 9)

byte address[][6] = {"TX_01", "RX_01"};

//Data packets to transmit
ControlPacket controlData;
ManualAdjustmentPacket manualData;
PIDAdjustmentPacket pidData;
SetPointAdjustmentPacket setPointData;

//ACK payload to receive
IMUDataPacket imuData;

//Setup resettable buttons that increment or decremnt a value each time they are pressed. Once the max value is reached they automatically reset to the intial value.
//The value can also be rest to the intial value programmically or with a long press.
UpDownCycleButton modeSwitch(MODE_PIN, &controlData.mode, UpDownCycleButton::ONEBASED, UpDownCycleButton::INCREMENT, 4);

//Set Point Control (Use these when in Mode 1) (The incrementing buttons and decrementing buttons operate on the same value)
UpDownCycleButton rollIncBtn(BTN_1_PIN, &setPointData.rollSetPoint, UpDownCycleButton::ZEROBASED, UpDownCycleButton::DECREMENT, 180);
UpDownCycleButton pitchIncBtn(BTN_2_PIN, &setPointData.rollSetPoint, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 180);
UpDownCycleButton rollDecBtn(BTN_3_PIN, &setPointData.pitchSetPoint, UpDownCycleButton::ZEROBASED, UpDownCycleButton::DECREMENT, 180);
UpDownCycleButton pitchDecBtn(BTN_4_PIN, &setPointData.pitchSetPoint, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 180);

//Control System (Use these in Mode 2) (The incrementing buttons and decrementing buttons buttons when the direction button is pressed)
UpDownCycleButton kpPitchBtn(BTN_1_PIN, &pidData.KpPitch, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 10);
UpDownCycleButton kiPitchBtn(BTN_2_PIN, &pidData.KiPitch, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 10);
UpDownCycleButton kdPitchBtn(BTN_3_PIN, &pidData.KdPitch, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 10);
PushEventButton directionBtn(BTN_4_PIN, PushEventButton::CAPTURERELEASE);     //This button returns true when it pressed and released, false otherwise.

//Control System (Use these in Mode 3) (The incrementing buttons and decrementing buttons buttons when the direction button is pressed)
UpDownCycleButton kpRollBtn(BTN_1_PIN, &pidData.KpRoll, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 10);
UpDownCycleButton kiRollBtn(BTN_2_PIN, &pidData.KiRoll, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 10);
UpDownCycleButton kdRollBtn(BTN_3_PIN, &pidData.KdRoll, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 10);

//Trim Tabs (Use these when in Mode 4) (The (The incrementing buttons and decrementing buttons operate on the same value)
UpDownCycleButton leftTrimTabUpBtn(BTN_1_PIN, &manualData.trimTabLeft, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 20);
UpDownCycleButton leftTrimTabDownBtn(BTN_2_PIN, &manualData.trimTabLeft, UpDownCycleButton::ZEROBASED, UpDownCycleButton::DECREMENT, 20);
UpDownCycleButton rightTrimTabUpBtn(BTN_3_PIN, &manualData.trimTabRight, UpDownCycleButton::ZEROBASED, UpDownCycleButton::INCREMENT, 20);
UpDownCycleButton rightTrimTabDownBtn(BTN_4_PIN, &manualData.trimTabRight, UpDownCycleButton::ZEROBASED, UpDownCycleButton::DECREMENT, 20);

//Throttle Limit
UpDownCycleButton throttleLimitSwitch(THROTTLE_LIMIT_PIN, &controlData.throttleLimit, UpDownCycleButton::ONEBASED, UpDownCycleButton::INCREMENT, 3);


void setup() {
  Serial.begin(115200);

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
  pinMode(BTN_1_PIN, INPUT_PULLUP);
  pinMode(BTN_2_PIN, INPUT_PULLUP);
  pinMode(BTN_3_PIN, INPUT_PULLUP);
  pinMode(BTN_4_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(THROTTLE_LIMIT_PIN, INPUT_PULLUP);

  //Outputs
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SIGNAL_LED_PIN, OUTPUT);
  pinMode(MODE_LED_PIN, OUTPUT);

  delay(20);      //Short delay to prevent noise from skewing analog reading on other pins

  //Default values for control packet
  controlData.mode = EEPROM[MODE_ADDRESS];                      //Get saved value from EEPROM
  controlData.throttleLimit = EEPROM[THROTTLE_LIMIT_ADDRESS];    //Get saved value from EEPROM

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
  EEPROM.get(SET_POINT_ADDRESS, setPointData);

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
  EEPROM.get(MANUAL_ADDRESS, manualData);

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
  EEPROM.get(PID_ADDRESS, pidData);

  //Set values to default if EEPROM is not written
  if (pidData.KpRoll == 255)
  {
    pidData.KpRoll = 0;
  }
  if (pidData.KiRoll == 255)
  {
    pidData.KiRoll = 0;
  }
  if (pidData.KdRoll == 255)
  {
    pidData.KdRoll = 0;
  }
  if (pidData.KpPitch == 255)
  {
    pidData.KpPitch = 0;
  }
  if (pidData.KiPitch == 255)
  {
    pidData.KiPitch = 0;
  }
  if (pidData.KdPitch == 255)
  {
    pidData.KdPitch = 0;
  }
}

void loop()
{
  //Read anlog inputs values and map

  controlData.steer = map(analogRead(STEER_PIN), 0, 1023, 0, 255);
  controlData.steerTrim = map(analogRead(STEER_TRIM_PIN), 0, 1023, 0, 255);
  controlData.steerRange = map(analogRead(STEER_RANGE_PIN), 0, 1023, 0, 255);
  controlData.throttle = map(analogRead(THROTTLE_PIN), 0, 1023, 0, 255);
  controlData.throttleTrim = map(analogRead(THROTTLE_TRIM_PIN), 0, 1023, 0, 255);
  controlData.throttleRange = map(analogRead(THROTTLE_RANGE_PIN), 0, 1023, 0, 255);

  if (throttleLimitSwitch.updateValue())
  {
    tone(BUZZER_PIN, NOTE_G5, 20);
  }

  //Read all digital inputs depending on mode
  if (modeSwitch.updateValue())
  {
    tone(BUZZER_PIN, NOTE_D5, 20);
  }
  switch (controlData.mode)
  {
    case 1:
    {
      if (rollIncBtn.updateValue())                                     //Get values from the buttons
      {
        tone(BUZZER_PIN, NOTE_G5, 20);
      }

      if (pitchIncBtn.updateValue())
      {
        tone(BUZZER_PIN, NOTE_G5, 20);
      }

      if (rollDecBtn.updateValue())
      {
        tone(BUZZER_PIN, NOTE_E4, 20);
      }

      if (pitchDecBtn.updateValue())
      {
        tone(BUZZER_PIN, NOTE_E4, 20);
      }

      /**/setPointDataDisplay();/**/ //Uncomment to allow print out

      //Update setPointData in control packet
      controlData.setPointData = setPointData;

      //Save values for setpoint adjustment mode in EEPROM
      EEPROM.put(SET_POINT_ADDRESS, setPointData);

      //Dim the mode LED to 5 while in mode 1
      analogWrite(MODE_LED_PIN, 5);
      break;
    }

    case 2:
    {
      if (directionBtn.pushed())                                //Check state of direction button
      {
        kpPitchBtn.changeDirection();
        kiPitchBtn.changeDirection();
        kdPitchBtn.changeDirection();
        tone(BUZZER_PIN, kpPitchBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4, 20);
      }

      if (kpPitchBtn.updateValue())
      {
        tone(BUZZER_PIN, kpPitchBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4, 20);
      }

      if (kiPitchBtn.updateValue())
      {
        tone(BUZZER_PIN, kiPitchBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4, 20);
      }

      if (kdPitchBtn.updateValue())
      {
        tone(BUZZER_PIN, kdPitchBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4, 20);
      }

      /**/pidPitchDataDisplay();/**/ //Uncomment to allow print out

      //Update pidData in control packet
      controlData.pidData = pidData;

      //Save values in for PID adjustemnt mode in EEPROM
      EEPROM.put(PID_ADDRESS, pidData);

      //Dim the mode LED to 255 while in mode 3
      analogWrite(MODE_LED_PIN, 36);
      break;
    }

    case 3:
    {
      if (directionBtn.pushed())                                //Check state of direction button
      {
        kpRollBtn.changeDirection();
        kiRollBtn.changeDirection();
        kdRollBtn.changeDirection();
        tone(BUZZER_PIN, kpRollBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4, 20);
      }

      if (kpRollBtn.updateValue())
      {
        tone(BUZZER_PIN, kpRollBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4, 20);
      }

      if (kiRollBtn.updateValue())
      {
        tone(BUZZER_PIN, kiRollBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4, 20);
      }

      if (kdRollBtn.updateValue())
      {
        tone(BUZZER_PIN, kdRollBtn.getDirection() == UpDownCycleButton::INCREMENT ? NOTE_G5 : NOTE_E4, 20);
      }

      /**/pidRollDataDisplay();/**/ //Uncomment to allow print out

      //Update pidData in control packet
      controlData.pidData = pidData;

      //Save values in for PID adjustemnt mode in EEPROM
      EEPROM.put(PID_ADDRESS, pidData);

      //Dim the mode LED to 255 while in mode 3
      analogWrite(MODE_LED_PIN, 114);
      break;
    }

    case 4:
    {
      if (leftTrimTabUpBtn.updateValue())                                     //Get values from the buttons
      {
        tone(BUZZER_PIN, NOTE_G5, 20);
      }

      if (rightTrimTabUpBtn.updateValue())
      {
        tone(BUZZER_PIN, NOTE_G5, 20);
      }

      if (leftTrimTabDownBtn.updateValue())
      {
        tone(BUZZER_PIN, NOTE_E4, 20);
      }

      if (rightTrimTabDownBtn.updateValue())
      {
        tone(BUZZER_PIN, NOTE_E4, 20);
      }

      /**/manualDataDisplay();/**/ //Uncomment to allow print out

      //Update manualData in control packet
      controlData.manualData = manualData;

      //Save values for manual adjustment mode in EEPROM
      EEPROM.put(MANUAL_ADDRESS, manualData);

      //Dim the mode LED to 36 while in mode 4
      analogWrite(MODE_LED_PIN, 255);
      break;
    }
    default:
      controlData.mode = 1;
      break;                                                  //Default case sets mode to 1

  };

  EEPROM.update(MODE_ADDRESS, controlData.mode);
  EEPROM.update(THROTTLE_LIMIT_ADDRESS, controlData.throttleLimit);

  /**/controlDataDisplay();/**/ //Uncomment to allow print out

  unsigned long startTime = micros();                              //Get the time when transmisson starts
  bool report = radio.write(&controlData, sizeof(controlData));   //Transmit the control data packet and get status
  unsigned long endTime = micros();                               //Get the time when transmission ends
  if (report)   // If transmission successful
  {
    Serial.println("Transmission Sucecssful!");
    Serial.print("Time to transmit = ");
    Serial.print(endTime - startTime);  // print the timer result
    Serial.println(" us.");
    digitalWrite(SIGNAL_LED_PIN, HIGH);    //Light signal LED if control data transmitted
    if (radio.available())
    {
      radio.read(&imuData, sizeof(imuData));
      /**/imuDataDisplay();/**/ //uncomment to allow print out
      digitalWrite(SIGNAL_LED_PIN, LOW);  //Turn off signal LED if imu data (ACK payload) received
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
  Serial.print("Picth -  ");
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

void setPointDataDisplay()
{
  Serial.println("Set Point Data");
  Serial.print("Roll Set Point -  ");
  Serial.println(setPointData.rollSetPoint);
  Serial.print("Pitch Set Point -  ");
  Serial.println(setPointData.pitchSetPoint);
}

void manualDataDisplay()
{
  Serial.println("Manual Data");
  Serial.print("Trim Tab Left -  ");
  Serial.println(manualData.trimTabLeft);
  Serial.print("Trim Tab Right -  ");
  Serial.println(manualData.trimTabRight);
}
/**/

/**/
void pidRollDataDisplay()
{
  Serial.println("PID Data");
  Serial.print("Kp Roll - ");
  Serial.println(pidData.KpRoll);
  Serial.print("Ki Roll- ");
  Serial.println(pidData.KiRoll);
  Serial.print("Kd Roll- ");
  Serial.println(pidData.KdRoll);
}
/**/

/**/
void pidPitchDataDisplay()
{
  Serial.print("Kp Pitch - ");
  Serial.println(pidData.KpPitch);
  Serial.print("Ki Pitch- ");
  Serial.println(pidData.KiPitch);
  Serial.print("Kd Pitch- ");
  Serial.println(pidData.KdPitch);
}
/**/