# ELET 3220 - CONTROL STSTEMS PROJECT

Arduino Based RC Transmitter/Receiver for Controlling Speed Boat with IMU and Servo controlled Trim Tabs

# TRANSMITTER

## FEATURES

* Classic surface RC Transmitter Controls
* 3 Modes to adjust paramters of control systems and Trim Tabs
* Buzzer & LEDs to notify user of changes 
* IMU Data packets are sent back to TX using ACK payloads output via serial


## MODES

### MODE 1

Adjust setpoint for PID control system (Low signal LED brightness) (Gamma Adjusted).

### MODE 2

Adjust trimtabs postion manually (Medium signal LED brightness).

### MODE 3

Adjust control system parameters (Max signal LED brightness).

# RECEIVER

## FEATURES

## 4 CHANNELS
* ESC Channel
* Rudder Channel
* Trim Tab Left Channel
* Trim Tab Right Channel

## IMU DATA
* Maintains boat pitch and yaw using control IMU and control system
* IMU data is sent back to TX using ACK Payloads

## FAIL SAFE MODE
* When signal is lost automatically enters user defined fail-safe mode