//Packet for Inertial Measuremnt Unit Data
struct IMUDataPacket
{
  float velocity;
  float acceleration;
  float pitch;
  float yaw;
  float roll;
  float time;
};

//Packet structure for adjustment of setpoint
struct SetPointAdjustmentPacket
{
  byte pitchSetPoint;
  byte rollSetPoint;
};

//Packet structure for PID parameter adjustment
struct PIDAdjustmentPacket
{
  byte Kp;
  byte Ki;
  byte Kd;
};

//Packet structure for manual trimtab adjustment
struct ManualAdjustmentPacket
{
  byte trimTabLeft;
  byte trimTabRight;
};

//Packet structure for control signals
struct ControlPacket
{
  byte steer;
  byte steerTrim;     
  byte steerRange;
  byte throttle;
  byte throttleTrim;
  byte throttleRange;
  byte throttleLimit;
  byte mode;
  SetPointAdjustmentPacket setPointData;
  PIDAdjustmentPacket pidData;
  ManualAdjustmentPacket manualData;
};