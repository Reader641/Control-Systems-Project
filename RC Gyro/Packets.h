//Packet for Inertial Measuremnt Unit Data
struct IMUDataPacket
{
  float velocity[2];      //Only store x & y. Dont care about z until the boat sinks (cant transmit packets that are larger than 32 bytes)
  float acceleration[2];
  float ypr[3];
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
  byte KpRoll;
  byte KiRoll;
  byte KdRoll;
  byte KpPitch;
  byte KiPitch;
  byte KdPitch;
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