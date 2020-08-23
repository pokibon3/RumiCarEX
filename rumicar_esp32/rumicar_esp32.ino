//=========================================================
//  rumicar_esp32.ino :  RumiCar application
//  History     : V1.0  2020-08-18 new create 
//=========================================================
#include <Wire.h>
#define  EXTERN extern
#include "RumiCar_esp32.h"
#include "BluetoothSerial.h"

#define BT_ON
#ifdef BT_ON
BluetoothSerial SerialBT;
#endif
//=========================================================
//  RumiCar Default Parameter
//=========================================================
#define DEVICE_NAME     "RumiCar_ESP32" // BLE Device Name
#define PILOT_MODE      1         // 1:Auto 2:Manual
#define MAX_TORQUE      255       // max pwm value
#define MAX_POWER       230       // 230 max
#define MIN_POWER       120       // 120 min
#define MAX_SPEED       1.0       // max speed factor
#define MID_SPEED       0.75      // mid speed factor
#define LOW_SPEED       0.5       // low speed need torque
#define BRAKE_TIME      1000      // coasting max time
#define MAX_DISTANCE_W  300       // max distance to wall
#define MID_DISTANCE_W  150       // keep distance from inside wall
#define MIN_DISTANCE_W  50        // min distance to wall
#define MAX_ANGLE       100       // max 100%
#define LIMIT_ANGLE      30       // max  30%
#define OVR_DISTANCE_F  800       // 800mm  detect straight
#define MAX_DISTANCE_F  300       // 300mm  detect front wall
#define MID_DISTANCE_F  200       // 200mm  speed down distance
#define MIN_DISTANCE_F  100       // 100mm  reverse start distance
#define REVERSE_DISTANCE 100      // 100mm  reverse distance
#define STP_DISTANCE_F  0         // 0mm kiss to wall
#define REVERSE_TIME    200       // reverse time 500ms
#define OIO_OFFSET      50        // out in out offset 0=off
#define OIO_TIME        500       // continue 500ms to inside
#define MAX_STOP_TIME   4         // stop time(for reverse mode)
#define KP_CONST        0.8       // Konstante p
#define KD_CONST        0.1       // Konstante d
#define DMODE           0         // Differential control mode
                                  // 1: normalize 0:active
#ifdef  SENSOR_VL53L1X            // use VL53L1X
VL53L1X sensor0;                  // create right sensor instanse
VL53L1X sensor1;                  // create front sensor instance
VL53L1X sensor2;                  // create left  sensor instance
#else                             // use VL53L0X
VL53L0X sensor0;                  // create right sensor instance
VL53L0X sensor1;                  // create front sensor instance
VL53L0X sensor2;                  // create left  sensor instance
#endif

//=========================================================
//  auto pilot variables difinition
//=========================================================
static int s0, s1, s2;                   // left, center, right sensor value
static int rawS0, rawS1, rawS2;          // sensor row value
static int ps0 = 0, ps1 = 0, ps2 = 0;    // left, center, right pre sensor value
static int st0, st1, st2;                // left, center, right sensor status
static int curDriveDir = BRAKE;          // current direction
static int lastDriveDir = BRAKE;         // last drive direction
static int lastSteerDir = CENTER;        // last steer direction

static unsigned long steerCount[3];      // steer count
static int courseLayout = 1;             // 0:LEFT 2:RIHGT 1:unknown
static float prep = 0;                   // pre value of proportional control
static float p;                          // proportional control
static float d  = 0;                     // differential control
static float d1 = 0;
static float d2 = 0;
static float kp = KP_CONST;
static float kd = KD_CONST;
static unsigned long preTime = 0;
static unsigned long dTime = 0;
static int dMode = DMODE;                // difference control mode
static int16_t Joy_X = 0;                // JoyStick X
static int16_t Joy_Y = 0;                // JoyStick Y
static int16_t autoPilot = PILOT_MODE;   // Default Pilot Mode
static int16_t Trim = 0;                 // Default Trimmer value
static int16_t maxSpeed = MAX_POWER;     // Default Max Motor Power
static unsigned long sFwdTime = 0;       // fwd passage time
static unsigned long eFwdTime = 0;       // fwd passage time
static unsigned long sCornerTime = 0;    // cornering passage time
static unsigned long eCornerTime = 0;    // cornering passage time
static int preDistance    = 0;           // previous distance(S1)
static int preSpeed       = 0;           // previous speed
static int targetSpeed    = 0;           // target speed
static int requestTorque  = 0;           // PWM Value
static int curSpeed       = 0;           // current speed
static int reverseMode    = 0;           // reversing
static int stopCounter    = 0;           // stop times
static int steerDir;                     // steering direction
static int dAngle;                       // steering angle
static int oioCount       = 0;           // oio count
static int cornerFlag     = 0;           // oio mode flag
//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
  int i;
  for (i = 0; i < 3; i++) {
    steerCount[i] = 0;
  }
  RC_setup();               //   RumiCar initial function
#ifdef BT_ON
  Serial.println("Start bluetooth!");
  SerialBT.begin(DEVICE_NAME);
  Serial.println("The device started, now you can pair it with bluetooth!");
#endif
}

//=========================================================
//  auto_driving
//=========================================================
void auto_driving()
{
  int dDistance      = 0;
  int minDistance    = 0;

  //=========================================================
  //  calc distance to wall & current speed
  //=========================================================
  dDistance = preDistance - s1;
  curSpeed = (dDistance * 108) / (int)dTime;
  //=========================================================
  //  detect near to wall and set reverse mode
  //=========================================================
  if ((s1 < MIN_DISTANCE_F) && (abs(dDistance) <= 5)) {
    stopCounter++;
    if (stopCounter > MAX_STOP_TIME && reverseMode == 0) {
      reverseMode = 1;
      stopCounter = 0;
    }
  } else {
    stopCounter = 0;
  }
  //=========================================================
  //  set reverse distance (expand 100ms)
  //=========================================================
  if (reverseMode == 1) {
    minDistance = MIN_DISTANCE_F + REVERSE_DISTANCE;
  } else {
    minDistance = MIN_DISTANCE_F;
  }
  //=========================================================
  //  calc distance and speed
  //=========================================================
  if(s1 < minDistance){                    // x < 100
    if (reverseMode == 0) {
      curDriveDir   = BRAKE;
      requestTorque = MAX_TORQUE;
    } else {
      curDriveDir = REVERSE;
      requestTorque = map(s1, STP_DISTANCE_F, MIN_DISTANCE_F, MIN_POWER + dAngle / 2, maxSpeed * LOW_SPEED);
    }
  } else {
    reverseMode = 0;
    if (s1 > OVR_DISTANCE_F) {                // 800 < x
      curDriveDir = FORWARD;
      requestTorque = maxSpeed * MAX_SPEED;   // full throttle
    } else {                                  // 100 < x < 800
      curDriveDir = FORWARD;
      targetSpeed   = map(s1, MIN_DISTANCE_F, OVR_DISTANCE_F, 0, maxSpeed * MAX_SPEED);
      requestTorque = map(s1, MIN_DISTANCE_F, OVR_DISTANCE_F, MIN_POWER + dAngle / 2, maxSpeed * MAX_SPEED);
      if (targetSpeed <= curSpeed) {           // over speed
        curDriveDir   = BRAKE;
        requestTorque = MAX_TORQUE;
      }
    }
  }
  RC_drive(curDriveDir, requestTorque);
  //=========================================================
  //  calc forward time
  //=========================================================
  if (lastDriveDir != FORWARD && curDriveDir == FORWARD) {  // change to FORWARD
    sFwdTime = millis();
    eFwdTime = 0;                                           // clear elapse time
  }
  if (lastDriveDir == FORWARD && curDriveDir == FORWARD) {  // continue FORWARD
    eFwdTime = millis() - sFwdTime;                         // add elapse  time
    if (eFwdTime > 10000) {
      eFwdTime = 10000;
    }
  }
  preDistance  = s1;
  preSpeed     = curSpeed;
}

//=========================================================
//  auto_steering
//=========================================================
void auto_steering()
{
  int steerMax;                     // limit steering angle
  int dMin = MIN_DISTANCE_W;        // min distance to wall
  int dMax = MAX_DISTANCE_W;        // max distance to wall
  int pos;                          // position between wall to wall
  int oioOffset;                    // out in out offset
  int targetPos;                    // target position
  int targetPos2;                   // oio target
  int curPos;                       // current position

  //=========================================================
  //  detect straight
  //=========================================================
  if (s1 > OVR_DISTANCE_F &&  // detect straight
      s0 > MID_DISTANCE_W &&  // not near left wall
      s2 > MID_DISTANCE_W) {  // not near write wall
    steerMax = LIMIT_ANGLE ;  // limit steering
  } else {
    steerMax = MAX_ANGLE;
  }
  //=========================================================
  //  Limit wall distance
  //=========================================================
  if      (s0 > dMax) s0 = dMax;    // correct Max Value
  else if (s0 < dMin) s0 = dMin;    // correct Min Value
  if      (s2 > dMax) s2 = dMax;    // correct Max Value
  else if (s2 < dMin) s2 = dMin;    // correct Min Value
  //=========================================================
  //  detect out in out mode & calc targetPos
  //=========================================================
  if (dMode > 0) {                  // oio mode on
    if (courseLayout == LEFT) {
      oioOffset = OIO_OFFSET * dMode;     // keep near left wall
    } else if (courseLayout == RIGHT) {
      oioOffset = - OIO_OFFSET * dMode;   // keep near right wall
    } else {
      oioOffset = 0;
    }
    if (cornerFlag == 1 && s1 > OVR_DISTANCE_F) {      // straight : keep near outside wall
      oioOffset = - oioOffset;
      cornerFlag = 0;
      oioCount = 0;
    } else if (cornerFlag == 0 && s1 <= OVR_DISTANCE_F) {
      cornerFlag = 1;
      oioCount = 0;
    }
    oioCount ++;
  } else {
    oioOffset = 0;
  }
  targetPos = (s0 + s2) / 2;
  targetPos2 = constrain((s0 + s2) / 2 + oioOffset, MIN_DISTANCE_W, s0 + s2 - MIN_DISTANCE_W);
  if (targetPos - targetPos2 > 10) {
    targetPos = constrain(targetPos - oioCount * 2, targetPos2, targetPos);
  } else if (targetPos - targetPos2 < 10) {
    targetPos = constrain(targetPos + oioCount * 2, targetPos, targetPos2) ;
  }
///*
  Serial.print("\tSensor0:");
  Serial.print(rawS0);
  Serial.print("\tStatus:");
  Serial.print(st0);
  Serial.print("\tSensor1:");
  Serial.print(rawS1);
  Serial.print("\tStatus:");
  Serial.print(st1);
  Serial.print("\tSensor2:");
  Serial.print(rawS2);
  Serial.print("\tStatus:");
  Serial.print(st2);
  Serial.print("\tcourseLayout:");
  Serial.print(courseLayout);
  Serial.print("\tTargetPos:");
  Serial.print(targetPos);
  Serial.print("\tTargetPos2:");
  Serial.print(targetPos2);
  Serial.print("\toioCount:");
  Serial.print(oioCount);
  Serial.println();
//*/
  //=========================================================
  //  calc steering angle
  //=========================================================
  curPos = s2;                      // standard wall is Right
  p = (targetPos - curPos) * kp;    // P control
  d = (p - prep) * 1000 / dTime * kd;  // calc differential
//  if (dMode == 1) {
//    d = (d + d1 + d2) / 3;
//    d2 = d1;
//    d1 = d;
//  }
  dAngle = constrain(p + d , -steerMax, steerMax);  // normalize dAngle -100 to 100
  prep = p;

  if (dAngle > 0) {
    steerDir = LEFT;
    dAngle = abs(dAngle);
  } else if (dAngle < 0) {
    steerDir = RIGHT;
    dAngle = abs(dAngle);
  } else {
    steerDir = CENTER;
    dAngle = 0;
  }
  //=========================================================
  //  kerikaeshi
  //=========================================================
  if (reverseMode == 1) {
    if (steerDir == RIGHT) {
      steerDir = LEFT;                  // counter steer
    } else if (steerDir == LEFT) {
      steerDir = RIGHT;                 // counter steer
    } else {                            // kirikaeshi
      steerDir = CENTER;
      dAngle = steerMax * 0.7;
    }
  }
  //=========================================================
  //  steering control
  //=========================================================
  RC_steer(steerDir, dAngle);           // steering
  //=========================================================
  //  calc corner time
  //=========================================================
  if (lastSteerDir != steerDir || lastDriveDir != curDriveDir) { // change drive direction
    sCornerTime = millis();
    eCornerTime = 0;                            // clear elapse time
  }
  if (lastSteerDir == steerDir && lastDriveDir == curDriveDir) { // continue
    eCornerTime = millis() - sCornerTime;       // calc elapse time
    if (eCornerTime > 10000) {
      eCornerTime = 10000;
    }
  }
  //=========================================================
  //  detect course layout
  //=========================================================
  steerCount[steerDir]++;
  if (steerCount[LEFT] > 500 || steerCount[RIGHT] > 500) {
    if (steerCount[LEFT] > (steerCount[RIGHT] * 2)) { //grater than 2 times
      courseLayout = LEFT;
    } else if ((steerCount[LEFT] * 2) < steerCount[RIGHT] )
      courseLayout = RIGHT;
    } else {
      courseLayout = CENTER;                    // unknown
  }
  //=========================================================
  //  save old direction
  //=========================================================
  lastDriveDir = curDriveDir;                 // save last value
  lastSteerDir = steerDir;
}

//=========================================================
//  auto_pilot
//=========================================================
void auto_pilot()
{
  unsigned long t;                  // current time
  char buf[256];                    // serial pirnt buffer

  t = millis();                     // get current time
  dTime = t - preTime;                 // diff time
  preTime = t;
  auto_driving();
  auto_steering();
  //=========================================================
  //  Logging
  //=========================================================
#ifdef BT_ON
  sprintf(buf, "\t%8d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%5.2f\t%5.2f\t%1d\t%3d\t%5.3f\t%5.3f\t%3d\t%3d\t%1d\t%1d\t%1d",
                t, rawS0, st0, rawS1, st1, rawS2, st2, p, d, steerDir, dAngle, kp, kd, requestTorque, curSpeed, curDriveDir, courseLayout, dMode);
//  sprintf(buf, "\t%8d\t%4d\t%4d\t%4d\t%5.2f\t%5.2f\t%1d\t%3d\t%5.3f\t%5.3f\t%3d\t%3d\t%1d\t%1d\t%1d",
//                t, rawS0, rawS1, rawS2, p, d, steerDir, dAngle, kp, kd, requestTorque, curSpeed, curDriveDir, courseLayout, dMode);
  SerialBT.println(buf);
#endif
//  SerialBT.println(buf);
}

//=========================================================
//  manual pilot function
//=========================================================
void manual_pilot()
{
  int sDrive;         // dirve power
  int angle;          // steering angle
  int K_OFF = 50;     // min power offset
                      // driving
  if (Joy_Y > 10) {
    sDrive = constrain( Joy_Y + K_OFF, 0, maxSpeed);
    RC_drive(FORWARD, sDrive);
  } else if (Joy_Y < -10) {
    sDrive = constrain(-Joy_Y + K_OFF, 0, maxSpeed);
    RC_drive(REVERSE, sDrive);
  } else {
    RC_drive(BRAKE, 0);
  }
                      // steering
  angle = constrain(Joy_X + Trim, -100, 100);
  if (angle > 0) {
    RC_steer(RIGHT, angle);
  } else if (angle < 0) {
    RC_steer(LEFT, -angle);
  } else {
    RC_steer(CENTER);
  }
}

//=========================================================
//  Arduino Main function
//=========================================================
void loop()
{
  //=========================================================
  //  get VL53L0X TOF sensor value
  //    S0: left S1: center S2: right
  //=========================================================
#ifdef SENSOR_VL53L1X
  rawS0 = sensor0.read();        // read left  sensor
  rawS1 = sensor1.read();        // read front sensor
  rawS2 = sensor2.read();        // read right sensor
  // detect overrange error and collect

  st0 = sensor0.ranging_data.range_status;
  st1 = sensor1.ranging_data.range_status;
  st2 = sensor2.ranging_data.range_status;

   if (st0 != 4)  ps0 = s0 = rawS0;                  // if range error occred set previus value
   else           s0 = ps0;
   if (st1 != 4)  ps1 = s1 = rawS1;
   else           s1 = ps1;
   if (st2 != 4)  ps2 = s2 = rawS2;
   else           s2 = ps2;  

#else
  //s0=sensor0.readRangeContinuousMillimeters();
  //s1=sensor1.readRangeContinuousMillimeters();
  //s2=sensor2.readRangeContinuousMillimeters();
  s0=sensor0.readRangeSingleMillimeters();
  s1=sensor1.readRangeSingleMillimeters();
  s2=sensor2.readRangeSingleMillimeters();
#endif
//=========================================================
//  Bluetooth Serial Control
//=========================================================
  if(SerialBT.available()){
    char action = SerialBT.read();
    if (action == 'a' ){
      if (autoPilot == 0) {
        autoPilot = 1;
#ifdef BT_ON
        SerialBT.println("\tTime\tS0\tst0\tS1\tst1\tS2\tst2\tD\tP\tDIR\tAngle\tKp\tKd\trequestSpeed\tcurSpeed\tcurDriveDIr\tLayout\tdMode");
//        SerialBT.println("\tTime\tS0\tS1\tS2\tD\tP\tDIR\tAngle\tKp\tKd\trequestSpeed\tcurSpeed\tcurDriveDIr\tLayout\tdMode");
#endif
      } else {
        autoPilot = 0;
      }
    } else if (action == 'M') {
      dMode++;
      if (dMode > 3) dMode = 3;
    } else if (action == 'm') {
      dMode--;
      if (dMode < 0) dMode = 0;
    } else if (action == 'S') {
      maxSpeed += 10;
      if (maxSpeed >= 255) maxSpeed = 255;
    } else if (action == 's') {
      maxSpeed -= 10;
      if (maxSpeed <= 100) maxSpeed = 100;
    } else if (action == 'P') {
      kp += 0.025;
    } else if (action == 'p') {
      kp -= 0.025;
      if (kp <= 0) kp = 0;
    } else if (action == 'D') {
      kd += 0.025;
    } else if (action == 'd') {
      kd -= 0.025;
      if (kd <= 0) kd = 0;
    }
  }
  if (autoPilot == 1) {
    auto_pilot();
  } else {
    manual_pilot();
  }
}
