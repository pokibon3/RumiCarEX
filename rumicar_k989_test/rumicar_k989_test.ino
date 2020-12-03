//=========================================================
//  rumicar_esp32.ino :  RumiCar application
//  History     : V1.0  2020-08-18 new create 
//=========================================================
#include <Wire.h>
#define  EXTERN extern
#include "RumiCar_esp32.h"

//=========================================================
//  RumiCar Default Parameter
//=========================================================
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
static int s0, s1, s2;            // left, center, right sensor value
static int st0, st1, st2;         // sensor status

void sensor_print(void)
{
  Serial.print("\tSensor0 : ");
  Serial.print(s0);
  Serial.print("\tstatus : ");
  Serial.print(st0);
  Serial.print("\tSensor1 : ");
  Serial.print(s1);
  Serial.print("\tstatus : ");
  Serial.print(st1);
  Serial.print("\tSensor2 : ");
  Serial.print(s2);  
  Serial.print("\tstatus : ");
  Serial.print(st2);
  Serial.println();
}
//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
  RC_setup();
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
  s0 = sensor0.read();        // read left  sensor
  s1 = sensor1.read();        // read front sensor
  s2 = sensor2.read();        // read right sensor

  st0 = sensor0.ranging_data.range_status;
  st1 = sensor1.ranging_data.range_status;
  st2 = sensor2.ranging_data.range_status;

#else
  s0=sensor0.readRangeContinuousMillimeters();
  s1=sensor1.readRangeContinuousMillimeters();
  s2=sensor2.readRangeContinuousMillimeters();
  //s0=sensor0.readRangeSingleMillimeters();
  //s1=sensor1.readRangeSingleMillimeters();
  //s2=sensor2.readRangeSingleMillimeters();
#endif

  sensor_print();
  //s0 = s2 = 0;  // for debug

  if (abs(s0 - s2) < 50) {
    RC_steer(CENTER);
  } else if(s0>s2){
     RC_steer(LEFT, 100);
  }else{
    RC_steer(RIGHT, 100);
  }

  if(s1<150){
    RC_drive(BRAKE,255);
  }else if (s1<200){
    RC_drive(FORWARD,120);
  }else if (s1<300){
    RC_drive(FORWARD,160);
  }else{
    RC_drive(FORWARD,200);
  }

}
