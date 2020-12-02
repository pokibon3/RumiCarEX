//=========================================================
//  RumiCar_esp32.cpp :  RumiCar Library for M5.Atom
//  History     : V1.0  2020-08-18 New Create(K.Ohe)
//=========================================================
#include <Wire.h>
#include <ESP32Servo.h>
//#include "ESP32TimerInterrupt.h"
#include <Ticker.h>
#define EXTERN
#ifndef RUMICAR_ESP32_H
#include "RumiCar_esp32.h"
#endif

// Private Variable for timmer task
#define   UP_SPEED      (200)       // StartUP speed
#define   UP_TIME       (5)         // 50[ms]
float timer_interval = 0.01;  //seconds
Ticker Timer1;

volatile  int s_speed = 0;
volatile  int s_gear = FREE;
volatile  int s_steer = CENTER;
volatile  int s_steera = 255;
volatile  int s_sptim = 0;

//=========================================================
//  Variable definition
//=========================================================

//Aが操舵、Bが走行       // Servo Version only use A
int AIN1 = 04;      // common with SHUT0
int AIN2 = 26;
int BIN1 = 27;
int BIN2 = 25;
int SERVO_PIN = 32;
Servo Steer_servo;  // create servo object to control a servo
#define SERVO_TRIM 0           // typ 16
#define SERVO_CENTER 90
#ifndef K989
#define SERVO_LEFT   20        // max 60
#define SERVO_RIGHT 160        // max 120
#else
#define SERVO_LEFT  160        // max 60
#define SERVO_RIGHT  20         // max 120
#endif
//=========================================================
// RC_delay:PWM 周波数変更しているためTimer0を調整
//=========================================================
void RC_delay(unsigned long tim)
{
#if defined ESP32
  delay(tim);
#else
  delay(tim / 4);
#endif
}

//=========================================================
//  RC_setup    :  RumiCar setup function
//=========================================================
void RC_setup()
{
  delay(50);
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(SHUT0, OUTPUT);
  pinMode(SHUT1, OUTPUT);
  pinMode(SHUT2, OUTPUT);
  digitalWrite(SHUT0, LOW);
  digitalWrite(SHUT1, LOW);
  digitalWrite(SHUT2, LOW);

  delay(150);
  Wire.begin(21, 22);

  pinMode(SHUT0, INPUT);
  delay(150);
  sensor0.init(true);
  delay(100);
  sensor0.setAddress((uint8_t)20); // 20
  sensor0.setTimeout(500);      // 500
  //seonsor1
  pinMode(SHUT1, INPUT);
  delay(150);
  sensor1.init(true);
  delay(100);
  sensor1.setAddress((uint8_t)21);
  sensor1.setTimeout(500);
  //seonsor2
  pinMode(SHUT2, INPUT);
  delay(150);
  sensor2.init(true);
  delay(100);
  sensor2.setAddress((uint8_t)22);
  sensor2.setTimeout(500);

  // reduce timing budget to 20 ms (default is about 33 ms)
#ifdef SENSOR_VL53L1X
  sensor0.VL53L1X_SetROI(16 , 8, 60);        // default 60
  sensor1.VL53L1X_SetROI(16 , 8, 60);        // def 60
  sensor2.VL53L1X_SetROI(16 , 8, 60);        // def 60
//  sensor0.setDistanceMode(VL53L1X::Medium);
//  sensor1.setDistanceMode(VL53L1X::Medium);
//  sensor2.setDistanceMode(VL53L1X::Medium);
  sensor0.setDistanceMode(VL53L1X_RANGE);
  sensor1.setDistanceMode(VL53L1X_RANGE);
  sensor2.setDistanceMode(VL53L1X_RANGE);
  sensor0.setMeasurementTimingBudget(VL53L1X_TB);
  sensor1.setMeasurementTimingBudget(VL53L1X_TB);
  sensor2.setMeasurementTimingBudget(VL53L1X_TB);
  sensor0.startContinuous(5);
  sensor1.startContinuous(5);
  sensor2.startContinuous(5);
#else
  sensor0.setMeasurementTimingBudget(20000);
  sensor1.setMeasurementTimingBudget(20000);
  sensor2.setMeasurementTimingBudget(20000);
  sensor0.startContinuous();
  sensor1.startContinuous();
  sensor2.startContinuous();
#endif
  //ESP32の場合はピン番号ではなくチャンネルでPWMを行うのでチャンネルとして再設定
#define PWM_level 8
  // 8の場合8bitの解像度でArduinoと同じESPは16bit迄行ける？
  //モータのPWMのチャンネル、周波数の設定
  ledcSetup(0, 490, PWM_level);
  ledcSetup(1, 490, PWM_level);
  ledcSetup(2, 240, PWM_level);  // 960
  ledcSetup(3, 240, PWM_level);  // 960
  ledcAttachPin(AIN1, 0);
  ledcAttachPin(AIN2, 1);
  ledcAttachPin(BIN1, 2);
  ledcAttachPin(BIN2, 3);
  AIN1 = 0;
  AIN2 = 1;
  BIN1 = 2;
  BIN2 = 3;
  RC_analogWrite(AIN1, 0);
  RC_analogWrite(AIN2, 0);
  RC_analogWrite(BIN1, 0);
  RC_analogWrite(BIN2, 0);

  Steer_servo.setPeriodHertz(50);           // standard 50 hz servo
  Steer_servo.attach(SERVO_PIN, 500, 2500);  // attaches the servo on pin 25 to the servo object

  //----  Timer task start
  Timer1.attach(timer_interval, RC_run);
}

//=========================================================
//  RC_steer    :  steering control function
//    direc     :  derection
//    angle     :  0 - 100
//=========================================================
int RC_steer (int direc, int angle )
{
  int steer_angle = 0;
  if (angle < 0) angle = 0;
  else if (angle > 100) angle = 100;

  if ( direc == RIGHT ){
#ifndef SERVO
    RC_analogWrite(AIN1,255); // 255
    RC_analogWrite(AIN2,0);
#else
    steer_angle = map(angle, 0, 100, SERVO_CENTER, SERVO_RIGHT);
    Steer_servo.write(steer_angle + SERVO_TRIM);
#endif
  }else if ( direc == LEFT ){
#ifndef SERVO
    RC_analogWrite(AIN1,0);
    RC_analogWrite(AIN2,255);
#else
    steer_angle = map(angle, 0, 100, SERVO_CENTER, SERVO_LEFT);
    Steer_servo.write(steer_angle + SERVO_TRIM);
#endif
  }else if ( direc == CENTER ){
#ifndef SERVO
    RC_analogWrite(AIN1,0);
    RC_analogWrite(AIN2,0);
#else
    Steer_servo.write(SERVO_CENTER + SERVO_TRIM);
#endif
  }
/*
  Serial.print("  logical_angle:");
  Serial.print(angle);
  Serial.print("  steer_angle:");
  Serial.println(steer_angle);
*/
  return 0;
}

//=========================================================
//  RC_drive    :  drive control function
//    direc     :  derection
//    ipwm      :  0 - 255
//=========================================================
int RC_drive(int direc, int ipwm){
  //-- StartUP check
  if((s_speed == 0) ||
    (s_gear == FREE) ||
    (s_gear == BRAKE))
  {
      s_sptim = UP_TIME;
  }
  //-- No need StartUP
  if(ipwm > UP_SPEED){
      s_sptim = 0;
  }
  // Save
  s_speed = ipwm;
  s_gear = direc;

}

//=========================================================
//  RC_drive    :  drive control function
//    direc     :  derection
//    ipwm      :  0 - 255
//=========================================================
void RC_run(void)
{
  if ( s_gear == FREE ){
    RC_analogWrite(BIN1,0);
    RC_analogWrite(BIN2,0);
  }else if ( s_gear == REVERSE ){
    if(s_sptim == 0){
      RC_analogWrite(BIN1,0);
      RC_analogWrite(BIN2,s_speed);
    }else{
      RC_analogWrite(BIN1,0);
      RC_analogWrite(BIN2,UP_SPEED);
    }
  }else if ( s_gear == FORWARD ){
    if(s_sptim == 0){
      RC_analogWrite(BIN1,s_speed);
      RC_analogWrite(BIN2,0);
    }else{
      RC_analogWrite(BIN1,UP_SPEED);
      RC_analogWrite(BIN2,0);
    }
  }else if ( s_gear == BRAKE ){
    RC_analogWrite(BIN1,s_speed);
    RC_analogWrite(BIN2,s_speed);
  }
//--  Timer
  if(s_sptim != 0){
    s_sptim --;
  }
}
