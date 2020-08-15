/*
This example shows how to take simple range measurements with the VL53L1X. The
range readings are in units of mm.
*/
#include "M5Atom.h"
#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor0;
VL53L1X sensor1;
VL53L1X sensor2;

// M5.Atom I/O Configuration for VL53L0X Shutdown pin
#define SHUT0 19        // Servo Version only
#define SHUT1 33
#define SHUT2 23

void setup()
{
  M5.begin(true, false, true);
  Serial.begin(115200);
  pinMode(SHUT0, OUTPUT);
  pinMode(SHUT1, OUTPUT);
  pinMode(SHUT2, OUTPUT);
  digitalWrite(SHUT0, LOW);
  digitalWrite(SHUT1, LOW);
  digitalWrite(SHUT2, LOW);
  delay(150);
  Wire.begin(32, 26);
  Wire.setClock(400000); // use 400 kHz I2C
  
  //seonsor0  
  pinMode(SHUT0, INPUT);
  delay(150);
  if (!sensor0.init())
  {
    Serial.println("Failed to detect and initialize sensor0!");
    while (1);
  }
  delay(100);
  sensor0.setAddress((uint8_t)20); // 20
  sensor0.setTimeout(500);      // 500   
  sensor0.setDistanceMode(VL53L1X::Long);
  sensor0.setMeasurementTimingBudget(20000); 
  
  //seonsor1
  pinMode(SHUT1, INPUT);
  delay(150);
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor1!");
    while (1);
  }  
  delay(100);
  sensor1.setAddress((uint8_t)21);
  sensor1.setTimeout(500);
  sensor1.setDistanceMode(VL53L1X::Long);
  sensor1.setMeasurementTimingBudget(20000);
  
  //seonsor2
  pinMode(SHUT2, INPUT);
  delay(150);
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1);
  }
  delay(100);
  sensor2.setAddress((uint8_t)22);
  sensor2.setTimeout(500);
  sensor2.setDistanceMode(VL53L1X::Long);
  sensor2.setMeasurementTimingBudget(20000); 
  
  sensor0.startContinuous(50);
  sensor1.startContinuous(50);
  sensor2.startContinuous(50);
}

void loop()
{
  Serial.print("Sensor0:");
  Serial.print(sensor0.read());
  Serial.print("  Sensor1:");
  Serial.print(sensor1.read());
  Serial.print("  Sensor2:");
  Serial.println(sensor2.read());  
}
