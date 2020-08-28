//=========================================================
//  rumicar_LiDAR.ino :  RumiCar application
//  History     : V1.0  2020-08-28 support LiDAR 
//=========================================================
#include "M5Atom.h"               // CPU: M5 Atom Matrix
#include "BluetoothSerial.h"

#define BT_ON
#ifdef BT_ON
BluetoothSerial SerialBT;
#endif
//=========================================================
//  RumiCar Default Parameter
//=========================================================
#define DEVICE_NAME     "RumiCar_LiDAR" // BLE Device Name

//=========================================================
//  auto pilot variables difinition
//=========================================================

//=========================================================
//  Arduino setup function
//=========================================================
void setup()
{
  M5.begin(true, false, true);
  delay(50);
  M5.dis.clear();
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 32, 26);// Grove 32:RX 26:TX
#ifdef BT_ON
  Serial.println("Start bluetooth!");
  SerialBT.begin(DEVICE_NAME);
  Serial.println("The device started, now you can pair it with bluetooth!");
#endif
}

//=========================================================
//  Arduino Main function
//=========================================================
void loop()
{
  char action;
  M5.update();

  if (Serial2.available()) {
    // Serial2(Grove) to Serial(PC)
    int inByte = Serial2.read();
    SerialBT.write(inByte);
  }
}
