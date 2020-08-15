#include "M5Atom.h"

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
 
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
 
float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;
 
int mode = -1;
 
void setup() {
  M5.begin();
  M5.IMU.Init();
}
 
void loop() {
  M5.update();
 
  // データ取得
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
 
  // モードチェンジ
  if ( mode == -1 || M5.Btn.wasReleased() ) {
    mode++;
    mode = mode % 3;
 
    // プロッタ用のタイトル出力
    if ( mode == 0 ) {
      Serial.printf("gyroX,gyroY,gyroZ\n");
    } else if ( mode == 1 ) {
      Serial.printf("accX,accY,accZ\n");
    } else if ( mode == 2 ) {
      Serial.printf("pitch,roll,yaw\n");
    }
  }
 
  // データ出力
  if ( mode == 0 ) {
    Serial.printf("%6.2f,%6.2f,%6.2f\n", gyroX, gyroY, gyroZ);
  } else if ( mode == 1 ) {
    Serial.printf("%5.2f,%5.2f,%5.2f\n", accX, accY, accZ);
  } else if ( mode == 2 ) {
    Serial.printf("%5.2f,%5.2f,%5.2f\n", pitch, roll, yaw);
  }
 
  delay(10);
}
