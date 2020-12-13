//=========================================================
//  RumiCar_esp32.h :  RumiCar Library Header for M5.Atom 
//  History    : V1.0  2020-08-18 New Create(K.Ohe)
//               V1.1  2020-12-04 change to K989
//=========================================================
#define RUMICAR_ESP32_H
//#define SENSOR_VL53L1X
#define VL53L1X_RANGE   VL53L1X::Long       // Short Long Medium
#define VL53L1X_TB      33000               // Short :20000
#define VL53L1X_POV_X   10
#define VL53L1X_POV_Y   8
#define VL53L1X_POV_CENTER  60
#ifdef SENSOR_VL53L1X
#include <VL53L1X.h>
#else
#include <VL53L0X.h>
#endif
#define K989

// RumiCar include

//操舵用の設定
#define LEFT   0
#define CENTER 1
#define RIGHT  2

//走行用の設定
#define FREE    0
#define REVERSE 1
#define FORWARD 2
#define BRAKE   3

// ESP32 PWM function
#define RC_analogWrite ledcWrite

// ESP32 I/O Configuration for VL53L0X Shutdown pin
#define SHUT0 19        // Servo Version only
#define SHUT1 18
#define SHUT2 05

// Prototype definition
EXTERN void RC_setup(void);
EXTERN int  RC_steer(int  direc, int angle = 50);
EXTERN int  RC_drive(int, int);
EXTERN void RC_delay(unsigned long tim);
EXTERN void RC_run(void);
EXTERN void RC_halt(void);

// TOF Sensor definition
#ifdef SENSOR_VL53L1X
extern VL53L1X sensor0;
extern VL53L1X sensor1;
extern VL53L1X sensor2;
#else
extern VL53L0X sensor0;
extern VL53L0X sensor1;
extern VL53L0X sensor2;
#endif


#define HIGH_SPEED
//#define HIGH_ACCURACY
//#define LONG_RANGE

