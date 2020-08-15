//=========================================================
//  RumiCar.h :  RumiCar Library Header for M5.Atom 
//  History    : V0.0  2020-05-29 New Create(K.Ohe)
//             : V1.0  2020-08-09 change VL53L1X Range
//=========================================================
#define RUMICAR_H
#define SENSOR_VL53L1X
#define ATOM_MATRIX
//#define ALGYAN_ESP32
#define VL53L1X_RANGE   VL53L1X::Long       // Short Long Medium
#define VL53L1X_TB      33000               // Short :20000
#ifdef SENSOR_VL53L1X
#include <VL53L1X.h>
#else
#include <VL53L0X.h>
#endif
// RumiCar include
#define SERVO   1       // Servo Control Version

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

// M5.Atom I/O Configuration for VL53L0X Shutdown pin
#define SHUT0 19        // Servo Version only
#define SHUT1 33
#define SHUT2 23

// Prototype definition
EXTERN void RC_setup(void);
EXTERN int  RC_steer(int  direc, int angle = 50);
EXTERN int  RC_drive(int, int);
EXTERN void RC_delay(unsigned long tim);
EXTERN void RC_run(void);

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

// M5.Atom LED definition
extern const unsigned char img_rumicar[677];
enum LED_DIR {
  DR_ARROW,
  RIGHT_ARROW,
  UR_ARROW,
  DOWN_ARROW,
  STOP,
  UP_ARROW,
  DL_ARROW,
  LEFT_ARROW,
  UL_ARROW
};

//=========================================================
// M5.Atom 5x5 Led Control class
//=========================================================
class DispLed {
private:
  int state_matrix[4][3] = {
    {RIGHT_ARROW, STOP,       LEFT_ARROW},  // Free
    {DR_ARROW,    DOWN_ARROW, DL_ARROW  },  // Reverse
    {UR_ARROW,    UP_ARROW,   UL_ARROW  },  // Forward
    {RIGHT_ARROW, STOP,       LEFT_ARROW}   // BREAK
  };
public:
  int x = 1;
  int y = 3;
  void show(void) {
    int dir = state_matrix[y][x];
    M5.dis.displaybuff((uint8_t *)img_rumicar, dir * 5, 0);
  }
};
