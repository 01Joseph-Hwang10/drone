#ifndef GLOBAL_H
#define GLOBAL_H
#include "Arduino.h"
#include "DroneBLE.h"

#ifdef flysky
  #define ROL 0
  #define PIT 1
  #define THR 2
  #define RUD 3
#else //orangerx
  #define ROL 1
  #define PIT 2
  #define THR 0
  #define RUD 3
#endif

#define AU1 4
#define AU2 5

#define ACCRESO 4096
#define CYCLETIME 3
#define MINTHROTTLE 1090
#define MIDRUD 1495
#define THRCORR 19

#define WIFI_CHANNEL 4
#define PWMOUT  // normal esc, uncomment for serial esc
#define LED 2
#define CALSTEPS 256 // gyro and acc calibration steps
//#define externRC // use of external RC receiver in ppmsum mode
//#define webServer // use of webserver to change PID

#define GYRO     0
#define STABI    1

#define CHANNELS 8
#define RC_IN_PIN 27 
#define RC_IN_GPIO GPIO_NUM_27

extern DroneBLE ble;

enum ang { ROLL,PITCH,YAW };

extern int16_t rcCommand[];

extern int16_t gyroADC[3];
extern int16_t accADC[3];
extern int16_t gyroData[3];

extern float angle[2];
extern int calibratingA;

extern int8_t flightmode;
extern int8_t oldflightmode;

extern int16_t accZero[3];
extern float yawRate;
extern float rollPitchRate;
extern float P_PID;
extern float I_PID;
extern float D_PID;
extern float P_Level_PID;
extern float I_Level_PID;
extern float D_Level_PID;

extern boolean armed;
extern uint8_t armct;

extern volatile uint16_t rcValue[CHANNELS];  // in us, center = 1500

#endif