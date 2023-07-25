#include "global.h"

DroneBLE ble;

int16_t rcCommand[] = {0,0,0};

int16_t gyroADC[3] = {0, 0, 0};
int16_t accADC[3] = {0, 0, 0};
int16_t gyroData[3] = {0, 0, 0};

float angle[2]    = {0,0};  

int8_t flightmode = 0;
int8_t oldflightmode = 0;

// Note: this PID settings were multiply defined in both EspCopter32.ino and pid.ino.
// If there is some problem in setting PID, please check the original project file.
float P_Level_PID = 0.40;   // P8
float I_Level_PID = 0.01;   // I8
float D_Level_PID = 0.05;   // D8

float P_PID = 0.14;    // P8
float I_PID = 0.00;    // I8
float D_PID = 0.08;    // D8

float yawRate = 5.0;
float rollPitchRate = 5.0;

// until here

boolean armed = false;
uint8_t armct = 0;

volatile uint16_t rcValue[CHANNELS]; 