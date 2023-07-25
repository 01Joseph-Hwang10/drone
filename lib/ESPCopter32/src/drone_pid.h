#ifndef DRONE_PID_H
#define DRONE_PID_H

#include "global.h"



//0.8 0.01 0.5 a little shaky, on the edge
//0.8 0.01 0.9 a little shaky, good to fly

static int16_t axisPID[3];
static int16_t lastError[3] = {0,0,0};
static float errorGyroI[3] = {0,0,0};
static float errorAngleI[3] = {0,0,0};

//----------PID controller----------
    
#define GYRO_I_MAX 10000.0
#define ANGLE_I_MAX 6000.0

void pid();

void zeroGyroAccI();


#endif