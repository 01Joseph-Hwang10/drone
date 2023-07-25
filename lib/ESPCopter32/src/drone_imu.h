#ifndef DRONE_IMU_H
#define DRONE_IMU_H
#include <Arduino.h>
#include "global.h"
#include "drone_mpu6050.h"

enum cart { X,Y,Z };

#define GYRO_SCALE ((1998 * PI)/(32767.0f * 180.0f * 1000.0f)) // MPU6050
#define F_GYRO_SCALE 0.001 * GYRO_SCALE // in usec

/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#define GYR_CMPF_FACTOR 90
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))

typedef struct fp_vector 
{		
  float X,Y,Z;		
} t_fp_vector_def;

typedef union 
{		
  float A[3];		
  t_fp_vector_def V;		
} t_fp_vector;


float InvSqrt (float x);
// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta);

void getEstimatedAttitude();


#endif