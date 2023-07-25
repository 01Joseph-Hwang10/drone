#ifndef DRONE_MPU6050_H
#define DRONE_MPU6050_H

#include "global.h"

#include <Wire.h>

#define MPU6050_ADDRESS 0x68 // 0x52//0x68 
#define SMPLRT_DIV 0
#define DLPF_CFG   4

#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}

#define ADDRESS 0x03
extern int calibratingG;
extern int calibratingA;
extern int16_t gyroZero[3];
extern int16_t accZero[3];
extern uint8_t rawADC[6];

void i2cRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);

void i2cWriteByte(uint8_t Address, uint8_t Register, uint8_t Data);



// ****************
// GYRO common part
// ****************
void GYRO_Common();

// ****************
// ACC common part
// ****************
void ACC_Common();


void Gyro_getADC ();

void ACC_getADC ();

void MPU6050_readId();

void MPU6050_init();

#endif