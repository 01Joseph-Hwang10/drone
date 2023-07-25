#ifndef DRONE_EEPROM_H
#define DRONE_EEPROM_H

#include <Arduino.h>
#include <EEPROM.h>
#include "global.h"

typedef union int16_ty
{
  int16_t d;
  byte    b[2];
} int16_ty;

typedef union float_ty
{
  float d;
  byte  b[4];
} float_ty;

void write_int16(int pos, int16_t d);
int16_t read_int16(int pos);
void write_float(int pos, float d);
float read_float(int pos);
void commit();
void ACC_Read();
void ACC_Store();
void PID_Read();
void PID_Store();

#endif