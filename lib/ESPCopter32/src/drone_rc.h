// Spec of RC data
#ifndef DRONE_RC_H
#define DRONE_RC_H

#include "global.h"


typedef struct 
{
  uint16_t Ch1     : 11; 
  uint16_t Ch2     : 11;
  uint16_t Ch3     : 11;
  uint16_t Ch4     : 11;
  uint16_t Ch5     : 11;
  uint16_t Ch6     : 11;
  uint16_t Ch7     : 11;
  uint16_t Ch8     : 11;
  uint8_t spare    :  8;
}Payload;

#define RCdataSize 12

typedef union
{
  Payload chans;
  uint8_t data[RCdataSize];
} RCdataTY;

extern RCdataTY RCdata;

extern volatile uint16_t rcValue[CHANNELS];  // in us, center = 1500
extern uint8_t seqno;



#if defined externRC
//------------------------------------------------
  IRAM_ATTR void rxInt();
  
//------------------------------------------------
#endif

void buf_to_rc();

void mix();

#if defined PWMOUT //----------------------------------------------

void writeServo();

void initServo();

#else //----------------------------------------------


void writeServo();
void initServo();

#endif //----------------------------------------------

#endif