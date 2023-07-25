
#include "drone_pid.h"

int plotct;
int16_t deltab[6][3];
int8_t  deltabpt = 0;
int32_t deltasum[3];

void pid()
{
  uint8_t axis;
  float errorAngle;
  float AngleRateTmp, RateError;
  float PTerm,ITerm,DTerm;
  int16_t delta;

  
      //----------PID controller----------
      for(axis=0;axis<3;axis++) 
      {
        if (axis == 2) 
        { //YAW is always gyro-controlled 
          AngleRateTmp = yawRate * rcCommand[YAW];
          RateError = AngleRateTmp - gyroData[axis];
          PTerm = RateError * P_PID;
          
          delta           = RateError - lastError[axis];  
          lastError[axis] = RateError;
          deltasum[axis] += delta;
          deltasum[axis] -= deltab[deltabpt][axis];
          deltab[deltabpt][axis] = delta;  
          DTerm = deltasum[axis] * D_PID;
         
          ITerm = 0.0;
          
          deltabpt++;
          if (deltabpt >= 6) deltabpt = 0;
        } 
        else 
        {
          if (flightmode == GYRO) // GYRO mode 
          { 
            //control is GYRO based (ACRO - direct sticks control is applied to rate PID
            AngleRateTmp = rollPitchRate * rcCommand[axis];
            RateError = AngleRateTmp - gyroData[axis];
            //-----calculate P-term
            PTerm = RateError * P_PID;
            //-----calculate D-term
            delta           = RateError - lastError[axis];  
            lastError[axis] = RateError;

            deltasum[axis] += delta;
            deltasum[axis] -= deltab[deltabpt][axis];
            deltab[deltabpt][axis] = delta;
            
            DTerm = deltasum[axis] * D_PID;
            //-----calculate I-term
            ITerm = 0.0;
          }
          else // STABI mode
          {
            // calculate error and limit the angle to 45 degrees max inclination
            errorAngle = constrain(rcCommand[axis],-400,+400) - angle[axis]; //16 bits is ok here           
            //it's the ANGLE mode - control is angle based, so control loop is needed
            //-----calculate P-term
            PTerm = errorAngle * P_Level_PID;
            //-----calculate D-term
            delta = - gyroData[axis]; 
            DTerm = delta * D_Level_PID; 
            //-----calculate I-term
            errorAngleI[axis]  += errorAngle * I_Level_PID;
            errorAngleI[axis]  = constrain(errorAngleI[axis], -ANGLE_I_MAX, +ANGLE_I_MAX);
            ITerm = errorAngleI[axis] * 0.01;
          } 
        }
         
        //-----calculate total PID output
        axisPID[axis] =  PTerm + ITerm + DTerm;

        /*
        if (axis==2)
        {
          Serial.print(AngleRateTmp); Serial.print("  ");
          Serial.print(RateError); Serial.print("  ");
          Serial.print(PTerm); Serial.print("  ");
          Serial.println();
        }
        */
        /*
        if (axis==0)
        {
          Serial.print(PTerm); Serial.print("  ");
          Serial.print(ITerm); Serial.print("  ");
          Serial.print(DTerm); Serial.print("  ");
          if      (plotct == 0) Serial.print(-2000); 
          else if (plotct == 1) Serial.print( 2000); 
          else                  Serial.print( 0);
          if (plotct == 300) plotct = 0; else plotct++; 
          Serial.println();
        }
        */     
      }
}

void zeroGyroAccI()
{
  for(int axis=0;axis<3;axis++) 
  {
    errorAngleI[axis] = 0.0;
    errorGyroI[axis] = 0.0;
  } 
}
