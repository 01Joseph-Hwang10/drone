#include "drone_mpu6050.h"

int calibratingG = CALSTEPS;
int calibratingA = 0;
int16_t gyroZero[3] = {0,0,0};
int16_t accZero[3] = {0,0,0};
uint8_t rawADC[6];

void i2cRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) 
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  Wire.requestFrom(Address, Nbytes);
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

void i2cWriteByte(uint8_t Address, uint8_t Register, uint8_t Data) 
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// ****************
// GYRO common part
// ****************
void GYRO_Common() 
{
  static int32_t g[3];
  uint8_t axis, tilt=0;

  if (calibratingG>0) 
  {
    for (axis = 0; axis < 3; axis++) 
    {
      // Reset g[axis] at start of calibration
      if (calibratingG == CALSTEPS) g[axis]=0;
      // Sum up 512 readings
      g[axis] += gyroADC[axis];
      // Clear global variables for next reading
      gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) 
      {
        if (g[axis]>=0) g[axis] += CALSTEPS/2;
        else            g[axis] -= CALSTEPS/2;
        gyroZero[axis] = g[axis]/CALSTEPS;
      }
    }
    calibratingG--;
  }

  for (axis = 0; axis < 3; axis++)
    gyroADC[axis] = gyroADC[axis] - gyroZero[axis];
}

// ****************
// ACC common part
// ****************

void ACC_Common() 
{
  static int32_t a[3];
  
  if (calibratingA>0) 
  {
    for (uint8_t axis = 0; axis < 3; axis++) 
    {
      // Reset a[axis] at start of calibration
      if (calibratingA == CALSTEPS) a[axis]=0;
      // Sum up 512 readings
      a[axis] += accADC[axis];
      // Clear global variables for next reading
      accADC[axis]=0;
      accZero[axis]=0;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) 
    {
      if (a[0] >= 0) a[0] += CALSTEPS/2; else a[0] -= CALSTEPS/2;
      if (a[1] >= 0) a[1] += CALSTEPS/2; else a[1] -= CALSTEPS/2;
      if (a[2] >= 0) a[2] += CALSTEPS/2; else a[2] -= CALSTEPS/2;
      accZero[0] = a[0]/CALSTEPS;
      accZero[1] = a[1]/CALSTEPS;
      accZero[2] = a[2]/CALSTEPS - ACCRESO;   
      Serial.print("  "); Serial.print(accZero[0]); Serial.println();
      Serial.print("  "); Serial.print(accZero[1]); Serial.println();
      Serial.print("  "); Serial.print(accZero[2]); Serial.println();
    }
    calibratingA--;
  }
  accADC[0] -=  accZero[0];
  accADC[1] -=  accZero[1];
  accADC[2] -=  accZero[2];
}

void Gyro_getADC () 
{
  i2cRead(MPU6050_ADDRESS, 0x43,6,rawADC);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) , 
                    ((rawADC[2]<<8) | rawADC[3]) ,
                    ((rawADC[4]<<8) | rawADC[5]) );
  GYRO_Common();
}

void ACC_getADC () 
{
  i2cRead(MPU6050_ADDRESS, 0x3B,6,rawADC);
  ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                   ((rawADC[2]<<8) | rawADC[3]) ,
                   ((rawADC[4]<<8) | rawADC[5]) );
  ACC_Common();
}

void MPU6050_readId() 
{
  uint8_t id;
  i2cRead(MPU6050_ADDRESS, 0x75, 1, &id);
  if (id == 0x68) Serial.println("6050 ID OK");
  else Serial.println("6050 ID Failed");
  Serial.println(id);
}

void MPU6050_init()
{
  Wire.begin();
  Wire.setClock(400000);

  //Gyro_init
  i2cWriteByte(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(50);
  
  i2cWriteByte(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2cWriteByte(MPU6050_ADDRESS, 0x1A, DLPF_CFG);         //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2cWriteByte(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  delay(50);
  
  //ACC_init 
  i2cWriteByte(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG 
  delay(50);
}
