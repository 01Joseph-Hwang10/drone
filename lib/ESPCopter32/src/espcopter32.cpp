#include "espcopter32.h"

volatile boolean recv;
bool interpreter_running = false;
// volatile int peernum = 0;
// esp_now_peer_info_t slave;

void recv_cb(const uint8_t *macaddr, const uint8_t *data, int len)
{
  recv = true;
  // Serial.print("recv_cb ");
  // Serial.println(len);
  if (len == RCdataSize)
  {
    for (int i = 0; i < RCdataSize; i++)
      RCdata.data[i] = data[i];
  }
  /*
  if (!esp_now_is_peer_exist(macaddr))
  {
    Serial.println("adding peer ");
    esp_now_add_peer(macaddr, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);
    peernum++;
  }
  */
};

int debugvalue = 0;

void drone_setup()
{
  Serial.begin(115200);
  Serial.println();

  delay(3000); // give it some time to stop shaking after battery plugin
  MPU6050_init();
  MPU6050_readId(); // must be 0x68, 104dec

  EEPROM.begin(64);
  if (EEPROM.read(63) != 0x55)
    Serial.println("Need to do ACC calib");
  else
    ACC_Read(); // eeprom is initialized
  if (EEPROM.read(62) != 0xAA)
    Serial.println("Need to check and write PID");
  else
    PID_Read(); // eeprom is initialized

  WiFi.mode(WIFI_STA); // Station mode for esp-now
#if defined webServer
  setupwebserver();
  delay(500);
#endif

#if defined externRC
  init_RC();
#elif defined(RC_FROM_INTERPRETER)
  setup_interpreter();
#if defined(FORCE_ARM)
  armed = true;
#endif
#else
  Serial.printf("This mac: %s, ", WiFi.macAddress().c_str());
  Serial.printf(", channel: %i\n", WIFI_CHANNEL);
  if (esp_now_init() != 0)
    Serial.println("*** ESP_Now init failed");
  esp_now_register_recv_cb(recv_cb);
#endif

  delay(500);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  initServo();
}

uint32_t rxt; // receive time, used for falisave

void drone_loop()
{
  // Serial.printf("armed: %d, armct: %d\n", armed, armct);
  uint32_t now, mnow, diff;
  now = millis(); // actual time
  if (debugvalue == 5)
    mnow = micros();

#if defined webServer
  loopwebserver();
#endif

  

#if defined(RC_FROM_INTERPRETER)
  if (load_instructions()) interpreter_running = true;
  
  if (!interpreter_running) return;

  if (buf_to_rc_from_interpreter())
    recv = true;
    // Serial.printf("%4d \n", rcValue[0]);
    
#endif

  if (recv)
  {
    recv = false;

    // #if !defined externRC
    //   buf_to_rc();
    // #endif

    if (debugvalue == 4)
      Serial.printf("%4d %4d %4d %4d \n", rcValue[0], rcValue[1], rcValue[2], rcValue[3]);

    // if      (rcValue[AU1] < 1300) flightmode = GYRO;
    // else                          flightmode = STABI;
    flightmode = GYRO;

    if (oldflightmode != flightmode)
    {
      zeroGyroAccI();
      oldflightmode = flightmode;
    }

    if (armed)
    {
      rcValue[THR] -= THRCORR;
      rcCommand[ROLL] = rcValue[ROL] - MIDRUD;
      rcCommand[PITCH] = rcValue[PIT] - MIDRUD;
      rcCommand[YAW] = rcValue[RUD] - MIDRUD;
    }
    else
    {
      if (rcValue[THR] < MINTHROTTLE)
        armct++;
      if (armct >= 25)
      {
        digitalWrite(LED, HIGH);
        armed = true;
      }
    }

    if (debugvalue == 5)
      Serial.printf("RC input ms: %d\n", now - rxt);
    rxt = millis();
  }

  Gyro_getADC();

  ACC_getADC();

  getEstimatedAttitude();

  pid();

  mix();

  writeServo();

  // // Failsave part
  // if (now > rxt+90)
  // {
  //   rcValue[THR] = MINTHROTTLE;
  //   if (debugvalue == 5) Serial.printf("RC Failsafe after %d \n",now-rxt);
  //   rxt = now;
  // }

  // parser part
  if (Serial.available())
  {
    char ch = Serial.read();
    // Perform ACC calibration
    if (ch == 10)
      Serial.println();
    else if (ch == 'A')
    {
      Serial.println("Doing ACC calib");
      calibratingA = CALSTEPS;
      while (calibratingA != 0)
      {
        delay(CYCLETIME);
        ACC_getADC();
      }
      ACC_Store();
      Serial.println("ACC calib Done");
    }
    else if (ch == 'R')
    {
      Serial.print("Act Rate :  ");
      Serial.print(yawRate);
      Serial.print("  ");
      Serial.print(rollPitchRate);
      Serial.println();
      Serial.println("Act PID :");
      Serial.print(P_PID);
      Serial.print("  ");
      Serial.print(I_PID);
      Serial.print("  ");
      Serial.print(D_PID);
      Serial.println();
      Serial.print(P_Level_PID);
      Serial.print("  ");
      Serial.print(I_Level_PID);
      Serial.print("  ");
      Serial.print(D_Level_PID);
      Serial.println();
    }
    else if (ch == 'D')
    {
      Serial.println("Loading default PID");
      yawRate = 6.0;
      rollPitchRate = 5.0;
      P_PID = 0.15; // P8
      I_PID = 0.00; // I8
      D_PID = 0.08;
      P_Level_PID = 0.35; // P8
      I_Level_PID = 0.00; // I8
      D_Level_PID = 0.10;
      PID_Store();
    }
    else if (ch == 'W')
    {
      char ch = Serial.read();
      int n = Serial.available();
      if (n == 3)
      {
        n = readsernum();
        if (ch == 'p')
        {
          P_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid P ");
          Serial.print(P_PID);
        }
        else if (ch == 'i')
        {
          I_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid I ");
          Serial.print(I_PID);
        }
        else if (ch == 'd')
        {
          D_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid D ");
          Serial.print(D_PID);
        }
        else if (ch == 'P')
        {
          P_Level_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid Level P ");
          Serial.print(P_Level_PID);
        }
        else if (ch == 'I')
        {
          I_Level_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid Level I ");
          Serial.print(I_Level_PID);
        }
        else if (ch == 'D')
        {
          D_Level_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid Level D ");
          Serial.print(D_Level_PID);
        }
        else
          Serial.println("unknown command");
      }
      else if (ch == 'S')
      {
        PID_Store();
        Serial.print("stored in EEPROM");
      }
      else
      {
        Serial.println("Input format wrong");
        Serial.println("Wpxx, Wixx, Wdxx - write gyro PID, example: Wd13");
        Serial.println("WPxx, WIxx, WDxx - write level PID, example: WD21");
      }
    }
    else if (ch >= '0' && ch <= '9')
      debugvalue = ch - '0';
    else
    {
      Serial.println("A - acc calib");
      Serial.println("D - write default PID");
      Serial.println("R - read actual PID");
      Serial.println("Wpxx, Wixx, Wdxx - write gyro PID");
      Serial.println("WPxx, WIxx, WDxx - write level PID");
      Serial.println("WS - Store PID in EEPROM");
      Serial.println("Display data:");
      Serial.println("0 - off");
      Serial.println("1 - Gyro values");
      Serial.println("2 - Acc values");
      Serial.println("3 - Angle values");
      Serial.println("4 - RC values");
      Serial.println("5 - Cycletime");
    }
  }

  if (debugvalue == 1)
    Serial.printf("%4d %4d %4d \n", gyroADC[0], gyroADC[1], gyroADC[2]);
  else if (debugvalue == 2)
    Serial.printf("%5d %5d %5d \n", accADC[0], accADC[1], accADC[2]);
  else if (debugvalue == 3)
    Serial.printf("%3f %3f \n", angle[0], angle[1]);
  else if (debugvalue == 4)
    Serial.printf("%d %d %d %d \n", rcValue[0], rcValue[1], rcValue[2], rcValue[3]);
  delay(CYCLETIME - 1);

  if (debugvalue == 5)
  {
    diff = micros() - mnow;
    Serial.println(diff);
  }
}

int readsernum()
{
  int num;
  char numStr[3];  
  numStr[0] = Serial.read();
  numStr[1] = Serial.read();
  return atol(numStr);
}
