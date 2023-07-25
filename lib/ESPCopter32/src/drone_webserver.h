#ifndef DRONE_WEBSERVER_H
#define DRONE_WEBSERVER_H

#include <WebServer.h>
#include "drone_eeprom.h"
#include "global.h"

#ifdef webServer

//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot();

void handleGyro();

void handleLevel();

//===============================================================
// This routine is executed when you press submit
//===============================================================

void action_pid();
void action_level();


//==============================================================
//                  SETUP
//==============================================================
void setupwebserver(void);

//==============================================================
//                     LOOP
//==============================================================
void loopwebserver(void);
void stopwebserver(void);

#endif

#endif