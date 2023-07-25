#ifndef ESPCOPTER32_H
#define ESPCOPTER32_H

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>
#include "drone_eeprom.h"
#include "drone_imu.h"
#include "drone_pid.h"
#include "drone_rc.h"
#include "drone_webserver.h"
#include "drone_interpreter.h"


#include "global.h"

void recv_cb(const uint8_t *macaddr, const uint8_t *data, int len);

void drone_setup();
void drone_loop();

int readsernum();

#endif