#include <Arduino.h>

#include "DroneConfig.h"
#include "espcopter32.h"

void setup() {
  ble.setup();
  drone_setup();
}

void loop() {
  drone_loop();
}
