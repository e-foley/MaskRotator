#include <Arduino.h>
#include "hall_switch.h"

HallSwitch hall_switch(4, 5);
bool last_state = false;

void setup() {
  Serial.begin(19200);
  hall_switch.init();
  hall_switch.setPowerState(HIGH);
}

void loop() {
  if (hall_switch.isTriggered() && !last_state) {
    last_state = true;
    Serial.println("HIGH!");
  } else if (!hall_switch.isTriggered() && last_state) {
    last_state = false;
    Serial.println("LOW!");
  }
}
