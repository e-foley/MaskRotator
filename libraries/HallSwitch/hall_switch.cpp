#include "hall_switch.h"
#include <Arduino.h>

HallSwitch::HallSwitch(const int power_pin, const int state_pin) : power_pin_(power_pin),
    state_pin_(state_pin), is_initialized_(false) {}

void HallSwitch::init() {
  pinMode(power_pin_, OUTPUT);
  digitalWrite(power_pin_, LOW);
  pinMode(state_pin_, INPUT);
  is_initialized_ = true;
}

bool HallSwitch::isInitialized() const {
  return is_initialized_;
}

void HallSwitch::setPowerState(bool power_state) {
  if (!is_initialized_) {
    return;
  }

  digitalWrite(power_pin_, power_state);
}

bool HallSwitch::isTriggered() const {
  if (!is_initialized_) {
    return false;
  }

  return !digitalRead(state_pin_);
}
