// Rotates the mask forward or backward and notes all hall switch transition
// positions.

#include <Arduino.h>
#include "bipolar_stepper.h"
#include "hall_switch.h"
#include "mask_controller.h"
#include "index_task.h"
#include "stepper_controller.h"
#include "timer_one.h"

// Serial config
const int SERIAL_BAUD_RATE = 19200;
const int SERIAL_TIMEOUT_MS = 10;  // [ms]

// Motor/mask config
const int BRKA_PIN = 9;
const int DIRA_PIN = 12;
const int PWMA_PIN = 3;
const int BRKB_PIN = 8;
const int DIRB_PIN = 13;
const int PWMB_PIN = 11;
const float GEAR_RATIO = 72.0/17.0;
const int16_t MOTOR_STEPS = 200u;  // Motor steps per revolution
uint32_t STEP_PERIOD_US = 8000u;  // [us]
const MaskController::Direction PREFERRED_DIRECTION =
    MaskController::Direction::AUTO;

// Hall switch config
const int HALL_SWITCH_POWER_PIN = 4;
const int HALL_SWITCH_STATE_PIN = 5;

// Objects, state variables, etc.
BipolarStepper stepper(BRKA_PIN, DIRA_PIN, PWMA_PIN, BRKB_PIN, DIRB_PIN, PWMB_PIN);
HallSwitch hall_switch(HALL_SWITCH_POWER_PIN, HALL_SWITCH_STATE_PIN);
StepperController motor_controller(&stepper, MOTOR_STEPS);
MaskController mask_controller(&motor_controller, GEAR_RATIO);
IndexTask index_task(&mask_controller, &hall_switch);
TimerOne timer;

enum State {
  START,
  RUNNING,
  DONE
} state = START;

bool ready_to_index = true;
int trials_complete = 0;
const int NUM_TRIALS = 3;
bool finished = false;
bool has_zeroed = false;
unsigned long index_time_ms = 0u;

bool hall_state = false;
unsigned long start_stamp_ms = 0u;
const int RUN_FOR_MS = 10000u;

// Called once at the start of the progrom; initializes all hardware and tasks.
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);
  stepper.initialize();
  stepper.enable();
  hall_switch.init();
  // index_task.init();
  // index_task.setIndexEventCallback(&actOnIndexEvent);
  timer.initialize();
  timer.attachInterrupt(update, STEP_PERIOD_US);
}

void loop() {
  switch (state) {
    case START:
      if (Serial.available()) {
        const char command = Serial.read();
        if (command == 'f') {
          hall_switch.setPowerState(true);
          mask_controller.forward();
          start_stamp_ms = millis();
          state = RUNNING;
        } else if (command == 'b') {
          hall_switch.setPowerState(true);
          mask_controller.reverse();
          start_stamp_ms = millis();
          state = RUNNING;
        }
      }
      break;
    case RUNNING: {
      const int delta_ms = millis() - start_stamp_ms;
      if (!hall_state && hall_switch.isTriggered()) {
        const float pos_deg = mask_controller.getPositionDeg(false);
        hall_state = true;
        printStuff(delta_ms, hall_state, pos_deg);
      } else if (hall_state && !hall_switch.isTriggered()) {
        const float pos_deg = mask_controller.getPositionDeg(false);
        hall_state = false;
        printStuff(delta_ms, hall_state, pos_deg);
      }

      if (delta_ms > RUN_FOR_MS) {
        mask_controller.stop();
        hall_switch.setPowerState(false);
        state = DONE;
      }
      break;
    }
    case DONE:
    default:
      mask_controller.stop();
      hall_switch.setPowerState(false);
      state = START;
      break;
  }
}

void printStuff(const int delta_ms, const bool hall_state, const float pos_deg) {
  Serial.print(delta_ms);
  Serial.print("\t");
  if (hall_state) {
    Serial.print("HIGH\t");
  } else {
    Serial.print("LOW\t");
  }
  Serial.println(pos_deg);
}

// Function run via timer interrupt to actuate motor.
void update() {
  motor_controller.update();
}
