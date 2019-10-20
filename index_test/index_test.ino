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
bool ready_to_index = true;
int trials_complete = 0;
const int NUM_TRIALS = 3;
bool finished = false;
bool has_zeroed = false;
unsigned long index_time_ms = 0u;

enum State {
  START,
  INDEXING,
  PAUSING,
  DONE
} state = START;

// Called once at the start of the progrom; initializes all hardware and tasks.
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);
  stepper.initialize();
  stepper.enable();
  hall_switch.init();
  index_task.init();
  index_task.setIndexEventCallback(&actOnIndexEvent);
  timer.initialize();
  timer.attachInterrupt(update, STEP_PERIOD_US);
}

void loop() {
  index_task.step();

  switch (state) {
    case START:
      if (ready_to_index && Serial.available() && Serial.read() == 'g') {
        index_task.index();
        ready_to_index = false;
        state = INDEXING;
      }
      break;
    case INDEXING:
      // ready_to_index flag is raised when the index task has found a position
      if (ready_to_index) {
        index_time_ms = millis();
        state = PAUSING;
      }
      break;
    case PAUSING:
      // We wait a little bit for the mask to find its new home position.
      if (millis() - index_time_ms > 1000 && !finished) {
        index_task.index();
        ready_to_index = false;
        state = INDEXING;
      } else if (finished) {
        state = DONE;
      }
    case DONE:
      // Spin
      break;
  }
}

void actOnIndexEvent(const IndexTask::IndexEvent event,
    const float index_offset_deg) {
  const float pos_deg = mask_controller.getPositionDeg(false);
  Serial.print(trials_complete + 1);
  Serial.print("\t");
  if (event == IndexTask::IndexEvent::INDEX_FOUND) {
    Serial.print("GOOD");
  } else {
    Serial.print("BAD");
  }
  Serial.print("\t");
  Serial.print(index_offset_deg);
  Serial.print("\t");
  Serial.print(pos_deg);
  Serial.println();
  trials_complete++;
  if (trials_complete >= NUM_TRIALS) {
    finished = true;
  } else {
    ready_to_index = true;
  }
}

// Function run via timer interrupt to actuate motor.
void update() {
  motor_controller.update();
}
