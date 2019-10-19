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
enum Command : char {
  FORWARD_COMMAND = 'f',
  BACKWARD_COMMAND = 'b',
  STOP_COMMAND = 's',
  GET_POSITION_COMMAND = 'p',
  GET_TARGET_COMMAND = 't',
  SET_ZERO_COMMAND = 'z',
  ENTER_RELATIVE_MODE_COMMAND = 'r',
  ENTER_ABSOLUTE_MODE_COMMAND = 'a',
  LOCATE_INDEX_COMMAND = 'i',
  FOUND_INDEX_RESPONSE = 'I',
  COULD_NOT_FIND_INDEX_RESPONSE = '~',
  PING_COMMAND = '?',
  PING_RESPONSE = '!',
  GO_TO_COMMAND = 'g',
  UNRECOGNIZED_COMMAND = 'x'
};

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
enum class Mode {
  NONE,
  ABSOLUTE,
  RELATIVE
} mode = Mode::ABSOLUTE;

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

// Called repeatedly: updates tasks and looks for new actions to take based on
// command inputs.
void loop() {
  index_task.step();

  // Process input.
  if (Serial.available()) {
    const char command = Serial.peek();
    switch (command) {
      case FORWARD_COMMAND:
        Serial.read();
        mask_controller.forward();
        Serial.write(FORWARD_COMMAND);
        Serial.println();
        break;
      case BACKWARD_COMMAND:
        Serial.read();
        mask_controller.reverse();
        Serial.write(BACKWARD_COMMAND);
        Serial.println();
        break;
      case STOP_COMMAND:
        Serial.read();
        mask_controller.stop();
        Serial.write(STOP_COMMAND);
        Serial.println();
        break;
      case GET_POSITION_COMMAND:
        Serial.read();
        Serial.write(GET_POSITION_COMMAND);
        Serial.println(degreesToSerial(mask_controller.getPositionDeg(true)));
        break;
      case GET_TARGET_COMMAND:
        Serial.read();
        Serial.write(GET_TARGET_COMMAND);
        Serial.println(degreesToSerial(mask_controller.getTargetDeg(true)));
        break;
      case SET_ZERO_COMMAND:
        Serial.read();
        mask_controller.setZero();
        Serial.write(SET_ZERO_COMMAND);
        Serial.println();
        break;
      case ENTER_RELATIVE_MODE_COMMAND:
        Serial.read();
        mode = Mode::RELATIVE;
        Serial.write(ENTER_RELATIVE_MODE_COMMAND);
        Serial.println();
        break;
      case ENTER_ABSOLUTE_MODE_COMMAND:
        Serial.read();
        mode = Mode::ABSOLUTE;
        Serial.write(ENTER_ABSOLUTE_MODE_COMMAND);
        Serial.println();
        break;
      case LOCATE_INDEX_COMMAND:
        Serial.read();
        index_task.index();
        Serial.write(LOCATE_INDEX_COMMAND);
        Serial.println();
        break;
      case PING_COMMAND:
        Serial.read();
        Serial.write(PING_RESPONSE);
        Serial.println();
        break;
      case GO_TO_COMMAND: {
        Serial.read();  // Get the command character out of the buffer.
        float serial_deg = serialToDegrees(Serial.parseInt());
        float actual_deg = 0.0f;
        if (mode == Mode::ABSOLUTE) {
          actual_deg = mask_controller.rotateTo(serial_deg, PREFERRED_DIRECTION);
        } else if (mode == Mode::RELATIVE) {
          actual_deg = mask_controller.rotateBy(serial_deg);
        }
        Serial.write(GO_TO_COMMAND);
        Serial.println(degreesToSerial(actual_deg));
        break;
      }
      default:
        Serial.read();  // Discard character if we don't recognize it.
        Serial.write(UNRECOGNIZED_COMMAND);
        Serial.println();
        break;
    }
  }
}

// Converts an angle from serial convention to degrees.
float serialToDegrees(const int32_t serial) {
  return serial / 100.0f;
}

// Convets an angle from degrees to serial convention. There is no overflow
// protection.
int32_t degreesToSerial(const float degrees) {
  return static_cast<int32_t>(round(degrees * 100.0f));
}

void actOnIndexEvent(const IndexTask::IndexEvent event,
    const float index_offset_deg) {
  (void)(index_offset_deg);  // Denote index offset parameter as unused.
  if (event == IndexTask::IndexEvent::INDEX_FOUND) {
    Serial.write(FOUND_INDEX_RESPONSE);
    Serial.println();
  } else if (event == IndexTask::IndexEvent::INDEX_NOT_FOUND) {
    Serial.write(COULD_NOT_FIND_INDEX_RESPONSE);
    Serial.println();
  }
}

// Function run via timer interrupt to actuate motor.
void update() {
  motor_controller.update();
}
