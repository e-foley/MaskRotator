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
enum class Mode {
  NONE,
  ABSOLUTE,
  RELATIVE
} mode = Mode::ABSOLUTE;
float target = 0.0f;
bool dir = true;

// Called once at the start of the progrom; initializes all hardware and tasks.
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);
  stepper.initialize();
  stepper.enable();
  hall_switch.init();
  index_task.init();
  timer.initialize();
  timer.attachInterrupt(update, STEP_PERIOD_US);
}

// Called repeatedly: updates tasks and looks for new actions to take based on
// command inputs.
void loop() {
  index_task.step();

  if (Serial.available()) {
    const char command = Serial.peek();
    switch (command) {
      case 'f':
        // Go forward.
        Serial.read();
        Serial.println("Moving mask forward.");
        mask_controller.forward();
        break;
      case 'b':
        // Go backward.
        Serial.read();
        Serial.println("Moving mask backward.");
        mask_controller.reverse();
        break;
      case 's':
        // Stop.
        Serial.read();
        Serial.println("Mask stopped.");
        mask_controller.stop();
        break;
      case 'p':
        // Retrieve mask position.
        Serial.read();
        Serial.print("Current mask position: ");
        Serial.print(mask_controller.getPositionDeg(true));
        Serial.print(" deg (");
        Serial.print(mask_controller.getPositionDeg(false));
        Serial.println(" deg)");
        break;
      case 't':
        // Retrieve mask target position.
        Serial.read();
        Serial.print("Mask target position: ");
        Serial.print(mask_controller.getTargetDeg(true));
        Serial.print(" deg (");
        Serial.print(mask_controller.getTargetDeg(false));
        Serial.println(" deg)");
        break;
      case 'z':
        // Zero current mask position.
        Serial.read();
        mask_controller.setZero();
        Serial.println("Mask position zeroed.");
        break;
      case 'r':
        Serial.read();
        mode = Mode::RELATIVE;
        Serial.println("Mode set to relative.");
        break;
      case 'a':
        Serial.read();
        mode = Mode::ABSOLUTE;
        Serial.println("Mode set to absolute.");
        break;
      case 'i':
        Serial.read();
        index_task.index();
        break;
      default: {
        float actual = 0.0f;
        if (mode == Mode::ABSOLUTE) {
          actual = mask_controller.rotateTo(Serial.parseFloat(),
              PREFERRED_DIRECTION);
          Serial.print("Target set to ");
          Serial.print(actual);
          Serial.println(" degrees.");
        } else if (mode == Mode::RELATIVE) {
          const float relative_angle = Serial.parseFloat();
          actual = mask_controller.rotateBy(relative_angle);
          Serial.print("Rotating mask by ");
          Serial.print(relative_angle);
          Serial.print(" degrees to new target of ");
          Serial.print(actual);
          Serial.println(" degrees.");
        }
        break;
      }
    }
  }
}

void update() {
  motor_controller.update();
}
