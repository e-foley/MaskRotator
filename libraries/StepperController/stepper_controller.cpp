#include "stepper_controller.h"

StepperController::StepperController(BipolarStepper* const stepper,
    const int steps_per_rotation) : stepper_(stepper),
    steps_per_rotation_(steps_per_rotation), position_steps_(0),
    target_deg_(0.0f), target_steps_(0), behavior_(Behavior::STOPPED) {}

void StepperController::forward() volatile {
  behavior_ = Behavior::FORWARD;
}

void StepperController::reverse() volatile {
  behavior_ = Behavior::REVERSE;
}

void StepperController::stop() volatile {
  behavior_ = Behavior::STOPPED;
}

float StepperController::rotateTo(const float target_deg) volatile {
  // Very brief pause to avoid potential momentary direction change.
  behavior_ = Behavior::STOPPED;
  target_deg_ = target_deg;
  target_steps_ = degreesToSteps(target_deg_);
  behavior_ = Behavior::TARGETING;
  return stepsToDegrees(target_steps_);
}

float StepperController::rotateBy(const float angle_deg) volatile {
  // Very brief pause to avoid position changes.
  behavior_ = Behavior::STOPPED;
  target_deg_ = stepsToDegrees(position_steps_) + angle_deg;
  target_steps_ = degreesToSteps(target_deg_);
  behavior_ = Behavior::TARGETING;
  return target_deg_;
}

float StepperController::getPositionDeg() const volatile {
  // TODO: Disable interrupts here.
  return stepsToDegrees(position_steps_);
}

float StepperController::getTarget() const volatile {
  return target_deg_;
}

void StepperController::setZero() volatile {
  position_steps_ = 0;
}

void StepperController::offsetZero(const float relative_angle_deg) volatile {
  position_steps_ -= degreesToSteps(relative_angle_deg);
}

// Note: Instead of a switch tree, we could set a function pointer (to a private
// helper function) whenever we alter behavior_. Snazzy but probably overkill.
void StepperController::update() volatile {
  if (stepper_ == nullptr) {
    return;
  }

  switch (behavior_) {
    default:
    case Behavior::STOPPED:
    case Behavior::REACHED_TARGET:
      break;
    case Behavior::FORWARD:
      stepper_->stepForward();
      position_steps_++;
      break;
    case Behavior::REVERSE:
      stepper_->stepBackward();
      position_steps_--;
      break;
    case Behavior::TARGETING:
      if (position_steps_ < target_steps_) {
        stepper_->stepForward();
        position_steps_++;
      } else if (position_steps_ > target_steps_) {
        stepper_->stepBackward();
        position_steps_--;
      } else /*position_steps_ == target_steps_*/ {
        behavior_ = Behavior::REACHED_TARGET;
      }
      break;
  }
}

int32_t StepperController::degreesToSteps(const float degrees) const volatile {
  return static_cast<int32_t>(round(degrees / 360.0f * steps_per_rotation_));
}

float StepperController::stepsToDegrees(const int32_t steps) const volatile {
  return 360.0f * steps / steps_per_rotation_;
}
