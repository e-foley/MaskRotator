#include "mask_controller.h"

MaskController::MaskController(
    volatile StepperController* const stepper_controller,
    const float gear_ratio) : stepper_controller_(stepper_controller),
    gear_ratio_(gear_ratio), target_deg_(0.0f) {}

void MaskController::forward() {
  if (stepper_controller_ == nullptr) {
    return;
  } else if (gear_ratio_ > 0.0f) {
    stepper_controller_->forward();
  } else {
    stepper_controller_->reverse();
  }
}

void MaskController::reverse() {
  if (stepper_controller_ == nullptr) {
    return;
  } else if (gear_ratio_ > 0.0f) {
    stepper_controller_->reverse();
  } else {
    stepper_controller_->forward();
  }
}

void MaskController::stop() {
  if (stepper_controller_ == nullptr) {
    return;
  } else {
    stepper_controller_->stop();
  }
}

float MaskController::rotateTo(const float target_deg_deg, const Direction direction,
    const bool wrap_result) {
  if (stepper_controller_ == nullptr) {
    return NAN;
  }

  // Stop and record because the motor angle would theoretically change as we
  // progress through the function if we didn't do this.
  stepper_controller_->stop();
  const float current_deg = getPositionDeg(false);
  const float forward_delta_deg = wrapAngle(target_deg - current_deg);
  const float reverse_delta_deg = wrapAngle(current_deg - target_deg);

  float delta_to_use_deg = 0.0f;
  switch (direction) {
    default:
    case Direction::NONE:
      break;
    case Direction::FORWARD:
      delta_to_use_deg = forward_delta_deg;
      break;
    case Direction::REVERSE:
      delta_to_use_deg = -reverse_delta_deg;
      break;
    case Direction::AUTO:
      delta_to_use_deg = forward_delta_deg < reverse_delta_deg ?
          forward_delta_deg : -reverse_delta_deg;
      break;
  }

  return rotateBy(delta_to_use_deg, wrap_result);
}

float MaskController::rotateBy(const float angle_deg, const bool wrap_result) {
  target_deg_ = getPositionDeg(false) + angle_deg;
  // We use rotateTo() below rather than rotateBy() so that we don't accumulate
  // roundoff error  between target_deg_ and the converted motor angle target in
  // repeated calls to this function.
  const float nominal_deg = motorToMaskAngle(
      stepper_controller_->rotateTo(maskToMotorAngle(target_deg_)));
  return wrap_result ? wrapAngleDeg(nominal_deg) : nominal_deg;
}

float MaskController::getPositionDeg(const bool wrap_result) const {
  const float nominal_deg = motorToMaskAngle(stepper_controller_->getPositionDeg());
  return wrap_result ? wrapAngleDeg(nominal_deg) : nominal_deg;
}

float MaskController::getTargetDeg(const bool wrap_result) const {
  return wrap_result ? wrapAngleDeg(target_deg_) : target_deg_;
}

void MaskController::setZero() {
  if (stepper_controller_ == nullptr) {
    return;
  }
  stepper_controller_->stop();
  stepper_controller_->setZero();
}

void MaskController::offsetZero(const float relative_angle_deg) {
  if (stepper_controller_ == nullptr) {
    return;
  }
  stepper_controller_->stop();
  stepper_controller_->offsetZero(maskToMotorAngle(relative_angle_deg));
}

float MaskController::wrapAngle(const float nominal) {
  return nominal - 360.0f * floor(nominal / 360.0f);
}

float MaskController::maskToMotorAngle(const float mask_angle_deg) const {
  return mask_angle_deg * gear_ratio_;
}

float MaskController::motorToMaskAngle(const float motor_angle) const {
  return motor_angle / gear_ratio_;
}
