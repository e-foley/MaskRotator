#include "index_task.h"
#include "hall_switch.h"
#include "stepper_controller.h"
#include <Arduino.h>


IndexTask::IndexTask(StepperController* const stepper_controller, HallSwitch* const hall_switch) :
    stepper_controller_(stepper_controller), hall_switch_(hall_switch), init_requested_(false),
    index_requested_(false), state_(State::START), last_index_progress_stamp_ms_(0u) {
  for (size_t i = 0u; i < NUM_KEY_POSITIONS; ++i) {
    key_positions_deg_[i] = 0.0f;
  }
}

void IndexTask::init() {
  init_requested_ = true;
}

void IndexTask::step() {
  switch (state_) {
    case State::START:
      // Just wait for an init command...
      if (init_requested_) {
        init_requested_ = false;
        stepper_controller_->stop();
        hall_switch_->setPowerState(false);
        state_ = State::INIT;
      }
      break;
    case State::INIT:
      // State reserved for more functionality... In the meantime, just wait for an index command.
      if (index_requested_) {
        index_requested_ = false;
        stepper_controller_->forward();
        hall_switch_->setPowerState(true);
        last_index_progress_stamp_ms_ = millis();
        state_ = State::WAITING_FOR_FORWARD_LOW;
      }
      break;
    case State::WAITING_FOR_FORWARD_LOW:
      // Wait for a low signal. (This is important if an index is requested when we are currently
      // near the index position.)
      if (!hall_switch_->isTriggered()) {
        last_index_progress_stamp_ms_ = millis();
        state_ = State::FORWARD_LOW;
      } else if (timedOut()) {
        stepper_controller_->stop();
        hall_switch_->setPowerState(false);
        state_ = State::CANNOT_INDEX;
      }
      break;
    case State::FORWARD_LOW:
      // Continue forward as we wait for a triggered sensor.
      if (hall_switch_->isTriggered()) {
        key_positions_deg_[0] = stepper_controller_->getPosition();
        last_index_progress_stamp_ms_ = millis();
        state_ = State::FORWARD_HIGH;
      } else if (timedOut()) {
        stepper_controller_->stop();
        hall_switch_->setPowerState(false);
        state_ = State::CANNOT_INDEX;
      }
      break;
    case State::FORWARD_HIGH:
      // We currently have a triggered sensor... Continue until it's not triggered anymore.
      if (!hall_switch_->isTriggered()) {
        key_positions_deg_[1] = stepper_controller_->getPosition();
        stepper_controller_->reverse();
        last_index_progress_stamp_ms_ = millis();
        state_ = State::REVERSE_LOW;
      } else if (timedOut()) {
        stepper_controller_->stop();
        hall_switch_->setPowerState(false);
        state_ = State::CANNOT_INDEX;
      }
      break;
    case State::REVERSE_LOW:
      // Retread our ground in reverse until sensor is high again...
      if (hall_switch_->isTriggered()) {
        key_positions_deg_[2] = stepper_controller_->getPosition();
        last_index_progress_stamp_ms_ = millis();
        state_ = State::REVERSE_HIGH;
      } else if (timedOut()) {
        stepper_controller_->stop();
        hall_switch_->setPowerState(false);
        state_ = State::CANNOT_INDEX;
      }
      break;
    case State::REVERSE_HIGH:
      // Last step in reverse...
      if (!hall_switch_->isTriggered()) {
        key_positions_deg_[3] = stepper_controller_->getPosition();
        stepper_controller_->stop();
        float angle_sum_deg_ = 0.0f;
        for (size_t i = 0u; i < NUM_KEY_POSITIONS; ++i) {
          angle_sum_deg_ += key_positions_deg_[i];
        }
        stepper_controller_->setZero(angle_sum_deg_ / NUM_KEY_POSITIONS);
        hall_switch_->setPowerState(false);
        last_index_progress_stamp_ms_ = millis();
        state_ = State::INDEXED;
      } else if (timedOut()) {
        stepper_controller_->stop();
        hall_switch_->setPowerState(false);
        state_ = State::CANNOT_INDEX;
      }
      break;
    case State::INDEXED:
      // We did it! Now wait for the next command to index so we can restart the process.
      if (index_requested_) {
        index_requested_ = false;
        stepper_controller_->forward();
        hall_switch_->setPowerState(true);
        last_index_progress_stamp_ms_ = millis();
        state_ = State::WAITING_FOR_FORWARD_LOW;
      }
      break;
    case State::CANNOT_INDEX:
      // Not a lot we can do in an error state except wait for our next opportunity.
      if (index_requested_) {
        index_requested_ = false;
        stepper_controller_->forward();
        hall_switch_->setPowerState(true);
        last_index_progress_stamp_ms_ = millis();
        state_ = State::WAITING_FOR_FORWARD_LOW;
      }
      break;
    default:
      // No clue how we got here... Let's reset everything.
      stepper_controller_->stop();
      hall_switch_->setPowerState(false);
      init_requested_ = false;
      index_requested_ = false;
      state_ = State::START;
      break;
  }
}

void IndexTask::index() {
  index_requested_ = true;
}

IndexTask::State IndexTask::getState() const {
  return state_;
}

bool IndexTask::timedOut() const {
  return (int)(millis() - last_index_progress_stamp_ms_) > INDEX_TIMEOUT_MS;
}
