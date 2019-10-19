#include "index_task.h"
#include "hall_switch.h"
#include "mask_controller.h"
#include <Arduino.h>

IndexTask::IndexTask(MaskController* const mask_controller,
    HallSwitch* const hall_switch) : mask_controller_(mask_controller),
    hall_switch_(hall_switch), init_requested_(false), index_requested_(false),
    state_(State::START), last_index_progress_stamp_ms_(0u),
    index_event_callback_(nullptr) {
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
        mask_controller_->stop();
        hall_switch_->setPowerState(false);
        state_ = State::INIT;
      }
      break;
    case State::INIT:
      // State reserved for more functionality... In the meantime, just wait for
      // an index command.
      if (index_requested_) {
        index_requested_ = false;
        mask_controller_->forward();
        hall_switch_->setPowerState(true);
        last_index_progress_stamp_ms_ = millis();
        state_ = State::WAITING_FOR_FORWARD_LOW;
      }
      break;
    case State::WAITING_FOR_FORWARD_LOW:
      // Wait for a low signal. (This is important if an index is requested when
      // we are currently near the index position.)
      if (!hall_switch_->isTriggered()) {
        last_index_progress_stamp_ms_ = millis();
        state_ = State::FORWARD_LOW;
      } else if (timedOut()) {
        mask_controller_->stop();
        hall_switch_->setPowerState(false);
        announceIndexNotFound();
        state_ = State::CANNOT_INDEX;
      }
      break;
    case State::FORWARD_LOW:
      // Continue forward as we wait for a triggered sensor.
      if (hall_switch_->isTriggered()) {
        key_positions_deg_[0] = mask_controller_->getPositionDeg(false);
        last_index_progress_stamp_ms_ = millis();
        state_ = State::FORWARD_HIGH;
      } else if (timedOut()) {
        mask_controller_->stop();
        hall_switch_->setPowerState(false);
        announceIndexNotFound();
        state_ = State::CANNOT_INDEX;
      }
      break;
    case State::FORWARD_HIGH:
      // We currently have a triggered sensor... Continue until it's not
      // triggered anymore.
      if (!hall_switch_->isTriggered()) {
        key_positions_deg_[1] = mask_controller_->getPositionDeg(false);
        mask_controller_->reverse();
        last_index_progress_stamp_ms_ = millis();
        state_ = State::REVERSE_LOW;
      } else if (timedOut()) {
        mask_controller_->stop();
        hall_switch_->setPowerState(false);
        announceIndexNotFound();
        state_ = State::CANNOT_INDEX;
      }
      break;
    case State::REVERSE_LOW:
      // Retread our ground in reverse until sensor is high again...
      if (hall_switch_->isTriggered()) {
        key_positions_deg_[2] = mask_controller_->getPositionDeg(false);
        last_index_progress_stamp_ms_ = millis();
        state_ = State::REVERSE_HIGH;
      } else if (timedOut()) {
        mask_controller_->stop();
        hall_switch_->setPowerState(false);
        announceIndexNotFound();
        state_ = State::CANNOT_INDEX;
      }
      break;
    case State::REVERSE_HIGH:
      // Last step in reverse...
      if (!hall_switch_->isTriggered()) {
        key_positions_deg_[3] = mask_controller_->getPositionDeg(false);
        mask_controller_->stop();
        hall_switch_->setPowerState(false);

        // Calculate average transition position.
        float angle_sum_deg = 0.0f;
        for (size_t i = 0u; i < NUM_KEY_POSITIONS; ++i) {
          angle_sum_deg += key_positions_deg_[i];
        }
        const float offset_deg = angle_sum_deg / NUM_KEY_POSITIONS;

        // Apply new index position and communicate it via callback.
        mask_controller_->offsetZero(offset_deg);
        if (index_event_callback_ != nullptr) {
          index_event_callback_(IndexEvent::INDEX_FOUND, offset_deg);
        }

        // Rotate to new zero to show users where we think it is.
        mask_controller_->rotateTo(0.0f);
        last_index_progress_stamp_ms_ = millis();
        state_ = State::INDEXED;
      } else if (timedOut()) {
        mask_controller_->stop();
        hall_switch_->setPowerState(false);
        announceIndexNotFound();
        state_ = State::CANNOT_INDEX;
      }
      break;
    case State::INDEXED:
      // We did it! Now wait for the next command to index so we can restart the
      // process.
      if (index_requested_) {
        index_requested_ = false;
        mask_controller_->forward();
        hall_switch_->setPowerState(true);
        last_index_progress_stamp_ms_ = millis();
        state_ = State::WAITING_FOR_FORWARD_LOW;
      }
      break;
    case State::CANNOT_INDEX:
      // Not a lot we can do in an error state except wait for instructions.
      if (index_requested_) {
        index_requested_ = false;
        mask_controller_->forward();
        hall_switch_->setPowerState(true);
        last_index_progress_stamp_ms_ = millis();
        state_ = State::WAITING_FOR_FORWARD_LOW;
      }
      break;
    default:
      // No clue how we got here... Let's reset everything.
      mask_controller_->stop();
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

void IndexTask::setIndexEventCallback(
    void (*const cb)(IndexEvent event, float index_offset_deg)) {
  index_event_callback_ = cb;
}

bool IndexTask::timedOut() const {
  return (int)(millis() - last_index_progress_stamp_ms_) > INDEX_TIMEOUT_MS;
}

void IndexTask::announceIndexNotFound() const {
  if (index_event_callback_ != nullptr) {
    index_event_callback_(IndexEvent::INDEX_NOT_FOUND, 0.0f);
  }
}
