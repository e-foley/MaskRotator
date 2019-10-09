#ifndef INDEX_TASK_H_
#define INDEX_TASK_H_

#include "hall_switch.h"
#include "mask_controller.h"
#include <Arduino.h>

class IndexTask {
 public:
  enum class State : int {
    START = 0,
    INIT,
    WAITING_FOR_FORWARD_LOW,
    FORWARD_LOW,
    FORWARD_HIGH,
    REVERSE_LOW,
    REVERSE_HIGH,
    INDEXED,
    CANNOT_INDEX
  };

  static const int INDEX_TIMEOUT_MS = 10000u;  // [ms]

  IndexTask(MaskController* mask_controller, HallSwitch* hall_switch);
  void init();
  void step();
  void index();
  State getState() const;

 private:
  static const size_t NUM_KEY_POSITIONS = 4u;

  bool timedOut() const;

  MaskController* const mask_controller_;
  HallSwitch* const hall_switch_;
  bool init_requested_;
  bool index_requested_;
  State state_;
  uint32_t last_index_progress_stamp_ms_;
  float key_positions_deg_[NUM_KEY_POSITIONS];
};

#endif
