#ifndef INDEX_TASK_H_
#define INDEX_TASK_H_

#include "hall_switch.h"
#include "mask_controller.h"
#include <Arduino.h>  // For size_t

// Operates a cooperative task whose responsibility is to drive a MaskController
// and HallSwitch in conjunction to determine a new index position for the mask.
// Physically, this index position is determined a location of peak magnetic
// field. No other functions should attempt to manipulate the HallSwitch,
// MaskController, or the MaskController's dependencies while indexing is
// active.
//
// The method used to determine an index is to advance the mask forward,
// recording angular positions at which the Hall effect switch triggers from
// low to high and from high to low; then doing the same in reverse; then taking
// the average of all four positions. Finally, the mask homes to its new zero
// point to show the operator where the device believes this location to be.
class IndexTask {
 public:
  // List of possible states the IndexTask can be in.
  enum class State : int {
    START = 0,                // Starting state.
    INIT,                     // State following a request to initialize.
    WAITING_FOR_FORWARD_LOW,  // Forward, waiting to be in a low state.
    FORWARD_LOW,              // Forward, waiting for low-to-high transition.
    FORWARD_HIGH,             // Forward, waiting for high-to-low transition.
    REVERSE_LOW,              // Backward, waiting for low-to-high transition.
    REVERSE_HIGH,             // Backward, waiting for high-to-low transition.
    INDEXED,                  // Index acquired; waiting for next action.
    CANNOT_INDEX              // Index can't be found; waiting for next action.
  };

  // Results of indexing operations.
  enum class IndexEvent {
    NONE,            // Default value.
    INDEX_FOUND,     // Index has been located.
    INDEX_NOT_FOUND  // We failed to find the index.
  };

  // Amount of time we are willing to wait for a HallSwitch state transition
  // before declaring that the device is  unable to find an index [ms].
  static const int INDEX_TIMEOUT_MS = 10000u;

  // Construct a new IndexTask, designating  the MaskController and
  // HallSwitch the task will operate.
  //
  // mask_controller: The MaskController to operate.
  // hall_switch: The HallSwitch to read.
  IndexTask(MaskController* mask_controller, HallSwitch* hall_switch);

  // Initialize the IndexTask. This must be requested before calling index().
  void init();

  // Checks for state transitions and takes actions accordingly. Call this as
  // frequently as possible to improve indexing resolution.
  void step();

  // Seek an index position for the mask. An index position will be established
  // where the task estimates a local peak in magnetic field strength, which
  // will typically be triggered when the magnet is directly above the physical
  // Hall effect sensor. Note that this operation will change the index of the
  // MaskController, affecting all subsequent MaskController actions.
  void index();

  // Retrieves the current state of the IndexTask. See the State enumeration.
  //
  // Returns: The current State enumerator describing the state of the task.
  State getState() const;

  // Establishes a function to call when we have finished looking for an index.
  //
  // cb: The function to invoke when we have finished looking for an index.
  //  -> event: The outcome of the indexing operation.
  //  -> index_offset_deg: The angle the index position has been adjusted by as
  //                       a result of the indexing operation [deg]. Set to
  //                       nullptr to remove the callback.
  void setIndexEventCallback(void (*cb)(IndexEvent event, float index_offset_deg));

 private:
  // Length of array in which we store positions to use in calculating an
  // index position.
  static const size_t NUM_KEY_POSITIONS = 4u;

  // Utility method to check when an index timeout has occurred.
  //
  // Returns: True if a timeout is active.
  bool timedOut() const;

  // Utility method announcing via callback that an index could not be located.
  void announceIndexNotFound() const;

  // The MaskController to manipulate.
  MaskController* const mask_controller_;

  // The HallSwitch to read.
  HallSwitch* const hall_switch_;

  // Flags for requested actions.
  bool init_requested_;
  bool index_requested_;

  // Current state of the IndexTask.
  State state_;

  // Time of last HallSwitch state change or request to index. Used as a
  // reference for index timeouts.
  unsigned long last_index_progress_stamp_ms_;

  // Container for angle datapoints used in the determination of the True
  // index position.
  float key_positions_deg_[NUM_KEY_POSITIONS];

  // Callback to invoke when we have finished looking for an index.
  void (*index_event_callback_)(IndexEvent event, float index_offset_deg);
};

#endif
