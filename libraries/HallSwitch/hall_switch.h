#ifndef HALL_SWITCH_H_
#define HALL_SWITCH_H_

// Represents a binary Hall effect switch that detects the presence of a nearby
// magnetic field.
class HallSwitch {
 public:
  // Constructs a HallSwitch object, delegating Arduino pins for its functions.
  // The HallSwitch object is constructed in an uninitialized state.
  //
  // power_pin: The Arduino pin used to power the hall effect switch.
  // state_pin: The Arduino pin delegated to read the digital state of the Hall
  //            effect switch.
  HallSwitch(int power_pin, int state_pin);

  // Initializes the HallEffect object. This must be called before setting the
  // power state of the switch or reading the switch's state.
  void init();

  // Checks whether the Hall effect switch has been initialized.
  //
  // Returns: True if the switch has been initialized.
  bool isInitialized() const;

  // Powers on or off the Hall effect switch.
  //
  // power_state: True to energize the hall effect switch. (Unenergized switches
  //              cannot trigger.)
  void setPowerState(bool power_state);

  // Checks whether the Hall switch is currently triggered by a magnetic field.
  //
  // Returns: True if the switch is triggered by a magnetic field.
  bool isTriggered() const;

 private:
   // Arduino pins delegated for Hall effects switch functions.
   const int power_pin_;
   const int state_pin_;

   // Whether the swiltch has been initialized.
   bool is_initialized_;
};

#endif
