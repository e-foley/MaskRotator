#ifndef BIPOLAR_STEPPER_H_
#define BIPOLAR_STEPPER_H_

#include <Arduino.h>

//! Represents a bipolar stepper motor.
class BipolarStepper {
 public:
  // Constructs a BipolarStepper by denoting Arduino pins to be used for motor
  // functions. The object will be created in an uninitialized, disabled state.
  //
  // brka: The Arduino pin corresponding to the motor's BRKA line.
  // dira: The Arduino pin corresponding to the motor's DIRA line.
  // (etc.)
  BipolarStepper(int brka, int dira, int pwma, int brkb, int dirb, int pwmb);

  // Destroys a BipolarStepper object, attempting to set the motor into a
  // deenergized state first.
  ~BipolarStepper();

  // Initializes a BipolarStepper object. This must be called in order for
  // actuation commands to function properly.
  void initialize();

  // Checks whether the BipolarStepper object has been initialized.
  //
  // Returns: True if the BipolarStepper object has been initialized.
  bool isInitialized() const;

  // Enables the motor. This must be called in order for actuation commands to
  // succeed.
  void enable();

  // Disables the motor. After disable() is called, actuation commands will be
  // ignored until
  void disable();

  // Checks whether the motor is enabled.
  //
  // Returns: True if the BipolarStepper object is enabled.
  bool isEnabled() const;

  // Steps the motor forward once. Will fail if the motor is not both
  // initialized and enabled.
  void stepForward();

  // Steps the motor backward once. Will fail if the motor is not both
  // initialized and enabled.
  void stepBackward();

 private:
  // The number of unique states that are cycled through via the stepForward()
  // and stepBackward() functions.
  static const int NUM_STATES = 4;

  // Internal function that executes a particular motor state by energizing pins
  // in a pattern appropriate for the current state.
  void doState(int state);

  // Arduino pin assignments for motor functions.
  const int brka_;
  const int dira_;
  const int pwma_;
  const int brkb_;
  const int dirb_;
  const int pwmb_;

  // Which energization state is currently active. Normally between 0 and
  // (NUM_STATES - 1).
  int state_;

  // Other status variables.
  bool initialized_;
  bool enabled_;
};

#endif
