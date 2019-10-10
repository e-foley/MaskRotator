#ifndef STEPPER_CONTROLLER_H_
#define STEPPER_CONTROLLER_H_

#include "bipolar_stepper.h"
#include "timer_one.h"

// Drives a motor represented by BipolarStepper object.
class StepperController {
  public:
    // Current motor action.
    enum class Behavior : int {
      STOPPED = 0,    // Motor is stopped. Default value.
      FORWARD,        // Motor is moving forward continuously.
      REVERSE,        // Motor is moving backward continuously.
      TARGETING,      // Motor is currently approaching its target position.
      REACHED_TARGET  // Motor has successfully reached its target position.
    };

    // Constructs a StepperController, delegating a BipolarStepper to manipulate
    // and a number of steps per rotation. The update() function should be
    // invoked within a timer interrupt at approximately 125 Hz.
    //
    // stepper: The BipolarStepper to manipulate.
    // steps_per_rotation: Number of steps that form one full motor rotation.
    StepperController(BipolarStepper* stepper, int16_t steps_per_rotation);

    // Drives the motor forward continuously.
    void forward() volatile;

    // Drives the motor backward continuously.
    void reverse() volatile;

    // Halts motor motion.
    void stop() volatile;

    // Rotates the motor to an absolute angle.
    //
    // target_deg: Absolute angle to rotate the motor to [deg].
    // Returns: The actual absolute angle rotated to [deg]. May not match the
    //          specified angle exactly due to the finite number of steps per
    //          rotation.
    float rotateTo(float target_deg) volatile;

    // Rotates the motor by a relative angle.
    //
    // angle_deg: Relative angle to rotate the motor by [deg].
    // Returns: The actual absolute angle rotated to [deg]. May not match the
    //          specified angle exactly due to the finite number of steps per
    //          rotation.
    float rotateBy(float angle_deg) volatile;

    // Retrieves the current absolute position of the motor.
    //
    // Returns: The current absolute position of the motor [deg].
    float getPositionDeg() const volatile;

    // Retrieves the current target position of the motor.
    //
    // Returns: The current target position of the motor [deg].
    float getTarget() const volatile;

    // Establishes the current motor position to be an absolute angle of zero.
    void setZero() volatile;

    // Offsets the existing zero reference by an angle.
    //
    // relative_angle_deg: The angle to offset the zero reference by [deg].
    void offsetZero(float relative_angle_deg) volatile;

    // Updates the state of the motor. For best results, this should be called
    // within a timer interrupt triggering at approximately 125 Hz.
    void update() volatile;

    // Converts an absolute motor position to an absolute number of motor steps.
    //
    // degrees: The absolute angle to convert [deg].
    // Returns: The integral number of steps forming an angle closest to the
    //          given angle.
    int32_t degreesToSteps(float degrees) const volatile;

    // Converts a number of motor steps to an absolute angular position.
    //
    // steps: The number of steps.
    // Returns: The angle formed by traveling the given number of steps [deg].
    float stepsToDegrees(int32_t steps) const volatile;

  private:
    // The BipolarStepper driver this StepperController manipulates.
    BipolarStepper* const stepper_;

    // The number of steps of the motor constituting one full revolution.
    const int16_t steps_per_rotation_;

    // Current position of the motor in steps relative to zero.
    volatile int32_t position_steps_;

    // Current target absolute angle of the motor [deg].
    float target_deg_;

    // Current target absolute position of the motor in steps.
    int32_t target_steps_;

    // Currently active behavior.
    volatile Behavior behavior_;
};

#endif
