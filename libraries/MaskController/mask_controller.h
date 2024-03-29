#ifndef MASK_CONTROLLER_H_
#define MASK_CONTROLLER_H_

#include "stepper_controller.h"

// Operates a StepperController to manipulate a mask interfacing with a stepper
// motor. Maintains knowledge of the gear ratio between motor and mask in order
// to drive the motor to the desired angles.
class MaskController {
  public:
    // Preferences for direction of motion.
    enum class Direction : int {
      NONE = 0,  // No motion: default value.
      FORWARD,   // Forward direction.
      REVERSE,   // Reverse direction.
      AUTO       // Direction that will reach the target the fastest.
    };

    // Constructs a MaskController that operates a specified StepperController
    // using a given gear ratio between motor and mask.
    //
    // stepper_controller: The StepperController to drive.
    // gear_ratio: Rotations of motor per one rotation of mask.
    MaskController(volatile StepperController* stepper_controller,
        float gear_ratio);

    // Drives the mask forward continuously.
    void forward();

    // Drives the mask backward continuously.
    void reverse();

    // Halts mask motion.
    void stop();

    // Rotates the mask to an absolute angle.
    //
    // target_deg: Absolute angle to rotate the mask to [deg].
    // direction: Preferred direction of motion.
    // wrap_result: Whether the angle returned from the function is wrapped to
    //              the range [0, 360) degrees.
    // Returns: The actual absolute angle rotated to [deg]. May not match the
    //          specified angle exactly due to motor resolution limits.
    float rotateTo(float target_deg, Direction direction = Direction::AUTO,
        bool wrap_result = true);

    // Rotates the mask by a relative angle.
    //
    // angle_deg: Relative angle to rotate the mask by [deg].
    // direction: Preferred direction of motion.
    // wrap_result: Whether the angle returned from the function is wrapped to
    //              the range [0, 360) degrees.
    // Returns: The actual absolute angle rotated to [deg]. May not match the
    //          specified angle exactly due to motor resolution limits.
    float rotateBy(float angle_deg, bool wrap_result = true);

    // Retrieves the current absolute position of the mask.
    //
    // wrap_result: Whether the angle returned from the function is wrapped to
    //              the range [0, 360) degrees.
    // Returns: The current absolute position of the mask [deg].
    float getPositionDeg(bool wrap_result = true) const;

    // Retrieves the current target position of the mask.
    //
    // wrap_result: Whether the angle returned from the function is wrapped to
    //              the range [0, 360) degrees.
    // Returns: The current target position of the mask [deg].
    float getTargetDeg(bool wrap_result = true) const;

    // Establishes the current mask position to be an absolute angle of zero.
    void setZero();

    // Offsets the existing zero reference by an angle.
    //
    // relative_angle_deg: The angle to offset the zero reference by [deg].
    void offsetZero(float relative_angle_deg);

    // Converts a mask angle to a motor angle.
    //
    // mask_angle_deg: An absolute mask angle [deg].
    // Returns: The motor angle corresponding to the mask angle [deg].
    float maskToMotorAngleDeg(float mask_angle_deg) const;

    // Converts a motor angle to a mask angle.
    //
    // motor_angle_deg: An absolute motor angle [deg].
    // Returns: The mask angle corresponding to the motor angle [deg].
    float motorToMaskAngleDeg(float motor_angle_deg) const;

  private:
    // Wraps an unbounded angle to the range [0, 360) degrees.
    //
    // nominal_deg: The unbounded angle [deg].
    // Returns: An equivalent angle on the range [0, 360) degrees.
    static float wrapAngleDeg(float nominal_deg);

    // The StepperController this MaskController manipulates.
    volatile StepperController* const stepper_controller_;

    // Rotations of motor per one rotation of mask.
    const float gear_ratio_;

    // Current absolute target angle [deg].
    float target_deg_;
};

#endif
