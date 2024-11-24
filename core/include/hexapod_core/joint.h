//
// Created by matt on 12/04/2021.
//

#ifndef HEXAPOD_JOINT_H_
#define HEXAPOD_JOINT_H_

#ifdef __AVR__
#include <Arduino.h>
#endif

namespace hexapod {

/** @class Joint
    @brief Wraps current angle and limits, plus some basic utility functions.
    @details This join is intended to represent a joint in the hexapod model. But if there is a
   physical robot, or some other model that needs to map to the hexapod model, then it may be
   convenient to provide an offset value and axis 'flip' indicator to describe the relationship
   between then so that angles can be converted between the alternate joint spaces. In relation to
   the Joint object, the alternate space will be referred to as 'physical' (though it does not have
   to be). No units enforced but other parts of the code expect radians. It is a common need to test
   against angles that aren't the current joint value, which is why most functions have a version
   which operates on the object itself, and a version which operates on a provided value and returns
   a result
    @see JointBuilder for options creating the Joint object
*/
class Joint {
  friend class JointBuilder;

 public:
  /// @brief Clockwise joint limit
  float lower_limit_ = 0;
  /// @brief Anti-clockwise joint limit
  float upper_limit_ = 0;
  /// @brief Current angle
  float angle_ = 0;
  /// @brief The angle of the alternate (physical?) joint at which the model joint is zero degrees
  float offset_ = 0;
  /// @brief Indicates whether the alternate joint has its rotation axis flipped (-1.0) or not (1.0)
  float flip_axis_ = 1.0;
  /// @brief default constructor to allow easy Joint array creation
  Joint() = default;
  /// @brief basic constructor that will allow using the physical-model conversion functions
  Joint(float offset, bool flip_axis);
  /// @brief 'full' constructor - takes angles from the physical frame - will like change in the
  /// near future
  Joint(float physical_lower_limit, float physical_upper_limit, float physical_angle = 0.0f, float offset = 0.0f,
        bool flip_axis = false);
  /// @brief is the passed angle within the model joint limits (with some float tolerance)
  bool isWithinLimits(float angle) const;
  /// @brief clamp the passed model angle down so that it is within the model joint limits
  float clampToLimits(float angle) const;
  /// @brief Set the angle using the model angle
  void set(float model_angle);
  /// @brief Get the model angle
  float angle() const;
  /// @brief Get the physical angle
  float physicalAngle() const;
  /// @brief Get the model angle value for this joint given a physical value
  float fromPhysicalAngle(float physical_angle) const;
  /// @brief Get the physical angle value for this joint given a model value
  float toPhysicalAngle(float model_angle) const;
  /// @brief Get the physical angle - WILL BE REMOVED
  float toPhysicalAngle() const;
  /// @brief Set the angle using the physical angle
  void setFromPhysicalAngle(float physical_angle);
  /// @brief A more explicit way to create a joint from physical angles
  static Joint createFromPhysicalAngles(float physical_lower_limit, float physical_upper_limit,
                                        float physical_angle = 0.0f, float offset = 0.0f, bool flip_axis = false);
  /// @brief A more explicit way to create a joint from model angles
  static Joint createFromModelAngles(float model_lower_limit, float model_upper_limit, float model_angle = 0.0f,
                                     float offset = 0.0f, bool flip_axis = false);
};

/** @class JointBuilder
    @brief Utility class to make creating the joints more explicit (I found it easy to get confused
   between the 2 joint spaces)
 */
class JointBuilder {
 private:
  Joint joint;

 public:
  /// @brief Start join creation by making a JointBuilder with an offset and flip indicator
  explicit JointBuilder(float offset = 0.0f, bool flip_axis = false);
  /// @brief Specify joint limits in the physical space
  JointBuilder addPhysicalLimits(float physical_lower_limit, float physical_upper_limit);
  /// @brief Specify joint limits in the model space
  JointBuilder addModelLimits(float model_lower_limit, float model_upper_limit);
  /// @brief Specify joint angle in the physical space
  JointBuilder setPhysicalAngle(float physical_angle);
  /// @brief Specify joint angle in the model space
  JointBuilder setModelAngle(float model_angle);
  /// @brief Output the built joint object
  Joint create() const;
};

}  // namespace hexapod

#endif  // HEXAPOD_JOINT_H_
