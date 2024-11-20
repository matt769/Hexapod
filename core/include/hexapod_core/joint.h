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
    @details It may be convenient to provide an offset value if the physical or modelled joint isn't
    actually at zero when the frames for the joints are in the 'default' place
    The joint limits should be provided as normal i.e. do not modify to take into account the offset,
     this will be done automatically.
    Internally, the leg will only use the angle and limits. The flip and offset can be applied to translate this angle
     back to a physical joint angle
*/
class Joint {
    friend class JointBuilder;
 public:
  float lower_limit_ = 0;
  float upper_limit_ = 0;
  float angle_ = 0;
  float offset_ = 0;
  float flip_axis_ = 1.0;
  Joint() = default;
  Joint(float offset, bool flip_axis);
  Joint(float physical_lower_limit, float physical_upper_limit, float physical_angle = 0.0f, float offset = 0.0f, bool flip_axis = false);
  bool isWithinLimits(float angle) const;
  float clampToLimts(float angle) const;
  void set(float model_angle);
  float angle() const;
  float physicalAngle() const;
  float fromPhysicalAngle(float physical_angle) const;
  float toPhysicalAngle(float model_angle) const;
  float toPhysicalAngle() const;
  void setFromPhysicalAngle(float physical_angle);

  static Joint createFromPhysicalAngles(float physical_lower_limit, float physical_upper_limit, float physical_angle = 0.0f, float offset = 0.0f, bool flip_axis = false);
  static Joint createFromModelAngles(float model_lower_limit, float model_upper_limit, float model_angle = 0.0f, float offset = 0.0f, bool flip_axis = false);



};

class JointBuilder {
private:
    Joint joint;
public:
    explicit JointBuilder(float offset = 0.0f, bool flip_axis = false);
    JointBuilder addPhysicalLimits(float physical_lower_limit, float physical_upper_limit);
    JointBuilder addModelLimits(float model_lower_limit, float model_upper_limit);
    JointBuilder setPhysicalAngle(float physical_angle);
    JointBuilder setModelAngle(float model_angle);
    Joint create() const;
};


} // namespace hexapod

#endif // HEXAPOD_JOINT_H_
