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
 public:
  float lower_limit_;
  float upper_limit_;
  float angle_;
  float offset_;
  float flip_axis_;
  Joint();
  Joint(float lower_limit, float upper_limit, float angle = 0.0f, float offset = 0.0f, bool flip_axis = false);
  bool isWithinLimits(float angle) const;
  float clampToLimts(float angle) const;
  float fromPhysicalAngle(float physical_angle) const;
  float toPhysicalAngle(float model_angle) const;
  float toPhysicalAngle() const;
  void setFromPhysicalAngle(float physical_angle);
};

} // namespace hexapod

#endif // HEXAPOD_JOINT_H_
