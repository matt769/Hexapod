//
// Created by matt on 12/04/2021.
//

#include "joint.h"

#include <cmath>

#include "kinematics_support.h"

namespace hexapod {

Joint::Joint() : Joint(-1.48f, 1.48f, 0.0f, 0.0f, false) {}
/**
 * @brief Construct a new Joint object
 * @details All input should relate to the physical joint used - it will be modified to fit the internal
 *  hexapod reference frames based on the offset and flip_axis parameters.
 *
 * @param lower_limit The joint limit in the clockwise direction of the physical joint. Always less than upper.
 * @param upper_limit  The joint limit in the anti-clockwise direction of the physical joint. Always more than lower.
 * @param angle The starting angle of the joint
 * @param offset The physical angle at which the model joint is at zero degrees
 * @param flip_axis If the physical model uses a joint that has its Z axis reversed
 */
Joint::Joint(const float lower_limit,
             const float upper_limit,
             const float angle,
             const float offset,
             const bool flip_axis)
    : lower_limit_(lower_limit), upper_limit_(upper_limit), offset_(offset), flip_axis_(1.0f) {

  auto sign = [](const float& num) -> float { return (num >= 0.0) ? 1.0 : -1.0; };

  if (flip_axis) {
    // Swap limits, but keep the signs (LL should stay lower than UL)
    flip_axis_ = -1.0;
    lower_limit_ = sign(lower_limit) * fabs(upper_limit);
    upper_limit_ = sign(upper_limit) * fabs(lower_limit);
  }

  if (offset_ != 0.0f) {
    lower_limit_ -= flip_axis_ * offset_;
    upper_limit_ -= flip_axis_ * offset_;
  }

  setFromPhysicalAngle(angle);
}
bool Joint::isWithinLimits(const float angle) const {
  return (angle >= lower_limit_ - hexapod::util::eps) &&
      (angle < upper_limit_ + hexapod::util::eps);
}
float Joint::clampToLimts(const float angle) const {
  return fmax(fmin(angle, upper_limit_), lower_limit_);
}
float Joint::fromPhysicalAngle(const float physical_angle) const {
  return (physical_angle - offset_) * flip_axis_;
}
float Joint::toPhysicalAngle() const {
  return (flip_axis_ * angle_) + offset_;
}
void Joint::setFromPhysicalAngle(const float physical_angle) {
  angle_ = fromPhysicalAngle(physical_angle);
}

} // namespace hexapod
