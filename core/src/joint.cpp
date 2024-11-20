//
// Created by matt on 12/04/2021.
//

#include "hexapod_core/joint.h"

#ifdef __AVR__
#include <Arduino.h>
#else
#include <cmath>
#endif

#include "hexapod_core/kinematics_support.h"

namespace hexapod {

/**
 * @brief Construct a new Joint object
 * @details All input should relate to the physical joint used - it will be modified to fit the internal
 *  hexapod reference frames based on the offset and flip_axis parameters as required.
 *
 * @param physical_lower_limit The joint limit in the clockwise direction of the physical joint. Always less than upper.
 * @param physical_upper_limit  The joint limit in the anti-clockwise direction of the physical joint. Always more than lower.
 * @param physical_angle The starting angle of the joint
 * @param offset The physical angle at which the model joint is at zero degrees
 * @param flip_axis If the physical model uses a joint that has its Z axis reversed
 */
Joint::Joint(const float physical_lower_limit,
             const float physical_upper_limit,
             const float physical_angle,
             const float offset,
             const bool flip_axis) {

  // Once we set offset_ and flip_axis_ we can use the conversion functions
  offset_ = offset;
  flip_axis_ = !flip_axis ? 1.0 : -1.0;
  lower_limit_ = fromPhysicalAngle(!flip_axis ? physical_lower_limit : physical_upper_limit);
  upper_limit_ = fromPhysicalAngle(!flip_axis ? physical_upper_limit : physical_lower_limit);
  angle_ = fromPhysicalAngle(physical_angle);
}


Joint Joint::createFromPhysicalAngles(const float physical_lower_limit,
                 const float physical_upper_limit,
                 const float physical_angle,
                 const float offset,
                 const bool flip_axis) {
    Joint j;
    j.offset_ = offset;
    j.flip_axis_ = !flip_axis ? 1.0 : -1.0;
    // Once we set offset_ and flip_axis_ we can use the conversion functions
    j.lower_limit_ = j.fromPhysicalAngle(!flip_axis ? physical_lower_limit : physical_upper_limit);
    j.upper_limit_ = j.fromPhysicalAngle(!flip_axis ? physical_upper_limit : physical_lower_limit);
    j.angle_ = j.fromPhysicalAngle(physical_angle);
    return j;
    }

Joint Joint::createFromModelAngles(const float model_lower_limit, const float model_upper_limit, const float model_angle, const float offset, const bool flip_axis) {
    Joint j;
    j.offset_ = offset;
    j.flip_axis_ = !flip_axis ? 1.0 : -1.0;
    j.lower_limit_ = model_lower_limit;
    j.upper_limit_ = model_upper_limit;
    j.angle_ = model_angle;
    return j;
}

bool Joint::isWithinLimits(const float angle) const {
  return (angle >= lower_limit_ - hexapod::util::eps) &&
      (angle < upper_limit_ + hexapod::util::eps);
}
float Joint::clampToLimts(const float angle) const {
  return fmax(fmin(angle, upper_limit_), lower_limit_);
}

void Joint::set(const float model_angle) {
    // TODO auto-clamp?
    angle_ = model_angle;
}

float Joint::angle() const {
    return angle_;
}

float Joint::fromPhysicalAngle(const float physical_angle) const {
  return (physical_angle - offset_) * flip_axis_;
}

float Joint::toPhysicalAngle(float model_angle) const {
  return (flip_axis_ * model_angle) + offset_;
}

float Joint::physicalAngle() const {
    return (flip_axis_ * angle_) + offset_;
}

float Joint::toPhysicalAngle() const {
  return (flip_axis_ * angle_) + offset_;
}

void Joint::setFromPhysicalAngle(const float physical_angle) {
  angle_ = fromPhysicalAngle(physical_angle);
}

Joint::Joint(float offset, bool flip_axis) : offset_(offset), flip_axis_(!flip_axis ? 1.0 : -1.0) {}

JointBuilder::JointBuilder(const float offset, const bool flip_axis) : joint(offset, flip_axis) {}
    JointBuilder JointBuilder::addPhysicalLimits(const float physical_lower_limit, const float physical_upper_limit) {
    joint.lower_limit_ = joint.fromPhysicalAngle(joint.flip_axis_ > 1 ? physical_lower_limit : physical_upper_limit);
    joint.upper_limit_ = joint.fromPhysicalAngle(joint.flip_axis_ > 1? physical_upper_limit : physical_lower_limit);
    return *this;
}

JointBuilder JointBuilder::addModelLimits(const float model_lower_limit, const float model_upper_limit) {
    joint.lower_limit_ = model_lower_limit;
    joint.upper_limit_ = model_upper_limit;
    return *this;
}

JointBuilder JointBuilder::setPhysicalAngle(float physical_angle) {
    joint.set(joint.fromPhysicalAngle(physical_angle));
    return *this;
}

JointBuilder JointBuilder::setModelAngle(float model_angle) {
    joint.set(model_angle);
    return *this;
}

Joint JointBuilder::create() const { return joint; } // TODO any checks?

// TODO should it be allowed to provide an angle outside the limits (it was before)
// TODO can we make it so that the limits have to be provided first?

} // namespace hexapod
