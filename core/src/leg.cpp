#include "leg.h"

#include "kinematics_support.h"
#include "transformations.h"
#include "joint.h"

#ifdef __AVR__
#include <Arduino.h>
#else
#include <cstddef>
#include <cmath>
#include <iostream>
#endif

namespace hexapod {

using namespace util;

Leg::Leg() {}

Leg::Leg(Dims dims, Joint *joints)
    : dims_(dims), neutral_pos_{(dims.a + dims.b + dims.c) * 2.0f / 3.0f, 0.0f, 0.0f}, step_idx_{0} {
  joints_[JOINT_1] = joints[0];
  joints_[JOINT_2] = joints[1];
  joints_[JOINT_3] = joints[2];
  setJointAngles({joints_[JOINT_1].angle_, joints_[JOINT_2].angle_, joints_[JOINT_3].angle_});
  updateMovementLimits(dims_.c / 2.0); // just a guess at the actual height
}

/**
 * @details
 *
 * @param angles The joint angles to check
 * @param pos The foot position to check against
 * @return true ff matching
 */
bool Leg::validateJointAngles(const JointAngles& angles, const Vector3& pos) {
  Vector3 pos_from_ik;
  calculateFootPosition(angles, pos_from_ik);
  return comparePositions(pos, pos_from_ik);
}

/**
 * @details
 *
 * @param pos Required foot position
 * @param[out] angles Array of joint angles to store the results
 * @return uint8_t Number of valid results
 */
uint8_t Leg::calculateJointAnglesFull(const Vector3& pos, JointAngles angles[2]) {
  bool angles_valid[2] = {false, false};  // only used after joint 3 calculated

  // ******* JOINT 1 *******
  // Does not handle beyond (-90,90) (including boundary)
  angles[0].theta_1 = atan2(pos.y(), pos.x());
  angles[0].theta_1 = wrapAngle(angles[0].theta_1);
  if (joints_[JOINT_1].isWithinLimits(angles[0].theta_1)) {
    angles[0].theta_1 = joints_[JOINT_1].clampToLimts(angles[0].theta_1);
  } else {
    // TODO change to be consistent with approach for joint 2
    const float test_angle_1 = angles[0].theta_1 + M_PI;
    const float test_angle_2 = angles[0].theta_1 - M_PI;
    if (joints_[JOINT_1].isWithinLimits(test_angle_1)) {
      angles[0].theta_1 = test_angle_1;
    } else if (joints_[JOINT_1].isWithinLimits(test_angle_2)) {
      angles[0].theta_1 = test_angle_2;
    } else {
      // no viable options for joint angle
      return 0;
    }
  }
  // only one option for J1
  angles[1].theta_1 = angles[0].theta_1;

  // ******* JOINT 3 *******
  float ha;
  // TODO is there a more efficient way?
  // slightly, can check |x| > |y| and use cos(theta1) if so
  // eliminates 1 trig call
  const float cos_theta_1 = cos(angles[0].theta_1);
  const float sin_theta_1 = sin(angles[0].theta_1);
  if (cos_theta_1 > fabs(sin_theta_1)) {
    ha = pos.x() / cos_theta_1 - dims_.a;
  } else {
    ha = pos.y() / sin_theta_1 - dims_.a;
  }
  float ka = (pos.z() * pos.z() + ha * ha - dims_.b * dims_.b - dims_.c * dims_.c) /
      (2 * dims_.b * dims_.c);

  if (clamp(ka, -1.0f, 1.0f)) {
    angles[0].theta_3 = acos(ka);
    angles[1].theta_3 = -angles[0].theta_3;

    for (uint8_t i = 0; i < 2; i++) {
      angles[i].theta_3 = wrapAngle(angles[i].theta_3);
      if (joints_[JOINT_3].isWithinLimits(angles[i].theta_3)) {
        angles[i].theta_3 = joints_[JOINT_3].clampToLimts(angles[i].theta_3);
        angles_valid[i] = true;
      }
    }
  }
  if (!angles_valid[0] && !angles_valid[1]) {
    return 0;
  }

  // ******* JOINT 2 *******
  // could use law of cosines again if quicker than atan version
  for (uint8_t i = 0; i < 2; i++) {
    if (angles_valid[i]) {
      const float kb = dims_.b + dims_.c * cos(angles[i].theta_3);
      const float kc = dims_.c * sin(angles[i].theta_3);
      angles[i].theta_2 = atan2(pos.z(), ha) - atan2(kc, kb);
      angles[i].theta_2 = wrapAngle(angles[i].theta_2);
      if (joints_[JOINT_2].isWithinLimits(angles[i].theta_2)) {
        angles[i].theta_2 = joints_[JOINT_2].clampToLimts(angles[i].theta_2);
        // possible that it's within the limits but not the one we want,
        //   still need to check the +/- PI option
        if (!validateJointAngles(angles[i], pos)) {
          if (angles[i].theta_2 < 0.0f) {
            angles[i].theta_2 += M_PI;
          } else if (angles[i].theta_2 > 0.0f) {
            angles[i].theta_2 -= M_PI;
          }
          if (!validateJointAngles(angles[i], pos)) {
            angles_valid[i] = false;
          }
        }
      } else {
        angles_valid[i] = false;
      }
    }
  }

  // clunky
  uint8_t num_results = 0;
  if (angles_valid[0]) {
    num_results++;
  }
  if (angles_valid[1]) {
    num_results++;
    if (num_results == 1) {
      // if this is the only result, move into the first position
      angles[0] = angles[1];
    }
  }
  return num_results;
}

/**
 * @details
 * To avoid choosing a technically correct but undesirable set of joint angles while walking,
 *  this version of inverse kinematics restricts joint 3 to being negative.
 *
 * A similar effect could be achieved through setting the appropriate joint limits but
 *  hopefully this approach allows a bit more flexibility and the option to use IK options elsewhere
 * if required
 *
 * @param pos Required foot position
 * @param[out] result_angles Joint angles to store the results
 * @return uint8_t Number of valid results (0 or 1)
 */
uint8_t Leg::calculateJointAnglesWalk(const Vector3& pos, JointAngles& result_angles) {
  // ******* JOINT 1 *******
  // Does not handle beyond (-90,90) (including boundary)
  float th1, th2, th3;
  th1 = atan2(pos.y(), pos.x());

  if (joints_[JOINT_1].isWithinLimits(th1)) {
    th1 = joints_[JOINT_1].clampToLimts(th1);
  } else {
    return 0;
  }

  // ******* JOINT 3 *******
  float ha;
  if (fabs(pos.x()) > fabs(pos.y())) {
    ha = pos.x() / cos(th1) - dims_.a;
  } else {
    ha = pos.y() / sin(th1) - dims_.a;
  }
  float ka = (pos.z() * pos.z() + ha * ha - dims_.b * dims_.b - dims_.c * dims_.c) /
      (2 * dims_.b * dims_.c);
  if (clamp(ka, -1.0f, 1.0f)) {
    th3 = -acos(ka);  // acos result always positive (between 0 and pi)
    // but the negative version also valid
    // and for walking, we want to option where J3 is negative
    if (joints_[JOINT_3].isWithinLimits(th3)) {
      th3 = joints_[JOINT_3].clampToLimts(th3);
    } else {
      return 0;
    }
  } else {
    return 0;
  }

  // ******* JOINT 2 *******
  const float kb = dims_.b + dims_.c * cos(th3);
  const float kc = dims_.c * sin(th3);
  th2 = atan2(pos.z(), ha) - atan2(kc, kb);
  th2 = wrapAngle(th2);
  if (joints_[JOINT_2].isWithinLimits(th2)) {
    th2 = joints_[JOINT_2].clampToLimts(th2);
  }

  JointAngles tmp{th1, th2, th3};
  if (!validateJointAngles(tmp, pos)) {
    return 0;
  }

  result_angles = tmp;
  return 1;  // max 1 result
}

/**
 * @details
 *
 * Wrapper for IK which choses the appropriate IK function based on the selected IKMode.
 * May also apply selection logic if multiple angle options available.
 *
 * @param pos Required foot position
 * @param ik_mode
 * @param [out] calculated_angles
 * @return true if valid angles were found
 */
bool Leg::calculateJointAngles(const Vector3& pos, const IKMode ik_mode, JointAngles& calculated_angles) {
  if (ik_mode == IKMode::FULL) {
    JointAngles anglesFull[2];
    const uint8_t num_results = calculateJointAnglesFull(pos, anglesFull);
    if (num_results > 0) {
      const uint8_t chosen_idx = chooseJointAnglesNearest(anglesFull, num_results, getJointAngles());
      calculated_angles = anglesFull[chosen_idx];
      return true;
    } else {
      return false;
    }
  } else if (ik_mode == IKMode::WALK) {
    return calculateJointAnglesWalk(pos, calculated_angles);
  } else {
    return false;
  }
}

bool Leg::calculateJointAngles(const Vector3& pos, const IKMode ik_mode) {
  return calculateJointAngles(pos, ik_mode, staged_angles_);
}


/**
 * @details
 *
 * @param joint_angles Angles to test
 * @return true if all joints within their respective joint limits
 * @return false
 */
bool Leg::jointsWithinLimits(const JointAngles& joint_angles) const {
  return (joints_[JOINT_1].isWithinLimits(joint_angles.theta_1) &&
      joints_[JOINT_2].isWithinLimits(joint_angles.theta_2) &&
      joints_[JOINT_3].isWithinLimits(joint_angles.theta_3));
}

/**
 * @details
 * Forward kinematics on joint angles to find the associated foot position. The provided joint angles MUST be valid!
 *
 * @param angles Joint angles to calculate position for - ASSUME THEY ARE VALID
 * @param[out] pos Calculated foot position
 */
void Leg::calculateFootPosition(const JointAngles& angles, Vector3& pos) {
  const float h =
      dims_.a + dims_.b * cos(angles.theta_2) + dims_.c * cos(angles.theta_2 + angles.theta_3);
  pos.x() = h * cos(angles.theta_1);
  pos.y() = h * sin(angles.theta_1);
  pos.z() = dims_.b * sin(angles.theta_2) + dims_.c * sin(angles.theta_2 + angles.theta_3);
}

void Leg::updateFootPosition() { calculateFootPosition(getJointAngles(), current_pos_); }

/**
 * @details
 * Sets the leg joint angles and recalculates the current foot position.
 * The provided joint angles MUST be valid - they are not checked.
 *
 * @param angles
 */
void Leg::setJointAngles(const JointAngles& angles) {
  joints_[JOINT_1].angle_ = angles.theta_1;
  joints_[JOINT_2].angle_ = angles.theta_2;
  joints_[JOINT_3].angle_ = angles.theta_3;
  staged_angles_ = angles; // keep in sync in case we've done a 'direct' update (or anything else that hasn't used IK to calculated first)
  updateFootPosition();
}

void Leg::setJointAnglesFromPhysical(const JointAngles& angles) {
  setJointAngles(fromPhysicalAngles(angles));
}

Leg::JointAngles Leg::getStagedAngles() const { return staged_angles_; }

void Leg::setStagedAngles(const JointAngles& angles) {
  staged_angles_ = angles;
}

void Leg::applyStagedAngles() { setJointAngles(staged_angles_); }

void Leg::resetStagedAngles() {
  staged_angles_ = getJointAngles();
}

/**
 * @details
 * Given an array of 2 joint angle options, return the index of the option closest to the reference
 * angles provided.
 *
 * Currently 'closest' is based solely on joint 2. Only expected to have to deal with relatively
 * small differences in joint angles.
 *
 * @param angle_options Array of angles to choose from (maximum 2)
 * @param num_valid Number of valid angles in array
 * @param ref_angles Reference angles to compare to
 * @return uint8_t Index of the angle_options closest to the ref_angles angles
 */
uint8_t Leg::chooseJointAnglesNearest(const JointAngles angle_options[2], uint8_t num_valid,
                                      const JointAngles& ref_angles) const {
  if (num_valid == 1) {
    return 0;
  } else {
    // joint 2 alone should be sufficient on which to make this decision
    if (fabs(angle_options[0].theta_2 - ref_angles.theta_2) <
        fabs(angle_options[1].theta_2 - ref_angles.theta_2)) {
      return 0;
    } else {
      return 1;
    }
  }
}

Vector3 Leg::getFootPosition() const { return current_pos_; }

Vector3 Leg::getNeutralPosition() const { return neutral_pos_; }

Vector3& Leg::getNeutralPosition() { return neutral_pos_; }

Vector3 Leg::getTargetPosition() const { return target_pos_; }

Vector3 Leg::getRaisedPosition() const { return raised_pos_; }

/**
 * @details
 * Calculate a trajectory for the leg in joint space. The trajectory will take the leg from the
 * ground,
 * move forward and up towards step_apex_angles_
 * and then forward and down back to target_angles_ (on the ground)
 *
 * This function will always be called when the foot is first about to be raised.
 *
 * It may also be called during the trajectory if the targets have changed
 * (in the leg frame - may still be stationary in a more global frame),
 * in which case the trajectory will be recalculated from the current position
 * (which may not lie on the old trajectory anymore).
 */
void Leg::calculateTrajectory() {
  // Use staged_angles_ as the latest version of the angles in the current calculation period
  const JointAngles& current_joint_angles = staged_angles_;

  // the foot must go up
  // (remember, current_step_duration_ always even)
  const float steps_up = static_cast<float>(fmax((current_step_duration_ / 2) - step_idx_, 0));

  // if (steps_up > 0) {
  inc_up_angles_.theta_1 = (step_apex_angles_.theta_1 - current_joint_angles.theta_1) / steps_up;
  inc_up_angles_.theta_2 = (step_apex_angles_.theta_2 - current_joint_angles.theta_2) / steps_up;
  inc_up_angles_.theta_3 = (step_apex_angles_.theta_3 - current_joint_angles.theta_3) / steps_up;
  // } // else { doesn't matter }

  // and then down
  const float steps_down = static_cast<float>(fmin((current_step_duration_ / 2), current_step_duration_ - step_idx_));
  if (step_idx_ < current_step_duration_ / 2) {
    // still moving up
    inc_down_angles_.theta_1 = (target_angles_.theta_1 - step_apex_angles_.theta_1) / steps_down;
    inc_down_angles_.theta_2 = (target_angles_.theta_2 - step_apex_angles_.theta_2) / steps_down;
    inc_down_angles_.theta_3 = (target_angles_.theta_3 - step_apex_angles_.theta_3) / steps_down;
  } else {
    // now moving down
    inc_down_angles_.theta_1 = (target_angles_.theta_1 - current_joint_angles.theta_1) / steps_down;
    inc_down_angles_.theta_2 = (target_angles_.theta_2 - current_joint_angles.theta_2) / steps_down;
    inc_down_angles_.theta_3 = (target_angles_.theta_3 - current_joint_angles.theta_3) / steps_down;
  }
}

void Leg::setTrajectory(const Leg::JointAngles& target,
                   const Leg::JointAngles& increment_up,
                   const Leg::JointAngles& midpoint,
                   const Leg::JointAngles& increment_down,
                   const uint16_t duration) {
  step_idx_ = 0;
  current_step_duration_ = duration;
  target_angles_ = target;
  inc_up_angles_ = increment_up;
  step_apex_angles_ = midpoint;
  inc_down_angles_ = increment_down;
}


/**
 * @details
 * Apply the pre-calculated joint increments to the current STAGED joint angles.
 *
 * If the trajectory is at the midpoint or end, set the STAGED joint angles directly to those values
 *  to avoid any rounding errors.
 */
void Leg::incrementLeg() {
  // Use staged_angles_ as the latest version of the angles in the current calculation period
  JointAngles& current_joint_angles = staged_angles_;
  if (step_idx_ == current_step_duration_ - 1) {
    current_joint_angles.theta_1 = target_angles_.theta_1;
    current_joint_angles.theta_2 = target_angles_.theta_2;
    current_joint_angles.theta_3 = target_angles_.theta_3;
  } else if (step_idx_ == (current_step_duration_ / 2) - 1) {
    current_joint_angles.theta_1 = step_apex_angles_.theta_1;
    current_joint_angles.theta_2 = step_apex_angles_.theta_2;
    current_joint_angles.theta_3 = step_apex_angles_.theta_3;
  } else if (step_idx_ < (current_step_duration_ / 2)) {
    current_joint_angles.theta_1 += inc_up_angles_.theta_1;
    current_joint_angles.theta_2 += inc_up_angles_.theta_2;
    current_joint_angles.theta_3 += inc_up_angles_.theta_3;
  } else if (step_idx_ < current_step_duration_) {
    current_joint_angles.theta_1 += inc_down_angles_.theta_1;
    current_joint_angles.theta_2 += inc_down_angles_.theta_2;
    current_joint_angles.theta_3 += inc_down_angles_.theta_3;
  } else {
    return; // trajectory has finished, incrementLeg() should have no effect
  }
  step_idx_++;
}

Leg::MovementLimits Leg::calculateMovementLimits(const float height) {

  // start off by finding the limits in x/y(in leg frame)
  // maybe could consider as diamond
  // later may want to to something more complex (approximate circle?)
  Vector3 neutral = getNeutralPosition(); // TODO this is actually callng the non-const version and returning a modifyable ref
  neutral.z() = -height;

  MovementLimits leg_movement_limits{neutral.x(), neutral.x(), neutral.y(), neutral.y()};
  Leg::JointAngles ik_result_angles;

  // Sense check that neutral position is achievable!
  if (!calculateJointAngles(neutral, Leg::IKMode::WALK, ik_result_angles)) {
    return leg_movement_limits;
  }

  const float max_extension = dims_.a + dims_.b + dims_.c;
  float test_value;

//  float test_value = max_extension;
  // 100 steps for now
  // start at the absolute limit
  // MAX X
  test_value = max_extension;
  while (test_value > neutral.x()) {
    Vector3 test_position = neutral;
    test_position.x() = test_value;
    if (calculateJointAngles(test_position, Leg::IKMode::WALK, ik_result_angles)) {
      leg_movement_limits.x_max = test_value;
      break;
    }
    test_value -= max_extension/100.0f;
  }

  test_value = -max_extension;
  while (test_value < neutral.x()) {
    Vector3 test_position = neutral;
    test_position.x() = test_value;
    if (calculateJointAngles(test_position, Leg::IKMode::WALK, ik_result_angles)) {
      leg_movement_limits.x_min = test_value;
      break;
    }
    test_value += max_extension/100.0f;
  }

  const float max_y_extension = sqrt(max_extension * max_extension + neutral.x() * neutral.x());
  test_value = max_y_extension;
  while (test_value > neutral.y()) {
    Vector3 test_position = neutral;
    test_position.y() = test_value;
    if (calculateJointAngles(test_position, Leg::IKMode::WALK, ik_result_angles)) {
      leg_movement_limits.y_max = test_value;
      break;
    }
    test_value -= max_y_extension/100.0f;
  }

  test_value = -max_y_extension;
  while (test_value < neutral.y()) {
    Vector3 test_position = neutral;
    test_position.y() = test_value;
    if (calculateJointAngles(test_position, Leg::IKMode::WALK, ik_result_angles)) {
      leg_movement_limits.y_min = test_value;
      break;
    }
    test_value += max_y_extension/100.0f;
  }

  return leg_movement_limits;
}

void Leg::updateMovementLimits(const float height) {
  movement_limits_ = calculateMovementLimits(height);
}

/**
 * @details Clamp a position to within the currently set movement_limits area
 * The movement_limits area describes a diamond which approximates the true area (probably ellipsoidal).
 * The movement limits are calculated for a specific foot Z position. If this changes, the limits would need to be recalculated,
 *  but for the moment this is just intended to be quite rough.
 * If clamped, the modified position will be in the same direction from the neutral position as the original, but
 *  on the border of the diamond.
 * Only consider XY.
 * @param the target position, which may be modified if outside allowed area
 * @return true if modified
 */
bool Leg::clampTarget(Vector3& target_position) const {
  // TODO include some simple checks to see if inside max circle within diamond, and then return unmodified?
  Vector3& orig_target_position = target_position;
  // where is target relative to neutral
  Vector3 neutral = neutral_pos_;
  neutral.z() = orig_target_position.z();
  const Vector3 movement = orig_target_position - neutral;
  if (util::comparePositions(movement, Vector3{0.0, 0.0, 0.0})) {
    return false; // zero vector, nothing to clamp
  }

  // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection Given two points on each line
  auto findIntersection = [&](const Vector3& ep1, const Vector3& ep2) -> Vector3 {
    // 1 edge_point_1
    // 2 edge_point_2
    // 3 zero
    // 4 movement
    const float D = (ep1.x() - ep2.x()) * (-movement.y()) - (ep1.y() - ep2.y()) * (-movement.x());
    const float k = (ep1.x()*ep2.y() - ep1.y()*ep2.x());
    const float x = (k * (-movement.x())) / D;
    const float y = (k * (-movement.y())) / D;
    return Vector3{x, y , 0.0};
  };

  // which side of the diamond will the movement vector pass through (if it extends far enough)
  // note movement limits are given in leg frame, not relative to neutral position
  Vector3 intersection;
  if (movement.x() >= 0.0f) {
    const Vector3 xmax{movement_limits_.x_max - neutral.x(), 0.0, 0.0};
    if (movement.y() >= 0.0f) {
      // line between xmax and ymax
      const Vector3 ymax{0.0, movement_limits_.y_max - neutral.y(), 0.0};
      intersection = findIntersection(xmax, ymax);
    } else {
      // line between xmax and ymin
      const Vector3 ymin{0.0, movement_limits_.y_min - neutral.y(), 0.0};
      intersection = findIntersection(xmax, ymin);
    }
  } else {
    const Vector3 xmin{movement_limits_.x_min - neutral.x(), 0.0, 0.0};
    if (movement.y() >= 0.0f) {
      // line between xmin and ymax
      const Vector3 ymax{0.0, movement_limits_.y_max - neutral.y(), 0.0};
      intersection = findIntersection(xmin, ymax);
    } else {
      // line between xmin and ymin
      const Vector3 ymin{0.0, movement_limits_.y_min - neutral.y(), 0.0};
      intersection = findIntersection(xmin, ymin);
    }
  }

  auto twoDNormSquared = [](const Vector3& v){
    return v.x()*v.x() + v.y()*v.y();
  };

  if (twoDNormSquared(movement) > twoDNormSquared(intersection)) {
    target_position = neutral + intersection;
    return true;
  }

  return false;
}

/**
 * @details
 * To be run for every period the leg is raised. Can be called if not raised - will not do anything.
 *
 * When foot has just entered raised state, or the target positions have been updated,
 *  call IK routine to find the associated joint angles, then calculate the trajectory between these
 * joint positions.
 * Then update the leg's joint angles.
 *
 * @return true if any required joint angles were calculated successfully and the leg joint angles
 * were updated
 */
bool Leg::stepUpdate() {
  // TODO check if foot air time has changed, may need to change speed of leg
  if (state_ == State::ON_GROUND) {
    return false;
  }

  // if leg has only just transitioned to RAISED then need to calculate trajectory
  if (target_updated_ && prev_state_ == State::ON_GROUND) {
    step_idx_ = 0;                                // reset
    current_step_duration_ = new_step_duration_;  // save for later when doing actual movement
  }

  if (target_updated_) {
    // update target angles
    if (!calculateJointAngles(target_pos_, IKMode::WALK, target_angles_)) {
#ifndef __AVR__
      std::cout << "Could not calculate joint angles at requested foot target position\n";
#endif
      return false;
    }
    // update trajectory apex_angles_
    if (!calculateJointAngles(raised_pos_, IKMode::WALK, step_apex_angles_)) {
#ifndef __AVR__
      std::cout << "Could not calculate joint angles at requested raise limit position\n";
#endif
      return false;
    }

    calculateTrajectory();
  }

  incrementLeg();
  target_updated_ = false;
  return true;
}

/**
 * @details
 * Will change status from raised to on ground if it has reached the end of its raised trajectory.
 *
 * Will change status from on ground to raised if a raise is requested by input param, and it's not
 * already at its target position.
 *  If it is already at its target it will not raise even if requested.
 *
 * @param raise indicates if the leg should raise if possible
 * @return true if the leg changes to raised, or skipped raising
 */
bool Leg::updateStatus(const bool raise) {
  prev_state_ = state_;
  bool result = false;
  if (state_ == State::RAISED && step_idx_ == current_step_duration_) {
    state_ = State::ON_GROUND;
    step_idx_ = 0;
  }
    // If it's already at its target then don't lift but return true as if it had
  else if (raise && state_ == State::ON_GROUND) {
    const float d = (current_pos_ - target_pos_).norm();
    if (!compareFloat(d, 0.0f, target_tolerance)) {
      state_ = State::RAISED;
      step_idx_ = 0; // TODO review where this is set throughout
    }
    result = true;
  }
  return result;
}

/**
 * @details
 * Update the leg raise trajectory target positions, and flag that they've been updated.
 *
 * @param target_pos Target position to place the foot on the ground
 * @param raised_pos Apex of the leg raise movement
 * @param foot_air_time
 */
void Leg::updateTargets(const Vector3& target_pos, const Vector3& raised_pos,
                        uint16_t foot_air_time) {
  target_pos_ = target_pos;
  raised_pos_ = raised_pos;
  new_step_duration_ = foot_air_time;
  target_updated_ = true;
}

Leg::JointAngles Leg::getJointAngles() const {
  return JointAngles{joints_[JOINT_1].angle_, joints_[JOINT_2].angle_, joints_[JOINT_3].angle_};
}

Leg::JointAngles Leg::getJointAnglesPhysical() const {
  return JointAngles{joints_[JOINT_1].toPhysicalAngle(),
                     joints_[JOINT_2].toPhysicalAngle(),
                     joints_[JOINT_3].toPhysicalAngle()};
}


Leg::JointAngles Leg::fromPhysicalAngles(const Leg::JointAngles& physical_angles) const {
  return Leg::JointAngles{joints_[JOINT_1].fromPhysicalAngle(physical_angles.theta_1),
                          joints_[JOINT_2].fromPhysicalAngle(physical_angles.theta_2),
                          joints_[JOINT_3].fromPhysicalAngle(physical_angles.theta_3)};
}

Leg::JointAngles Leg::toPhysicalAngles(const Leg::JointAngles& model_angles) const {
  return Leg::JointAngles{joints_[JOINT_1].toPhysicalAngle(model_angles.theta_1),
                          joints_[JOINT_2].toPhysicalAngle(model_angles.theta_2),
                          joints_[JOINT_3].toPhysicalAngle(model_angles.theta_3)};
}


void Leg::setStartingAngles(Leg::JointAngles starting_angles) {
  setJointAngles(starting_angles);
  target_pos_ = current_pos_;
}

uint16_t Leg::getStepIdx() const { return step_idx_; }

uint16_t Leg::getCurrentStepDuration() const { return current_step_duration_; }

/**
 * @details If the leg is currently raised, return the percentage through the current trajectory.
 *  Otherwise return 1 (i.e. trajectory complete).
 * @return percentage progress through trajectory (0 to 1)
 */
float Leg::getCurrentStepProgress() const{
  if (state_ == State::ON_GROUND) { return 1.0f; }
  else if (step_idx_ == 0) { return 0.0; }
  else { return static_cast<float>(step_idx_) / static_cast<float>(current_step_duration_); }
}

} // namespace hexapod
