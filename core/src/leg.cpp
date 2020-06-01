#include "leg.h"

#include "kinematics_support.h"
#include "transformations.h"

#include <stddef.h>
#include <cmath>
#include <iostream>

using namespace KinematicsSupport;
using namespace Transformations;

Joint::Joint() : lower_limit_(-1.48f), upper_limit_(1.48f), angle_(0.0f) {}

Joint::Joint(float lower_limit, float upper_limit)
    : lower_limit_(lower_limit), upper_limit_(upper_limit), angle_(0.0f) {}

Joint::Joint(float lower_limit, float upper_limit, float angle)
    : lower_limit_(lower_limit), upper_limit_(upper_limit), angle_(angle) {}

bool Joint::isWithinLimits(const float angle) const {
  return (angle >= lower_limit_ - KinematicsSupport::eps) &&
         (angle < upper_limit_ + KinematicsSupport::eps);
}

float Joint::clampToLimts(float angle) const {
  return fmax(fmin(angle, upper_limit_), lower_limit_);
}

Leg::Leg() {}

Leg::Leg(Dims dims, Joint* joints)
  : dims_(dims)
{
  joints_[JOINT_1] = joints[0];
  joints_[JOINT_2] = joints[1];
  joints_[JOINT_3] = joints[2];
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
 * @return size_t Number of valid results
 */
size_t Leg::calculateJointAnglesFull(const Vector3& pos, JointAngles angles[2]) {
  bool angles_valid[2] = {false, false};  // only used after joint 3 calculated

  // ******* JOINT 1 *******
  // Does not handle beyond (-90,90) (including boundary)
  angles[0].theta_1 = atan2(pos.y(), pos.x());
  angles[0].theta_1 = wrapAngle(angles[0].theta_1);
  if (joints_[JOINT_1].isWithinLimits(angles[0].theta_1)) {
    angles[0].theta_1 = joints_[JOINT_1].clampToLimts(angles[0].theta_1);
  } else {
    // TODO change to be consistent with approach for joint 2
    float test_angle_1 = angles[0].theta_1 + M_PI;
    float test_angle_2 = angles[0].theta_1 - M_PI;
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
  float cos_theta_1 = cos(angles[0].theta_1);
  float sin_theta_1 = sin(angles[0].theta_1);
  if (cos_theta_1 > fabs(sin_theta_1)) {
    ha = pos.x() / cos_theta_1 - dims_.a;
  } else {
    ha = pos.y() / sin_theta_1 - dims_.a;
  }
  float ka = (pos.z() * pos.z() + ha * ha - dims_.b * dims_.b - dims_.c * dims_.c) / (2 * dims_.b * dims_.c);

  if (clamp(ka, -1.0f, 1.0f)) {
    angles[0].theta_3 = acos(ka);
    angles[1].theta_3 = -angles[0].theta_3;

    for (size_t i = 0; i < 2; i++) {
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
  for (size_t i = 0; i < 2; i++) {
    if (angles_valid[i]) {
      float kb = dims_.b + dims_.c * cos(angles[i].theta_3);
      float kc = dims_.c * sin(angles[i].theta_3);
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
  size_t num_results = 0;
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
 * To avoid choosing an technically correct but undesirable set of joint angles while walking,
 *  this version of inverse kinematics restricts joint 3 to being negative.
 * 
 * A similar effect could be achieved through setting the appropriate joint limits but
 *  hopefully this approach allows a bit more flexibility and the option to use IK options elsewhere if required
 * 
 * @param pos Required foot position
 * @param[out] angles Joint angles to store the results
 * @return size_t Number of valid results (0 or 1)
 */
size_t Leg::calculateJointAnglesWalk(const Vector3& pos, JointAngles& result_angles) {
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
  float ka = (pos.z() * pos.z() + ha * ha - dims_.b * dims_.b - dims_.c * dims_.c) / (2 * dims_.b * dims_.c);
  if (clamp(ka, -1.0f, 1.0f)) {
    th3 = -acos(ka); // acos result always positive (between 0 and pi)
                     // but the negative version also valid
                     // and for walking, we want to option where J3 is negative
    if (joints_[JOINT_3].isWithinLimits(th3)) {
        th3 = joints_[JOINT_3].clampToLimts(th3);
    }
    else {
      return 0;
    }
  }
  else {
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

  JointAngles tmp {th1, th2, th3};
  if (!validateJointAngles(tmp, pos)) {
    return 0;
  }

  result_angles = tmp;
  return 1; // max 1 result
}

// result_angles are not modified if no result found
/**
 * @details
 * 
 * Wrapper for IK which choses the appropriate IK function based on the selected IKMode.
 * May also apply selection logic if multiple angle options available.
 * 
 * @param pos Required foot position
 * @param[out] result_angles Contains the result if found, not modified if no valid results
 * @param ik_mode 
 * @return true if valid angles were found
 */
bool Leg::calculateJointAngles(const Vector3& pos, JointAngles& result_angles, const IKMode ik_mode) {
  if (ik_mode == IKMode::FULL) {
    JointAngles anglesFull[2];
    size_t num_results = calculateJointAnglesFull(pos, anglesFull);
    if (num_results > 0) {
      size_t chosen_idx = chooseJointAnglesNearest(anglesFull, num_results, getJointAngles());
      result_angles = anglesFull[chosen_idx];
      return true;
    } else {
      return false;
    }
  }
  else if (ik_mode == IKMode::WALK) {
    return calculateJointAnglesWalk(pos, result_angles);
  }
  else {
    return false;
  }
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
 * Forward kinematics on joint angles to find the associated foot position.
 * 
 * @param angles Joint angles to calculate position for
 * @param[out] pos Calculated foot position
 * @return true always TODO make void or add checks
 */
bool Leg::calculateFootPosition(const JointAngles& angles, Vector3& pos) {
  float h = dims_.a + dims_.b * cos(angles.theta_2) + dims_.c * cos(angles.theta_2 + angles.theta_3);
  pos.x() = h * cos(angles.theta_1);
  pos.y() = h * sin(angles.theta_1);
  pos.z() = dims_.b * sin(angles.theta_2) + dims_.c * sin(angles.theta_2 + angles.theta_3);
  return true;
}

/**
 * @details
 * 
 * @return true always
 */
bool Leg::updateFootPosition() {
  return calculateFootPosition(getJointAngles(), pos_);
}

/**
 * @details
 * Sets the leg joint angles and recalculates the current foot position.
 * 
 * No checks on input.
 * 
 * @param angles 
 * @return true always
 */
bool Leg::setJointAngles(const JointAngles& angles) {
  joints_[JOINT_1].angle_ = angles.theta_1;
  joints_[JOINT_2].angle_ = angles.theta_2;
  joints_[JOINT_3].angle_ = angles.theta_3;
  updateFootPosition();
  return true;
}

/**
 * @details
 * Given an array of 2 joint angle options, return the index of the option closest to the reference angles provided.
 * 
 * Currently 'closest' is based solely on joint 2. Only expected to have to deal with relatively small differences in joint angles.
 * 
 * @param angle_options Array of angles to choose from (maximum 2)
 * @param num_valid Number of valid angles in array
 * @param ref_angles Reference angles to compare to
 * @return size_t Index of the angle_options closest to the ref_angles angles
 */
size_t Leg::chooseJointAnglesNearest(const JointAngles angle_options[2], size_t num_valid,
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

Vector3 Leg::getFootPosition() const { return pos_; }

/**
 * @details
 * Calculate a trajectory for the leg in joint space. The trajectory will take the leg from the ground, 
 * move forward and up towards joint_targets_raised_ 
 * and then forward and down back to joint_targets_ (on the ground)
 * 
 * This function will always be called when the foot is first about to be raised.
 * 
 * It may also be called during the trajectory if the targets have changed 
 * (in the leg frame - may still be stationary in a more global frame),
 * in which case the trajectory will be recalculated from the current position 
 * (which may not lie on the old trajectory anymore).
 */
void Leg::calculateTrajectory() {
  JointAngles current_joint_angles = getJointAngles();

  // the foot must go up
  // (remember, current_foot_air_time_ always even)
  float steps_up = (float)fmax((current_foot_air_time_/2) - step_no_, 0);

  // if (steps_up > 0) {
    joint_inc_up_.theta_1 = (joint_targets_raised_.theta_1 - current_joint_angles.theta_1) /
                              steps_up;
    joint_inc_up_.theta_2 = (joint_targets_raised_.theta_2 - current_joint_angles.theta_2) /
                            steps_up;
    joint_inc_up_.theta_3 = (joint_targets_raised_.theta_3 - current_joint_angles.theta_3) /
                            steps_up;
  // } // else { doesn't matter }

  // and then down
  float steps_down = (float)fmin((current_foot_air_time_/2), current_foot_air_time_ - step_no_);
  if (step_no_ < current_foot_air_time_/2) {
    // still moving up
    joint_inc_down_.theta_1 = (joint_targets_.theta_1 - joint_targets_raised_.theta_1) /
                              steps_down;
    joint_inc_down_.theta_2 = (joint_targets_.theta_2 - joint_targets_raised_.theta_2) /
                              steps_down;
    joint_inc_down_.theta_3 = (joint_targets_.theta_3 - joint_targets_raised_.theta_3) /
                              steps_down;
  }
  else {
    // now moving down
    joint_inc_down_.theta_1 = (joint_targets_.theta_1 - current_joint_angles.theta_1) /
                              steps_down;
    joint_inc_down_.theta_2 = (joint_targets_.theta_2 - current_joint_angles.theta_2) /
                            steps_down;
    joint_inc_down_.theta_3 = (joint_targets_.theta_3 - current_joint_angles.theta_3) /
                            steps_down;
  }
}

/**
 * @details
 * Apply the pre-calculated joint increments to the current joint angles.
 * 
 * If the trajectory is at the midpoint or end, set the joint angles directly to those values
 *  to avoid any rounding errors.
 */
void Leg::incrementLeg() {
  JointAngles current_joint_angles = getJointAngles();
  if (step_no_ == current_foot_air_time_ - 1) {
    current_joint_angles.theta_1 = joint_targets_.theta_1;
    current_joint_angles.theta_2 = joint_targets_.theta_2;
    current_joint_angles.theta_3 = joint_targets_.theta_3;
  } else if (step_no_ == (current_foot_air_time_ / 2) - 1) {
    current_joint_angles.theta_1 = joint_targets_raised_.theta_1;
    current_joint_angles.theta_2 = joint_targets_raised_.theta_2;
    current_joint_angles.theta_3 = joint_targets_raised_.theta_3;
  }
  else if (step_no_ < (current_foot_air_time_ / 2)) {
    current_joint_angles.theta_1 += joint_inc_up_.theta_1;
    current_joint_angles.theta_2 += joint_inc_up_.theta_2;
    current_joint_angles.theta_3 += joint_inc_up_.theta_3;
  } else {
    current_joint_angles.theta_1 += joint_inc_down_.theta_1;
    current_joint_angles.theta_2 += joint_inc_down_.theta_2;
    current_joint_angles.theta_3 += joint_inc_down_.theta_3;
  }
  setJointAngles(current_joint_angles);
}


/**
 * @details
 * To be run for every period the leg is raised. Can be called if not raised - will not do anything.
 * 
 * When foot has just entered raised state, or the target positions have been updated, 
 *  call IK routine to find the associated joint angles, then calculate the trajectory between these joint positions.
 * Then update the leg's joint angles.
 * 
 * @return true if any required joint angles were calculated successfully and the leg joint angles were updated
 */
bool Leg::stepUpdate() {
  // TODO check if foot air time has changed, may need to change speed of leg
  if (state_ == State::ON_GROUND) {
    return false;
  }

  // if leg has only just transitioned to RAISED then need to calculate trajectory
  if (target_updated_ && prev_state_ == State::ON_GROUND) {
    step_no_ = 0;                                 // reset
    current_foot_air_time_ = foot_air_time_new_;  // save for later when doing actual movement
    bool result_target = calculateJointAngles(target_pos_, joint_targets_, IKMode::WALK);
    if (!result_target) {
      std::cout << "Could not calculate joint angles at requested foot target position\n";
      return false;
    }

    bool result = calculateJointAngles(raised_pos_, joint_targets_raised_, IKMode::WALK);
    if (!result) {
      // TODO in future can try reducing the raise amount until this works
      std::cout << "Could not calculate joint angles at requested raise limit position\n";
      return false;
    }
    calculateTrajectory();
  }

  // recalculate target and trajectory
  if (target_updated_ && step_no_ > 0) {
    JointAngles target_joints_upd;
    bool result_upd_target = calculateJointAngles(target_pos_, target_joints_upd, IKMode::WALK);
    if (!result_upd_target) {
      std::cout << "Could not calculate joint angles at updated foot target position\n";
    } else {
      joint_targets_ = target_joints_upd;
      calculateTrajectory();
    }
  }

  incrementLeg();
  target_updated_ = false;
  step_no_++;
  return true;
}

/**
 * @details
 * Will change status from raised to on ground if it has reached the end of its raised trajectory.
 * 
 * Will change status from on groun to raised if a raise is requested by input param, and it's not already at its target position.
 *  If it is already at its target it will not raise even if requested.
 * 
 * @param raise indicates if the leg should raise if possible
 * @return true if the leg changes to raised, or skipped raising
 */
bool Leg::updateStatus(const bool raise) {
  prev_state_ = state_;
  bool result = false;
  if (state_ == State::RAISED && step_no_ == current_foot_air_time_) {
    state_ = State::ON_GROUND;
  }
  // If it's already at its target then don't lift but return true as if it had
  else if (raise && state_ == State::ON_GROUND) {
    float dx = pos_.x() - target_pos_.x();
    float dy = pos_.y() - target_pos_.y();
    float dz = pos_.z() - target_pos_.z();
    float d =
        sqrtf(dx * dx + dy * dy + dz * dz);  // TODO skip sqrt and compare to tolerance squared
    if (!compareFloat(d, 0.0f, target_tolerance)) {
      state_ = State::RAISED;
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
                        size_t foot_air_time) {
  target_pos_ = target_pos;
  raised_pos_ = raised_pos;
  foot_air_time_new_ = foot_air_time;
  target_updated_ = true;
}

Leg::JointAngles Leg::getJointAngles() const {
  return JointAngles{joints_[JOINT_1].angle_, joints_[JOINT_2].angle_, joints_[JOINT_3].angle_};
}

bool Leg::setStartingAngles(Leg::JointAngles starting_angles) {
  bool result = setJointAngles(starting_angles);
  if (result) {
    target_pos_ = pos_;
  }
  return result;
}