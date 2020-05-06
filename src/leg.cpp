#include "leg.h"

#include "kinematics_support.h"
#include "transformations.h"

#include <stddef.h>
#include <cmath>
#include <iostream>

using namespace KinematicsSupport;
using namespace Transformations;

Joint::Joint() : offset_(0.0f), lower_limit_(-1.48f), upper_limit_(1.48f), angle_(0.0f) {}

Joint::Joint(float offset, float lower_limit, float upper_limit)
    : offset_(offset), lower_limit_(lower_limit), upper_limit_(upper_limit), angle_(0.0f) {}

Joint::Joint(float offset, float lower_limit, float upper_limit, float angle)
    : offset_(offset), lower_limit_(lower_limit), upper_limit_(upper_limit), angle_(angle) {}

bool Joint::isWithinLimits(const float angle) const {
  return (angle >= lower_limit_ - KinematicsSupport::eps) &&
         (angle < upper_limit_ + KinematicsSupport::eps);
}

float Joint::clampToLimts(float angle) const {
  return fmax(fmin(angle, upper_limit_), lower_limit_);
}

Leg::Leg() : Leg(JointAngles{0,0,0}){}

Leg::Leg(JointAngles joint_angles) {
  // BIT OF A PLACEHOLDER
  // until I decide how to initialise everything properly
  joints_[JOINT_1].offset_ = 0;
  joints_[JOINT_1].lower_limit_ = -90.0f * M_PI / 180.0;
  joints_[JOINT_1].upper_limit_ = 90.0f * M_PI / 180.0;
  joints_[JOINT_1].angle_ = 0.0f;

  joints_[JOINT_2].offset_ = M_PI / 4.0f;
  joints_[JOINT_2].lower_limit_ = -150.0f * M_PI / 180.0;
  joints_[JOINT_2].upper_limit_ = 150.0f * M_PI / 180.0;
  joints_[JOINT_2].angle_ = 0.0f;

  joints_[JOINT_3].offset_ = -3.0f * M_PI / 4.0f;
  joints_[JOINT_3].lower_limit_ = -150.0f * M_PI / 180.0;
  joints_[JOINT_3].upper_limit_ = 150.0f * M_PI / 180.0;
  joints_[JOINT_3].angle_ = 0.0f;

  setJointAngles(joint_angles);
}

// checks that a given set of joint angles do lead to a specific foot position
bool Leg::validateJointAngles(const JointAngles& angles, const Vector3& pos) {
  Vector3 pos_from_ik;
  calculateFootPosition(angles, pos_from_ik);
  return comparePositions(pos, pos_from_ik);
}

// This function is quite messy
// Closed form IK
size_t Leg::calculateJointAngles(const Vector3& pos, JointAngles angles[2]) {
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
    ha = pos.x() / cos_theta_1 - a;
  } else {
    ha = pos.y() / sin_theta_1 - a;
  }
  float ka = (pos.z() * pos.z() + ha * ha - b * b - c * c) / (2 * b * c);

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
      float kb = b + c * cos(angles[i].theta_3);
      float kc = c * sin(angles[i].theta_3);
      angles[i].theta_2 = atan2(pos.z(), ha) - atan2(kc, kb);
      angles[i].theta_2 = wrapAngle(angles[i].theta_2);
      if (joints_[JOINT_2].isWithinLimits(angles[i].theta_2)) {
        angles[i].theta_2 = joints_[JOINT_3].clampToLimts(angles[i].theta_2);
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

bool Leg::jointsWithinLimits(const JointAngles& joint_angles) const {
  return (joints_[JOINT_1].isWithinLimits(joint_angles.theta_1) &&
          joints_[JOINT_2].isWithinLimits(joint_angles.theta_2) &&
          joints_[JOINT_3].isWithinLimits(joint_angles.theta_3));
}

bool Leg::calculateFootPosition(const JointAngles& angles, Vector3& pos) {
  // TODO add joint limit check
  float h = a + b * cos(angles.theta_2) + c * cos(angles.theta_2 + angles.theta_3);
  pos.x() = h * cos(angles.theta_1);
  pos.y() = h * sin(angles.theta_1);
  pos.z() = b * sin(angles.theta_2) + c * sin(angles.theta_2 + angles.theta_3);
  return true;
}

bool Leg::updateFootPosition() {
  return calculateFootPosition(getJointAngles(), pos_);
}

// TODO have any validity checks here
// or assume that they've already been done?
// should joint angles be stored here or within joint instances?
bool Leg::setJointAngles(const JointAngles& angles) {
  joints_[JOINT_1].angle_ = angles.theta_1;
  joints_[JOINT_2].angle_ = angles.theta_2;
  joints_[JOINT_3].angle_ = angles.theta_3;

  // updates foot position
  updateFootPosition();
  return true;
}

bool Leg::setFootPosition(const Vector3& pos) {
  JointAngles angles[2];
  size_t num_results = calculateJointAngles(pos, angles);
  if (num_results > 0) {
    size_t chosen_idx = chooseJointAngleOption(angles, num_results, getJointAngles());
    setJointAngles(angles[chosen_idx]);
    return true;
  } else {
    return false;
  }
}

bool Leg::testSetFootPosition(const Vector3& pos, JointAngles& ret_angles_if_valid) {
  JointAngles angles[2];
  size_t num_results = calculateJointAngles(pos, angles);
  if (num_results > 0) {
    size_t chosen_idx = chooseJointAngleOption(angles, num_results, getJointAngles());
    ret_angles_if_valid = angles[chosen_idx];
    return true;
  } else {
    return false;
  }
}

// return index of selected joint angles from angle_options
// compared to provided reference value
size_t Leg::chooseJointAngleOption(const JointAngles angle_options[2], size_t num_valid,
                                   const JointAngles& ref_angles) const {
  if (num_valid == 0) {
    return 0;
  } else if (num_valid == 1) {
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

// return the foot position in the leg frame
Vector3 Leg::getFootPosition() const { return pos_; }

// Must only be called if state == RAISED
bool Leg::stepUpdate() {
  // TODO check if foot air time has changed, may need to change speed of leg


  JointAngles current_joint_angles = getJointAngles();
  // if leg has only just transitioned to RAISED then need to calculate trajectory
  if (target_updated_ && prev_state_ == State::ON_GROUND) {
    step_no_ = 0;                                 // reset
    current_foot_air_time_ = foot_air_time_new_;  // save for later when doing actual movement
    bool result_target = testSetFootPosition(target_pos_, joint_targets_);
    if (!result_target) {
      std::cout << "Could not calculate joint angles at requested foot target position\n";
      return false;
    }

    JointAngles raised_joint_limits;
    bool result = testSetFootPosition(raised_pos_, raised_joint_limits);
    if (!result) {
      // TODO in future can try reducing the raise amount until this works
      std::cout << "Could not calculate joint angles at requested raise limit position\n";
      return false;
    }
    // the foot must go up and then down
    joint_inc_up_.theta_1 =
        (joint_targets_.theta_1 - current_joint_angles.theta_1) / (float)current_foot_air_time_;
    joint_inc_up_.theta_2 = (raised_joint_limits.theta_2 - current_joint_angles.theta_2) /
                            ((float)current_foot_air_time_ / 2.0);
    joint_inc_up_.theta_3 = (raised_joint_limits.theta_3 - current_joint_angles.theta_3) /
                            ((float)current_foot_air_time_ / 2.0);
    joint_inc_down_.theta_2 = (joint_targets_.theta_2 - raised_joint_limits.theta_2) /
                              ((float)current_foot_air_time_ / 2.0);
    joint_inc_down_.theta_3 = (joint_targets_.theta_3 - raised_joint_limits.theta_3) /
                              ((float)current_foot_air_time_ / 2.0);
  }

  // recalculate target and trajectory
  if (target_updated_ && step_no_ > 0) {
    JointAngles target_joints_upd;
    bool result_upd_target = testSetFootPosition(target_pos_, target_joints_upd);
    if (!result_upd_target) {
      std::cout << "Could not calculate joint angles at updated foot target position\n";
    } else {
      joint_targets_ = target_joints_upd;
      // only update second half
      if (step_no_ >= (current_foot_air_time_ / 2) && step_no_ < current_foot_air_time_ - 1) {
        size_t steps_remaining = current_foot_air_time_ - 1 - step_no_;
        joint_inc_up_.theta_1 =
            (joint_targets_.theta_1 - current_joint_angles.theta_1) / (float)steps_remaining;
        joint_inc_down_.theta_2 =
            (joint_targets_.theta_2 - current_joint_angles.theta_2) / (float)steps_remaining;
        joint_inc_down_.theta_3 =
            (joint_targets_.theta_3 - current_joint_angles.theta_3) / (float)steps_remaining;
      }
    }
  }

  // to avoid rounding errors, when we get to the last step just set the angles exactly to the
  // target
  if (step_no_ == current_foot_air_time_ - 1) {
    current_joint_angles.theta_1 = joint_targets_.theta_1;
    current_joint_angles.theta_2 = joint_targets_.theta_2;
    current_joint_angles.theta_3 = joint_targets_.theta_3;
  } else if (step_no_ < (current_foot_air_time_ / 2)) {
    current_joint_angles.theta_1 += joint_inc_up_.theta_1;
    current_joint_angles.theta_2 += joint_inc_up_.theta_2;
    current_joint_angles.theta_3 += joint_inc_up_.theta_3;
  } else {
    current_joint_angles.theta_1 += joint_inc_up_.theta_1;
    current_joint_angles.theta_2 += joint_inc_down_.theta_2;
    current_joint_angles.theta_3 += joint_inc_down_.theta_3;
  }
  setJointAngles(current_joint_angles);
  target_updated_ = false;
  step_no_++;
  return true;
}

// 'raise' arg indicates if the leg should raise IF POSSIBLE i.e. not already raised or on ground at
// target position
// return value of true indicates raising (or skip raise)
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