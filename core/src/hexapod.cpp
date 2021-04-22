#include "hexapod.h"

#include "kinematics_support.h"
#include "leg.h"
#include "transformations.h"
#include "joint.h"
#include "build_hexapod.h"

#ifdef __AVR__
#include <Arduino.h>
#else
#include <cstddef>
#include <cmath>
#include <iostream>
#endif

namespace hexapod {

using namespace util;

/**
 * @details
 * The transforms and Legs provided MUST be in the following order:
 *
 * Starting at the front row/pair of legs...\n
 * Left leg, then right leg\n
 * Move back 1 row\n
 * Left leg, then right leg etc
 *
 * So the indicies for a 6 legged robot are:\n
 * Front left ->  0  1  <- Front right\n
 * Middle left -> 2  3  <- Middle right\n
 * Back left ->   4  5  <- Back right\n
 *
 *
 * @param num_legs - number of legs
 * @param hex_dims - hexapod body dimensions
 * @param tf_body_to_leg - array of transforms relating each leg to the body
 * @param legs - array of Legs
 */
Hexapod::Hexapod(const uint8_t num_legs, Dims hex_dims, Transform* tf_body_to_leg, Leg* legs)
    : dims_(hex_dims), num_legs_(num_legs), height_(hex_dims.depth / 2.0f) {
  tf_base_to_body_ = Transform();
  tf_base_to_body_prev_ = Transform();
  tf_base_movement_ = Transform();
  tf_base_to_new_base_ = Transform();
  tf_base_to_body_target_ = Transform();

  legs_ = legs;
  tf_body_to_leg_ = tf_body_to_leg;

  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    tf_body_to_leg_[leg_idx] = tf_body_to_leg[leg_idx];
    legs_[leg_idx] = legs[leg_idx];
  }


  // set various movement parameters based on body/leg dimensions
  walk_height_default_ = legs_[0].dims_.c / 2.0;
  stance_width_default_ = (legs_[0].dims_.a + legs_[0].dims_.b + legs_[0].dims_.c) * 0.5f;
  stance_width_min_ = (legs_[0].dims_.a + legs_[0].dims_.b + legs_[0].dims_.c) * 0.25f;
  stance_width_max_ = (legs_[0].dims_.a + legs_[0].dims_.b + legs_[0].dims_.c) * 0.75f;
  leg_lift_height_min_ = walk_height_default_ * 0.1f;
  leg_lift_height_max_ = walk_height_default_;
  leg_lift_height_default_ = walk_height_default_ * 0.3f;
  allowed_foot_position_diameter_ = (legs_[0].dims_.a + legs_[0].dims_.b + legs_[0].dims_.c) / 3.0;

  stance_width_ = stance_width_default_;
  leg_lift_height_ = leg_lift_height_default_;

  rising_increment_ = (walk_height_default_-height_) / 20.0f; // TODO ideally hexapod model should have an idea of what frequency it will be updated

}

Hexapod::~Hexapod() {
  delete[] legs_;
  delete[] tf_body_to_leg_;
}

/**
 * @details Expected to be used only at start up when robot is in UNSUPPORTED state
 *
 * @param starting_angles array of all angles in standard library convention order
 * @return true if joint angles set successfully
 */
bool Hexapod::setStartingAngles(const Leg::JointAngles starting_angles[]) {
  if (state_ != State::UNSUPPORTED) {
    return false;
  }

  bool result = true;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    result &= legs_[leg_idx].jointsWithinLimits(starting_angles[leg_idx]);
  }

  if (result) {
    for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
      legs_[leg_idx].setStartingAngles(starting_angles[leg_idx]);
    }
  }

  return result;
}


bool Hexapod::setStartingAngles(const Leg::JointAngles& starting_angles) {
  if (state_ != State::UNSUPPORTED) {
    return false;
  }

  Leg::JointAngles starting_angles_all[num_legs_];
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    starting_angles_all[leg_idx] = starting_angles;
  }
  return setStartingAngles(starting_angles_all);
}

bool Hexapod::setStartingAnglesPhysical(const Leg::JointAngles starting_angles_physical[]) {
  Leg::JointAngles starting_angles_model[num_legs_];
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    starting_angles_model[leg_idx] = legs_[leg_idx].fromPhysicalAngles(starting_angles_physical[leg_idx]);
  }
  return setStartingAngles(starting_angles_model);
}

//bool Hexapod::setStartingAnglesPhysical(const Leg::JointAngles& starting_angles_physical) {
//  Leg::JointAngles starting_angles_model[num_legs_];
//  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
//    starting_angles_model[leg_idx] = legs_[leg_idx].fromPhysicalAngles(starting_angles_physical);
//  }
//  return setStartingAngles(starting_angles_model);
//}

bool Hexapod::setLegJoints(const uint8_t leg_idx, const Leg::JointAngles& joint_angles) {
  if (legs_[leg_idx].jointsWithinLimits(joint_angles)) {
    legs_[leg_idx].setJointAngles(joint_angles);
    return true;
  }
  return false;
}

bool Hexapod::setLegJointsPhysical(const uint8_t leg_idx, const Leg::JointAngles& physical_joint_angles) {
  const Leg::JointAngles model_joint_angles = legs_[leg_idx].fromPhysicalAngles(physical_joint_angles);
  return setLegJoints(leg_idx, model_joint_angles);
}

/**
 * @details
 * @return true if an IK solution is found for all legs
 */
bool Hexapod::calculateGroundedLegs() {
  const Transform tf_base_to_new_body = tf_base_to_new_base_ * tf_base_to_body_target_;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (legs_[leg_idx].state_ == Leg::State::ON_GROUND) {
      // current foot position in base frame - this is not going to change
      const Vector3 foot_in_base =
          tf_base_to_body_ * tf_body_to_leg_[leg_idx] * legs_[leg_idx].getFootPosition();
      // position of foot in the updated leg frame (base on walk and body movement)
      const Vector3 leg_to_foot_new =
          (tf_base_to_new_body * tf_body_to_leg_[leg_idx]).inverse() * foot_in_base;
      const bool result = legs_[leg_idx].calculateJointAngles(leg_to_foot_new, Leg::IKMode::WALK);
      if (!result) {
#ifndef __AVR__
        std::cout << "Unable to find IK solution for all legs.\n";
#endif
        return false;
      }
    }
  }
  return true;
}

void Hexapod::applyChangesGroundedLegs() {
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (legs_[leg_idx].state_ == Leg::State::ON_GROUND) {
      legs_[leg_idx].applyStagedAngles();
    }
  }
}

uint8_t Hexapod::getNumLegsRaised() const {
  uint8_t num_legs_raised = 0;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (legs_[leg_idx].state_ == Leg::State::RAISED) {
      num_legs_raised++;
    }
  }
  return num_legs_raised;
}

/**
 * @details
 * Calls Leg::updateStatus() for all legs.
 *
 * Determines whether the robot wants the leg to raise based on other conditions like the current
 * position in the gait sequence
 * and the number of legs currently raised.
 *
 * If there's no base change flag (includes zero speed) then don't request raise
 *
 */
void Hexapod::updateLegsStatus() {
  uint8_t num_legs_raised = getNumLegsRaised();
  bool raise_result = false;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    // check if this leg is next to be lifted, and if we want to allow it
    if (base_change_ && legs_[leg_idx].state_ == Leg::State::ON_GROUND &&
        gaitNextLeg() == leg_idx && num_legs_raised < gaitMaxRaised()) {
      updateFootTarget(leg_idx); // TODO I think this can (and should) be removed because it's already called for
                                  // all legs in updateFootTargets called in update() (unless there's anything
                                  // significant happening inbetween but I don't think so
      raise_result = legs_[leg_idx].updateStatus(true);
    } else {
      legs_[leg_idx].updateStatus(false);  // ignore return result (will be false)
    }
  }

  if (raise_result) {
    advanceGait();
  }
}

/**
 * @details
 * Calculate 'step' vector induced by change in base at each foot neutral point.\n
 * Current point = foot neutral point in current leg base frame\n
 * New point = foot neutral point in new leg base frame\n
 * Neutral point is fixed wrt base (so base->body change does not affect it)\n
 *
 * @param leg_idx - calculate for which leg
 * @return Vector3 - step vector
 */
Vector3 Hexapod::calculateFootVector(const uint8_t leg_idx) const {
  const Vector3 base_to_neutral = tf_body_to_leg_[leg_idx] * getNeutralPosition(leg_idx);
  const Vector3 base_to_neutral_new = tf_base_to_new_base_ * base_to_neutral;
  const Vector3 step = base_to_neutral_new - base_to_neutral;
  return step;
}

Vector3 Hexapod::legToBase(const uint8_t leg_idx, const Vector3& v) const {
  return tf_base_to_body_ * tf_body_to_leg_[leg_idx] * v;
}

/**
 * @details
 * The leg itself doesn't know how high the robot is, so leg::getNeutralPosition just has z = 0
 * so we just add it
 *
 * The neutral position does not change with the body, so we only need to apply the body_to_leg
 * transform
 * to get the neutral position in the base frame i.e. as if the base to body transform is identity
 *
 * @param leg_idx
 * @return Vector3
 */
Vector3 Hexapod::getNeutralPosition(const uint8_t leg_idx) const {
  Vector3 leg_neutral = legs_[leg_idx].getNeutralPosition();
  leg_neutral.z() = -height_;
  return tf_body_to_leg_[leg_idx] * leg_neutral;
}

/**
 * @details
 * Calculate the required angles for all legs on the ground to accomodate the desired walk/turn.\n
 * Apply them if possible, and if so then update the current state to incorporate the latest base
 * and body movements.
 *
 * @return true if an IK solution was found for all grounded legs
 */
bool Hexapod::handleGroundedLegs() {
  // calculate the required angles for all legs on the ground to accomodate the desired walk/turn
  // movement plus any body offset/rotation
  bool ik_result = calculateGroundedLegs();
  if (ik_result) {
    applyChangesGroundedLegs();
    walk_step_current_ = walk_step_new_;
    turn_step_current_ = turn_step_new_;
    if (move_mode_ == MoveMode::HEADLESS) {
      total_base_rotation_ += turn_step_new_;
    }
    if (base_change_) {
      tf_base_movement_ = tf_base_to_new_base_;
      height_ += tf_base_to_new_base_.t_.z();
    }
    if (body_change_) {
      tf_base_to_body_prev_ = tf_base_to_body_;
      tf_base_to_body_ = tf_base_to_body_target_;
    }
  } else {
    tf_base_to_body_target_ = tf_base_to_body_;  // use current body position in further
                                                 // calculations since we haven't moved it
#ifndef __AVR__
    std::cout << "Unable to find IK solution for all grounded legs.\n";
#endif
  }
  return ik_result;
}

/**
 * @details
 * Update the leg raise time, apex position and target position of a step.\n
 * The time is based upon the magnitude of the step vector.\n
 * The target is based upon the direction of the step vector.\n
 * The apex is a point above the neutral position.
 *
 * @param leg_idx - calculate for which leg
 */
void Hexapod::updateFootTarget(const uint8_t leg_idx) {
  const Vector3 combined_step = calculateFootVector(leg_idx);
  uint16_t foot_air_time;  // how long we want the foot in the air for

  float speed = combined_step.norm();
  Vector3 step_unit;
  if (compareFloat(speed, 0.0f, 0.0001f)) {
    step_unit = Vector3(0.0f, 0.0f, 0.0f);  // not actually a unit vector obviously
    speed = 0.0f;
    foot_air_time = foot_air_time_default_;
  } else {
    step_unit = combined_step.unit();

    // Check desired speed within limits and cap it if not
    uint16_t min_foot_ground_time = foot_air_time_min_ * (num_legs_ - gaitMaxRaised());
    // max speed if travel full allowed distance in the minimum allowed time
    const float max_v = allowed_foot_position_diameter_ / static_cast<float>(min_foot_ground_time);
    if (speed > max_v) {
#ifndef __AVR__
      std::cout << "Speed over limit. Capped to " << max_v << '\n';
#endif
      speed = max_v;
    }

    // Convert speed to number of time steps that foot will be on the ground
    const float foot_ground_distance =
        allowed_foot_position_diameter_ * foot_ground_travel_ratio_;  // stride length
    const float foot_ground_time_fl = foot_ground_distance / speed;
    // and in the air - make sure it's even (round up if not)
    const float foot_air_time_fl = foot_ground_time_fl / static_cast<float>(num_legs_ - gaitMaxRaised());
    foot_air_time = (uint16_t)ceilf(foot_air_time_fl);
    if (foot_air_time % 2 == 1) {
      foot_air_time += 1;
    }
  }

  Vector3 raised_pos;
  Vector3 target_pos;
  // if the leg has already been lifted, use the already calculated targets
  //    we just need to modify them to account to changes in body position
  if (legs_[leg_idx].state_ == Leg::State::RAISED && legs_[leg_idx].getStepIdx() > 0) {
    // perhaps move this calculation 'higher' to avoid repeating (although will only repeat for
    // raised legs)
    const Transform tf_update = (tf_base_to_body_ * tf_body_to_leg_[leg_idx]).inverse() *
                          tf_base_to_body_prev_ * tf_body_to_leg_[leg_idx];
    target_pos = tf_update * legs_[leg_idx].getTargetPosition();
    raised_pos = tf_update * legs_[leg_idx].getRaisedPosition();
    // also need to update the current position to account for the body change
    // I think I need to re-think the whole leg raise movement approach really
    // because it could probably be a lot simpler
    const Vector3 upd_current_pos = tf_update * legs_[leg_idx].getFootPosition();
    legs_[leg_idx].calculateJointAngles(upd_current_pos, Leg::IKMode::WALK);
    if (legs_[leg_idx].calculateJointAngles(upd_current_pos, Leg::IKMode::WALK)) {
      legs_[leg_idx].applyStagedAngles();
    }
  }
  // if the leg is only just about to become raised then need to calculate targets for first time
  // also do this if the foot is on the ground i.e. target update doesn't relate to a step
  //  (this allows the leg to respond to changes in e.g. stance width)
  else {
    const Vector3 neutral_pos = getNeutralPosition(leg_idx);
    Vector3 raised_pos_in_base = neutral_pos;
    raised_pos_in_base.z() += leg_lift_height_;

    // HACK add 2 (times num feet on ground) to account for a few steps overhead in changing state
    // that means feet are actually on the ground longer than calculated
    const uint16_t foot_ground_time = 2 + foot_air_time * (num_legs_ - gaitMaxRaised());
    const float half_distance_to_travel = (static_cast<float>(foot_ground_time) * speed) / 2.0f;
    const Vector3 target_pos_in_base = neutral_pos + half_distance_to_travel * step_unit;
    // transform the step vector from base frame to leg base
    // can ignore the new base to base stuff since we only care about the relative position to the
    // base, wherever it is
    const Transform tf_leg_to_base = (tf_base_to_body_ * tf_body_to_leg_[leg_idx]).inverse();
    raised_pos = tf_leg_to_base * raised_pos_in_base;
    target_pos = tf_leg_to_base * target_pos_in_base;
  }

  legs_[leg_idx].updateTargets(target_pos, raised_pos, foot_air_time);
}

void Hexapod::updateFootTargets() {
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (recalculate_all_feet_targets_ ||
        (recalculate_raised_feet_targets_ && legs_[leg_idx].state_ == Leg::State::RAISED)) {
      updateFootTarget(leg_idx);
    }
  }
}

Vector3 Hexapod::getFootPosition(const uint8_t leg_idx) const {
  return legToBase(leg_idx, legs_[leg_idx].getFootPosition());
}

Vector3 Hexapod::getTargetPosition(const uint8_t leg_idx) const {
  return legToBase(leg_idx, legs_[leg_idx].getTargetPosition());
}

Vector3 Hexapod::getRaisedPosition(const uint8_t leg_idx) const {
  return legToBase(leg_idx, legs_[leg_idx].getRaisedPosition());
}

void Hexapod::handleRaisedLegs() {
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (legs_[leg_idx].state_ == Leg::State::RAISED) {
      legs_[leg_idx].stepUpdate();
    }
  }
}

bool Hexapod::setWalk(const Vector3& walk_step, const float angle_step) {
  if (!(state_ == State::WALKING)) {
    return false;
  }

  if (move_mode_ == MoveMode::HEADLESS) {
    float x =
        cos(-total_base_rotation_) * walk_step.x() - sin(-total_base_rotation_) * walk_step.y();
    float y =
        sin(-total_base_rotation_) * walk_step.x() + cos(-total_base_rotation_) * walk_step.y();
    walk_step_new_ = Vector3(x, y, 0);
  } else {
    walk_step_new_ = walk_step;
  }
  turn_step_new_ = angle_step;
  tf_base_to_new_base_.R_.setRPYExtr(0.0f, 0.0f, turn_step_new_);
  tf_base_to_new_base_.t_ = walk_step_new_;
  base_change_ = true;
  // If there's a change, need to update raised feet target, unless now stopped in which case update
  // all to allow feet to return to neutral position
  // Current Vector3 comparison allows some tolerance, here explicity test if zero
  if (walk_step_current_ != walk_step_new_ || turn_step_current_ != turn_step_new_) {
    if (walk_step_new_ == Vector3{0.0f, 0.0f, 0.0f} && turn_step_new_ == 0.0f) {
      recalculate_all_feet_targets_ = true;
    } else {
      recalculate_raised_feet_targets_ = true;
    }
  }
  return true;
}

bool Hexapod::setWalk(const Vector3& walk_step) { return setWalk(walk_step, 0.0f); }

bool Hexapod::setWalk(const float angle_step) { return setWalk(Vector3(0.0f, 0.0f, 0.0f), angle_step); }

bool Hexapod::setBody(const Transform& tf_base_to_body_target) {
  if (!(state_ == State::WALKING)) {
    return false;
  }
  tf_base_to_body_target_ = tf_base_to_body_target;
  body_change_ = true;
  recalculate_raised_feet_targets_ = true;
  return true;
}

bool Hexapod::changeBody(const Transform& tf_base_to_body_change) {
  Transform tf_base_to_body_target;
  if (move_mode_ == MoveMode::HEADLESS) {
    Transform headless_correction;
    headless_correction.R_.setRPYExtr(0, 0, -total_base_rotation_);
    tf_base_to_body_target = headless_correction.inverse() * tf_base_to_body_change *
                             headless_correction * tf_base_to_body_;
  } else {
    tf_base_to_body_target = tf_base_to_body_change * tf_base_to_body_;
  }
  return setBody(tf_base_to_body_target);
}

bool Hexapod::clearMovement() { return setWalk(Vector3(0.0f, 0.0f, 0.0f), 0.0f); }

/**
 * @details
 * The hexapod will not move until another movement type command (e.g. setWalk, changeBody) is given
 *
 */
void Hexapod::clearTargets() {
  base_change_ = false;
  body_change_ = false;
  tf_base_to_body_target_ = tf_base_to_body_;
  tf_base_to_new_base_ = Transform();
  recalculate_raised_feet_targets_ = false;
  recalculate_all_feet_targets_ = false;
}

void Hexapod::clearVisualisationChanges() { tf_base_movement_ = Transform(); }

/**
 * @details
 * clearVisualisationChanges must come at the beginning because those changes need to be available
 * after update() is called
 *  and before the next update() is called.
 *
 * TODO - move updateFootTargets and updateLegsStatus into handleRaisedLegs?
 *
 * @return true always - this is now redundant
 */
bool Hexapod::update() {
  clearVisualisationChanges();

  if (state_ == State::UNSUPPORTED) {
    updateMoveLegs();
  } else if (state_ == State::STANDING) {
    handleGroundedLegs();
  } else if (state_ == State::WALKING) {
    handleGroundedLegs();
    updateFootTargets();  // Update foot targets if required for other non-raising legs
    handleRaisedLegs();   // We can keep moving the raised legs even if we couldn't move the ones on
                          // the ground
    updateLegsStatus();  // Allow them (based on conditions) to change state between ON_GROUND and RAISED
  } else {
    // state_ == State::FULL_MANUAL
    // The legs have already been modified directly though the manualMoveFoot and manualChangeJoint functions
  }

  clearTargets();
  handleStateChange();
  return true;
}

/**
 * @details
 * TODO the foot movement limits should (in theory) also be updated after this gets called
 *
 * @param stance_width
 * @return true if the value was changed
 */
bool Hexapod::setStanceWidth(float stance_width) {
  if (stance_width < stance_width_min_) {
    stance_width = stance_width_min_;
  } else if (stance_width > stance_width_max_) {
    stance_width = stance_width_max_;
  }

  if (stance_width_ == stance_width) {
    return false;
  } else {
    stance_width_ = stance_width;
    recalculate_all_feet_targets_ = true;
    for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
      legs_[leg_idx].getNeutralPosition().x() = stance_width_;
    }
    return true;
  }
}

/**
  * @details
  *
  * @param change
 * @return true if the value was changed
 */
bool Hexapod::changeStanceWidth(const float change) {
  const float new_stance_width = stance_width_ + change;
  return setStanceWidth(new_stance_width);
}

/**
 * @details
 *
 * @return true if the value was changed
 */
bool Hexapod::resetStanceWidth() { return setStanceWidth(stance_width_default_); }

/**
 * @details
 * TODO need to add quite a few checks here to make sure this is possible without exceeding limits
 *
 * @param gait
 * @return true if the gait was changed
 */
bool Hexapod::changeGait(const Gait gait) {
  if (gait < Gait::NUM_GAITS) {
    current_gait_seq_ = gait;
  }
  return true;
}

/**
 * @details
 *
 * @param ratio
 * @return true if the value was changed
 */
bool Hexapod::setFootGroundTravelRatio(float ratio) {
  if (ratio < fgtr_min_)
    ratio = fgtr_min_;
  else if (ratio > fgtr_max_)
    ratio = fgtr_max_;
  if (foot_ground_travel_ratio_ == ratio) {
    return false;
  } else {
    foot_ground_travel_ratio_ = ratio;
    return true;
  }
}

/**
 * @details
 *
 * @param change
 * @return true if the value was changed
 */
bool Hexapod::changeFootGroundTravelRatio(const float change) {
  float new_ratio = foot_ground_travel_ratio_ + change;
  return setFootGroundTravelRatio(new_ratio);
}

/**
 * @details
 *
 * @return true if the value was changed
 */
bool Hexapod::resetFootGroundTravelRatio() { return setFootGroundTravelRatio(fgtr_default_); }

/**
 * @details
 *
 * @param height
 * @return true if the value was changed
 */
bool Hexapod::setLegRaiseHeight(float height) {
  if (height < leg_lift_height_min_)
    height = leg_lift_height_min_;
  else if (height > leg_lift_height_max_)
    height = leg_lift_height_max_;
  if (leg_lift_height_ == height) {
    return false;
  } else {
    leg_lift_height_ = height;
    return true;
  }
}

/**
 * @details
 *
 * @param change
 * @return true if the value was changed
 */
bool Hexapod::changeLegRaiseHeight(const float change) {
  const float new_height = leg_lift_height_ + change;
  return setLegRaiseHeight(new_height);
}

/**
 * @details
 *
 * @return true if the value was changed
 */
bool Hexapod::resetLegRaiseHeight() { return setLegRaiseHeight(leg_lift_height_default_); }

/**
 * @details
 * The fixed heading of headless mode will always be set to the current heading when this is called
 *  to enter headless mode, whether or not it is already in headless mode
 *
 * @param move_mode
 */
void Hexapod::setMoveMode(const MoveMode move_mode) {
  move_mode_ = move_mode;
  if (move_mode_ == MoveMode::HEADLESS) {
    total_base_rotation_ = 0.0f;
  }
}

/**
 * @details
 * One of several basic functions for getting the hexapod to move from a starting position to an
 * upright position supported by the legs from which it can start walking.
 *
 * @param joint_targets
 * @return true if the requested targets were set
 */
bool Hexapod::setLegTargets(const Leg::JointAngles joint_targets[], const uint16_t duration) {
  if (state_ != State::UNSUPPORTED) {
    return false;
  }

  // check that targets are achievable
  bool joint_check_result = true;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    joint_check_result &= legs_[leg_idx].jointsWithinLimits(joint_targets[leg_idx]);
  }
  if (!joint_check_result) {
#ifdef __AVR__
    Serial.print(F("Requested movement not achievable\n"));
#else
    std::cout << "Requested movement not achievable\n";
#endif
    return false;
  }

  // calculate trajectory
  // if start and end angles were ok, then everything in between should be too
  // since we never try and go the 'shorter' way around i.e. don't cross -180/+180 boundary
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    const Leg::JointAngles& leg_joint_targets = joint_targets[leg_idx];
    const Leg::JointAngles current_angles = legs_[leg_idx].getJointAngles();
    const Leg::JointAngles angle_range{leg_joint_targets.theta_1 - current_angles.theta_1,
                                 leg_joint_targets.theta_2 - current_angles.theta_2,
                                 leg_joint_targets.theta_3 - current_angles.theta_3};
    const Leg::JointAngles joint_increments{angle_range.theta_1 / static_cast<float>(duration),
                                      angle_range.theta_2 / static_cast<float>(duration),
                                      angle_range.theta_3 / static_cast<float>(duration)};
    // To fit existing setup will need to calculate midpoint although not strictly required
    const Leg::JointAngles
        midpoint{current_angles.theta_1 + joint_increments.theta_1 * static_cast<float>(duration/2),
                 current_angles.theta_2 + joint_increments.theta_2 * static_cast<float>(duration/2),
                 current_angles.theta_3 + joint_increments.theta_3 * static_cast<float>(duration/2)};

    legs_[leg_idx].setTrajectory(leg_joint_targets,
                                 joint_increments,
                                 midpoint,
                                 joint_increments,
                                 duration);
  }
  return true;
}

/**
 * @details Call setLegTargets with the same joint angles for every leg.
 * @param joint_targets
 * @return true if the requested targets were set
 */
bool Hexapod::setLegTargets(const Leg::JointAngles& joint_targets, const uint16_t duration) {
  if (state_ != State::UNSUPPORTED) {
    return false;
  }

  Leg::JointAngles joint_targets_all[num_legs_];
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    joint_targets_all[leg_idx] = joint_targets;
  }

  return setLegTargets(joint_targets_all, duration);
}




/**
 * @details
 * One of several basic functions for getting the hexapod to move from a starting position to an
 * upright position supported by the legs from which it can start walking.
 *
 * @return true if all legs have completed their trajectories
 */
bool Hexapod::updateMoveLegs() {
  bool finished = true;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    legs_[leg_idx].incrementLeg();
    finished &= (legs_[leg_idx].getStepIdx() >= legs_[leg_idx].getCurrentStepDuration());
  }
  return finished;
}

/**
 * @details
 * One of several basic functions for getting the hexapod to move from a starting position to an
 * upright position supported by the legs from which it can start walking.
 * Basically facilitates the transition from STANDING state to WALKING state
 *
 * @return true
 * @return false
 */
bool Hexapod::riseToWalk() {
  if (state_ == State::STANDING && height_ < walk_height_default_) {
    changeBase(Vector3(0, 0, rising_increment_));
    requested_state_ = State::WALKING;
    return true;
  } else {
    return false;
  }
}

/**
 * @details
 * One of several basic functions for getting the hexapod to move from a starting position to an
 * upright position supported by the legs from which it can start walking.
 *
 * TODO - maybe refactor.
 *
 * @param move_base
 * @return true always
 */
bool Hexapod::changeBase(const Vector3& move_base) {
  tf_base_to_new_base_.t_ = move_base;
  base_change_ = true;
  return true;
}

/**
 * @details
 * One of several basic functions for getting the hexapod to move from a starting position to an
 * upright position supported by the legs from which it can start walking.
 *
 * @return true if an IK solution was found for all legs
 */
bool Hexapod::setLegTargetsToGround(const uint16_t duration) {
  if (state_ != State::UNSUPPORTED) {
    return false;
  }

  bool ik_result = true;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    // some default position
    Vector3 grounded_position = legs_[leg_idx].getNeutralPosition();
    grounded_position.z() = -height_;
    ik_result &= legs_[leg_idx].calculateJointAngles(grounded_position, Leg::IKMode::WALK);
  }
  if (ik_result) {
    bool set_target_result = true; // unnecessary?
    for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
      set_target_result &= setLegTargets(legs_[0].getStagedAngles(), duration);
    }
    if (set_target_result) {
      requested_state_ = State::STANDING;
      return true;
    }
  }

  return false;
}

void Hexapod::handleStateChange() {
  if (state_ == State::UNSUPPORTED && requested_state_ == State::STANDING) {
    // check all legs on floor
    bool ready = true;
    for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
      // get position in base frame
      const float leg_height = getFootPosition(leg_idx).z();
      const float floor_height = -height_;
      // TODO review hardcoded value?
      if (!compareFloat(leg_height, floor_height, 0.0001f)) {
        ready = false;
      }
    }
    if (ready) {
      state_ = requested_state_;
#ifdef __AVR__
      Serial.print(F("State changed to: STANDING\n"));
#else
      std::cout << "State changed to: STANDING\n";
#endif
    }
  }

  // go from standing to walking if base it at some predefined position
  // hexapod doesn't actually know base position except the height
  if (state_ == State::STANDING && requested_state_ == State::WALKING &&
      height_ >= walk_height_default_) {
    state_ = requested_state_;
#ifdef __AVR__
    Serial.print(F("State changed to: WALKING\n"));
#else
    std::cout << "State changed to: WALKING\n";
#endif
  }

  // TODO add some conditions for going back
  // will need to ensure that legs are allowed to finish current step
  if (state_ == State::WALKING && requested_state_ == State::STANDING && false) {
    state_ = requested_state_;
#ifndef __AVR__
    std::cout << "State changed to: STANDING\n";
#endif
  }

  // TODO add some conditions for going back
  if (state_ == State::STANDING && requested_state_ == State::UNSUPPORTED && false) {
    state_ = requested_state_;
#ifndef __AVR__
    std::cout << "State changed to: UNSUPPORTED\n";
#endif
  }

  // immediately transition - stop all other movement
  // note that all other state changes so far were triggered from internal changes - this one will be external
  if (state_ != State::FULL_MANUAL && requested_state_ == State::FULL_MANUAL) {
    state_ = State::FULL_MANUAL;
  }
  // TODO need sensible way to go back
}

Hexapod::State Hexapod::getState() const { return state_; }

const Leg& Hexapod::getLeg(const uint8_t leg_idx) const { return legs_[leg_idx]; }

const Transform& Hexapod::getBaseToBody() const { return tf_base_to_body_; }

const Transform& Hexapod::getBaseMovement() const { return tf_base_movement_; }

float Hexapod::getHeight() const { return height_; }

void Hexapod::setFullManualControl(const bool control_on) {
  if (state_ != State::FULL_MANUAL && control_on) {
    requested_state_ = State::FULL_MANUAL;
    setManualLegControl(0); // default
    manual_leg_idx_ = 0;
    manual_joint_idx_ = 0;
  }
  if (state_ == State::FULL_MANUAL && !control_on) {
    // always return from MANUAL back into UNSUPPORTED // TODO review this later
    requested_state_ = State::UNSUPPORTED;
  }
}

void Hexapod::setManualLegControl() {
  manual_control_type_ = ManualControlType::ALL_LEGS;
  // TODO not yet implemented
}

void Hexapod::setManualLegControl(const uint8_t leg_idx) {
  manual_control_type_ = ManualControlType::SINGLE_LEG;
  manual_leg_idx_ = leg_idx < num_legs_ ? leg_idx : 0;
}

void Hexapod::setManualJointControl(const uint8_t joint_idx) {
  manual_control_type_ = ManualControlType::SINGLE_JOINT;
  manual_joint_idx_ = joint_idx < Leg::NUM_JOINTS ? joint_idx : 0;
}

void Hexapod::manualMoveFoot(const Vector3& movement) {
  if (state_ == State::FULL_MANUAL && manual_control_type_== ManualControlType::SINGLE_LEG) {
    const Vector3 new_pos = legs_[manual_leg_idx_].getFootPosition() + movement;
    const bool ik_result = legs_[manual_leg_idx_].calculateJointAngles(new_pos, Leg::IKMode::FULL);
    if (ik_result) {
      legs_[manual_leg_idx_].applyStagedAngles();
    }
  }
}

void Hexapod::manualChangeJoint(const float angle_change) {
  if (state_ == State::FULL_MANUAL && manual_control_type_== ManualControlType::SINGLE_JOINT) {
    // TODO this is very awkward/awful! Maybe some refactoring required (make JointAngles an indexable array)
    const Joint joint = legs_[manual_leg_idx_].joints_[manual_joint_idx_];
    const float new_angle = joint.clampToLimts(joint.angle_ + angle_change);
    // can't set it directly, need to do so via leg, which only offers setting all the angles
    Leg::JointAngles current_joint_angles = legs_[manual_leg_idx_].getJointAngles();
    switch (manual_joint_idx_) {
      case Leg::JOINT_1:
        current_joint_angles.theta_1 = new_angle;
        break;
      case Leg::JOINT_2:
        current_joint_angles.theta_2 = new_angle;
        break;
      case Leg::JOINT_3:
        current_joint_angles.theta_3 = new_angle;
        break;
    }
    legs_[manual_leg_idx_].setJointAngles(current_joint_angles);
  }
}

Hexapod::ManualControlType Hexapod::getManualControlType() const {
  return manual_control_type_;
}

uint8_t Hexapod::getManualControlLegIdx() const {
  return manual_leg_idx_;
}

uint8_t Hexapod::getManualControlJointIdx() const {
  return manual_joint_idx_;
}

/**
 * @details
 * Advance gait_current_pos_ to the index of the next leg to be raised.
 *
 * Based on some basic rules for each gait.
 *
 */
void Hexapod::advanceGait() {
  switch (current_gait_seq_) {
    case Gait::RIPPLE:
      // next leg is on the other side and 1 'row' further back
      if (gait_current_pos_ % 2 == 0) {
        // on left
        gait_current_pos_ += 3;
      } else {
        gait_current_pos_ += 1;
      }
      if (gait_current_pos_ >= num_legs_) {
        if (gait_current_pos_ % 2 == 0) {
          if (num_legs_ / 2 % 2 == 0) {
            gait_current_pos_ = 1;
          } else {
            gait_current_pos_ = 0;
          }
        } else {
          if (num_legs_ / 2 % 2 == 0) {
            gait_current_pos_ = 0;
          } else {
            gait_current_pos_ = 1;
          }
        }
      }
      break;

    case Gait::LEFT_RIGHT_LEFT_RIGHT:
      gait_current_pos_ += 1;
      gait_current_pos_ = gait_current_pos_ % num_legs_;
      break;

    case Gait::LHS_THEN_RHS:
      gait_current_pos_ += 2;
      if (gait_current_pos_ >= num_legs_) {
        if (gait_current_pos_ % 2 == 0) {
          gait_current_pos_ = 1;
        } else {
          gait_current_pos_ = 0;
        }
      }
      break;

    case Gait::AROUND_THE_CLOCK:
      if (gait_current_pos_ % 2 == 0) {
        gait_current_pos_ += 2;
        if (gait_current_pos_ == num_legs_) {
          gait_current_pos_ = num_legs_ - 1;
        }
      } else {
        if (gait_current_pos_ == 1) {
          gait_current_pos_ = 0;
        } else {
          gait_current_pos_ -= 2;
        }
      }
      break;

    default:
      break;
  }
}

uint8_t Hexapod::gaitNextLeg() { return gait_current_pos_; }

uint8_t Hexapod::gaitMaxRaised() {
  return 1;  // currently fixed for all gaits
}

} // namespace hexapod