#include "hexapod.h"

#include "kinematics_support.h"
#include "leg.h"
#include "transformations.h"

#include <stddef.h>
#include <cmath>
#include <iostream>

using namespace KinematicsSupport;
using namespace Transformations;

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
Hexapod::Hexapod(size_t num_legs, Dims hex_dims, Tfm::Transform* tf_body_to_leg, Leg* legs)
    : dims_(hex_dims), num_legs_(num_legs), height_(hex_dims.depth / 2.0f) {
  tf_base_to_body_ = Transform();
  tf_base_to_body_prev_ = Transform();
  tf_base_movement_ = Transform();
  tf_base_to_new_base_ = Transform();
  tf_base_to_body_target_ = Transform();

  legs_ = legs;
  tf_body_to_leg_ = tf_body_to_leg;

  for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    tf_body_to_leg_[leg_idx] = tf_body_to_leg[leg_idx];
    legs_[leg_idx] = legs[leg_idx];
  }
}

Hexapod::~Hexapod() {
  delete[] legs_;
  delete[] tf_body_to_leg_;
}

/**
 * @details
 * Mostly redundant - will likely be removed later.
 *
 * @param starting_angles
 * @return true if ajoint angles set successfully
 */
bool Hexapod::setStartingPosition(Leg::JointAngles starting_angles) {
  bool result = true;
  for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    result &=
        legs_[leg_idx].setStartingAngles(starting_angles);  // get into close enough position so
                                                            // that IK will pick the right option
                                                            // later
  }
  // if legs below body, then adjust height_
  // TODO they shouldn't be below - remove this section?
  float leg_pos_z = legs_[0].getFootPosition().z();
  if (leg_pos_z < -height_) {
    height_ = -leg_pos_z;
  }
  return result;
}

/**
 * @details
 * @return true if an IK solution is found for all legs
 */
bool Hexapod::calculateGroundedLegs() {
  Transform tf_base_to_new_body = tf_base_to_new_base_ * tf_base_to_body_target_;
  for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (legs_[leg_idx].state_ == Leg::State::ON_GROUND) {
      // current foot position in base frame - this is not going to change
      Vector3 foot_in_base =
          tf_base_to_body_ * tf_body_to_leg_[leg_idx] * legs_[leg_idx].getFootPosition();
      // position of foot in the updated leg frame (base on walk and body movement)
      Vector3 leg_to_foot_new =
          (tf_base_to_new_body * tf_body_to_leg_[leg_idx]).inverse() * foot_in_base;
      bool result = legs_[leg_idx].calculateJointAngles(leg_to_foot_new, Leg::IKMode::WALK);
      if (!result) {
        std::cout << "Unable to find IK solution for all legs.\n";
        return false;
      }
    }
  }
  return true;
}

void Hexapod::applyChangesGroundedLegs() {
  for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (legs_[leg_idx].state_ == Leg::State::ON_GROUND) {
      legs_[leg_idx].applyStagedAngles();
    }
  }
}

size_t Hexapod::getNumLegsRaised() const {
  size_t num_legs_raised = 0;
  for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
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
void Hexapod::updateLegs() {
  size_t num_legs_raised = getNumLegsRaised();
  bool raise_result = false;
  for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    // check if this leg is next to be lifted, and if we want to allow it
    if (base_change_ && legs_[leg_idx].state_ == Leg::State::ON_GROUND &&
        gaitNextLeg() == leg_idx && num_legs_raised < gaitMaxRaised()) {
      updateFootTarget(leg_idx);
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
Vector3 Hexapod::calculateFootVector(const size_t leg_idx) const {
  Vector3 base_to_neutral = tf_body_to_leg_[leg_idx] * getNeutralPosition(leg_idx);
  Vector3 base_to_neutral_new = tf_base_to_new_base_ * base_to_neutral;
  Vector3 step = base_to_neutral_new - base_to_neutral;
  return step;
}

Vector3 Hexapod::legToBase(const size_t leg_idx, const Vector3& v) const {
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
Vector3 Hexapod::getNeutralPosition(const size_t leg_idx) const {
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
    std::cout << "Unable to find IK solution for all grounded legs.\n";
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
void Hexapod::updateFootTarget(size_t leg_idx) {
  Vector3 combined_step = calculateFootVector(leg_idx);
  size_t foot_air_time;  // how long we want the foot in the air for

  float speed = combined_step.norm();
  Vector3 step_unit;
  if (compareFloat(speed, 0.0f, 0.0001f)) {
    step_unit = Vector3(0.0f, 0.0f, 0.0f);  // not actually a unit vector obviously
    speed = 0.0f;
    foot_air_time = foot_air_time_default_;
  } else {
    step_unit = combined_step.unit();

    // Check desired speed within limits and cap it if not
    size_t min_foot_ground_time = foot_air_time_min_ * (num_legs_ - gaitMaxRaised());
    // max speed if travel full allowed distance in the minimum allowed time
    float max_v = allowed_foot_position_.dia / (float)min_foot_ground_time;
    if (speed > max_v) {
      std::cout << "Speed over limit. Capped to " << max_v << '\n';
      speed = max_v;
    }

    // Convert speed to number of time steps that foot will be on the ground
    float foot_ground_distance =
        allowed_foot_position_.dia * foot_ground_travel_ratio_;  // stride length
    float foot_ground_time_fl = foot_ground_distance / speed;
    // and in the air - make sure it's even (round up if not)
    float foot_air_time_fl = foot_ground_time_fl / (float)(num_legs_ - gaitMaxRaised());
    foot_air_time = (size_t)ceilf(foot_air_time_fl);
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
    Transform tf_update = (tf_base_to_body_ * tf_body_to_leg_[leg_idx]).inverse() *
                          tf_base_to_body_prev_ * tf_body_to_leg_[leg_idx];
    target_pos = tf_update * legs_[leg_idx].getTargetPosition();
    raised_pos = tf_update * legs_[leg_idx].getRaisedPosition();
    // also need to update the current position to account for the body change
    // I think I need to re-think the whole leg raise movement approach really
    // because it could probably be a lot simpler
    Vector3 upd_current_pos = tf_update * legs_[leg_idx].getFootPosition();
    legs_[leg_idx].calculateJointAngles(upd_current_pos, Leg::IKMode::WALK);
    if (legs_[leg_idx].calculateJointAngles(upd_current_pos, Leg::IKMode::WALK)) {
      legs_[leg_idx].applyStagedAngles();
    }
  }
  // if the leg os only just about to become raised then need to calculate targets for first time
  // also do this if the foot is on the ground i.e. target update doesn't relate to a step
  else {
    Vector3 neutral_pos = getNeutralPosition(leg_idx);
    Vector3 raised_pos_in_base = neutral_pos;
    raised_pos_in_base.z() += leg_lift_height_;

    // HACK add 2 (times num feet on ground) to account for a few steps overhead in changing state
    // that means feet are actually on the ground longer than calculated
    const size_t foot_ground_time = 2 + foot_air_time * (num_legs_ - gaitMaxRaised());
    const float half_distance_to_travel = ((float)foot_ground_time * speed) / 2.0f;
    Vector3 target_pos_in_base = neutral_pos + half_distance_to_travel * step_unit;
    // transform the step vector from base frame to leg base
    // can ignore the new base to base stuff since we only care about the relative position to the
    // base, wherever it is
    Transform tf_leg_to_base = (tf_base_to_body_ * tf_body_to_leg_[leg_idx]).inverse();
    raised_pos = tf_leg_to_base * raised_pos_in_base;
    target_pos = tf_leg_to_base * target_pos_in_base;
  }

  legs_[leg_idx].updateTargets(target_pos, raised_pos, foot_air_time);
}

void Hexapod::updateFootTargets() {
  for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (recalculate_all_feet_targets_ ||
        (recalculate_raised_feet_targets_ && legs_[leg_idx].state_ == Leg::State::RAISED)) {
      updateFootTarget(leg_idx);
    }
  }
}

Vector3 Hexapod::getFootPosition(const size_t leg_idx) const {
  return legToBase(leg_idx, legs_[leg_idx].getFootPosition());
}

Vector3 Hexapod::getTargetPosition(const size_t leg_idx) const {
  return legToBase(leg_idx, legs_[leg_idx].getTargetPosition());
}

Vector3 Hexapod::getRaisedPosition(const size_t leg_idx) const {
  return legToBase(leg_idx, legs_[leg_idx].getRaisedPosition());
}

void Hexapod::handleRaisedLegs() {
  for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (legs_[leg_idx].state_ == Leg::State::RAISED) {
      legs_[leg_idx].stepUpdate();
    }
  }
}

bool Hexapod::setWalk(const Vector3& walk_step, float angle_step) {
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

bool Hexapod::setWalk(float angle_step) { return setWalk(Vector3(0.0f, 0.0f, 0.0f), angle_step); }

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
 * TODO - move updateFootTargets and updateLegs into handleRaisedLegs?
 *
 * @return true always - this is now redundant
 */
bool Hexapod::update() {
  clearVisualisationChanges();

  if (state_ == State::UNSUPPORTED) {
    updateMoveLegs();
  } else if (state_ == State::STANDING) {
    handleGroundedLegs();
  } else {
    handleGroundedLegs();
    updateFootTargets();  // Update foot targets if required for other non-raising legs
    handleRaisedLegs();   // We can keep moving the raised legs even if we couldn't move the ones on
                          // the ground
    updateLegs();  // Allow them (based on conditions) to change state between ON_GROUND and RAISED
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
    for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
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
bool Hexapod::changeStanceWidth(float change) {
  float new_stance_width = stance_width_ + change;
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
bool Hexapod::changeGait(Gait gait) {
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
  float new_height = leg_lift_height_ + change;
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
void Hexapod::setMoveMode(MoveMode move_mode) {
  move_mode_ = move_mode;
  if (move_mode_ == MoveMode::HEADLESS) {
    total_base_rotation_ = 0.0f;
  }
}

/**
 * @details
 * One of several basic functions for getting the hexapod to move from a starting position to an
 * upright
 *  position supported by the legs from which it can start walking.
 *
 * TODO - maybe refactor.
 *
 * @param joint_targets
 * @return true if the requested targets were set
 */
bool Hexapod::setTargetsMoveLegs(Leg::JointAngles joint_targets) {
  if (state_ != State::UNSUPPORTED) {
    return false;
  }

  // check that targets are achievable
  if (!legs_[0].jointsWithinLimits(joint_targets)) {
    std::cout << "Requested movement not achievable\n";
    return false;
  }

  // movement time DEFAULT
  total_movement_steps_ = 50;
  current_movement_step_ = 0;
  sub_state_ = SubState::IN_PROGRESS;

  // calculate trajectory
  // if start and end angles were ok, then everything in between should be too
  // since we never try and go the 'shorter' way around i.e. don't cross -180/+180 boundary
  joint_targets_ = joint_targets;
  Leg::JointAngles current_angles = legs_[0].getJointAngles();
  Leg::JointAngles angle_range{joint_targets_.theta_1 - current_angles.theta_1,
                               joint_targets_.theta_2 - current_angles.theta_2,
                               joint_targets_.theta_3 - current_angles.theta_3};
  joint_increments_ = Leg::JointAngles{angle_range.theta_1 / (float)total_movement_steps_,
                                       angle_range.theta_2 / (float)total_movement_steps_,
                                       angle_range.theta_3 / (float)total_movement_steps_};
  // TODO should this stuff be stored in Leg rather than hexapod
  // I think probably yes but for now will stay here
  return true;
}

/**
 * @details
 * One of several basic functions for getting the hexapod to move from a starting position to an
 * upright
 *  position supported by the legs from which it can start walking.
 *
 * TODO - maybe refactor.
 *
 * @return true if the movement was achieved
 */
bool Hexapod::updateMoveLegs() {
  if (state_ == State::STANDING) {
    return false;
  }

  if (!(sub_state_ == SubState::IN_PROGRESS)) {
    return false;
  }

  // We don't check if joints were set
  // We just assume they were
  if (current_movement_step_ >= total_movement_steps_) {
    for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
      legs_[leg_idx].setJointAngles(joint_targets_);
    }
    sub_state_ = SubState::FINISHED;
  } else {
    for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
      Leg::JointAngles current_angles = legs_[leg_idx].getJointAngles();
      Leg::JointAngles new_angles{current_angles.theta_1 + joint_increments_.theta_1,
                                  current_angles.theta_2 + joint_increments_.theta_2,
                                  current_angles.theta_3 + joint_increments_.theta_3};
      legs_[leg_idx].setJointAngles(new_angles);
    }
    current_movement_step_++;
  }

  return true;
}

/**
 * @details
 * One of several basic functions for getting the hexapod to move from a starting position to an
 * upright
 *  position supported by the legs from which it can start walking.
 *
 * TODO - maybe refactor.
 *
 * @return true
 * @return false
 */
bool Hexapod::riseToWalk() {
  if (!(state_ == State::STANDING)) {
    return false;
  }

  if (height_ < walk_height_default_) {
    changeBase(Vector3(0, 0, 0.01f));
    requested_state_ = State::WALKING;
    return true;
  }
  return false;
}

/**
 * @details
 * One of several basic functions for getting the hexapod to move from a starting position to an
 * upright
 *  position supported by the legs from which it can start walking.
 *
 * TODO - maybe refactor.
 *
 * @param move_base
 * @return true always
 */
bool Hexapod::changeBase(Vector3 move_base) {
  tf_base_to_new_base_.t_.z() = move_base.z();
  base_change_ = true;
  return true;
}

/**
 * @details
 * One of several basic functions for getting the hexapod to move from a starting position to an
 * upright
 *  position supported by the legs from which it can start walking.
 *
 * TODO - maybe refactor, does not currently support different sized legs.
 *
 * @return true if an IK solution was found for all legs
 */
bool Hexapod::setLegsToGround() {
  if (!(state_ == State::UNSUPPORTED)) {
    return false;
  }
  // some default position
  Vector3 grounded_position = legs_[0].getNeutralPosition();
  grounded_position.z() = -height_;
  bool result = legs_[0].calculateJointAngles(grounded_position, Leg::IKMode::WALK);
  if (result) {
    result &= setTargetsMoveLegs(legs_[0].getStagedAngles());
  }
  requested_state_ = State::STANDING;
  return result;
}

void Hexapod::handleStateChange() {
  if (state_ == State::UNSUPPORTED && requested_state_ == State::STANDING) {
    // check all legs on floor
    bool ready = true;
    for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
      // get position in base frame
      float leg_height = getFootPosition(leg_idx).z();
      float floor_height = -height_;
      if (!compareFloat(leg_height, floor_height, 0.0001f)) {
        ready = false;
      }
    }
    if (ready) {
      state_ = requested_state_;
      std::cout << "State changed to: STANDING\n";
    }
  }

  // go from standing to walking if base it at some predefined position
  // hexapod doesn't actually know base position except the height
  if (state_ == State::STANDING && requested_state_ == State::WALKING &&
      compareFloat(height_, walk_height_default_, 0.0001f)) {
    state_ = requested_state_;
    std::cout << "State changed to: WALKING\n";
  }

  // add some conditions for going back
  // will need to ensure that legs are allowed to finish current step
  if (state_ == State::WALKING && requested_state_ == State::STANDING && false) {
    state_ = requested_state_;
    std::cout << "State changed to: STANDING\n";
  }

  // add some conditions for going back
  if (state_ == State::STANDING && requested_state_ == State::UNSUPPORTED && false) {
    state_ = requested_state_;
    std::cout << "State changed to: UNSUPPORTED\n";
  }
}

Hexapod::State Hexapod::getState() const { return state_; }

const Leg& Hexapod::getLeg(size_t leg_idx) const { return legs_[leg_idx]; }

const Tfm::Transform& Hexapod::getBaseToBody() const { return tf_base_to_body_; }

const Tfm::Transform& Hexapod::getBaseMovement() const { return tf_base_movement_; }

float Hexapod::getHeight() const { return height_; }

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

size_t Hexapod::gaitNextLeg() { return gait_current_pos_; }

size_t Hexapod::gaitMaxRaised() {
  return 1;  // currently fixed for all gaits
}

Hexapod buildDefaultHexapod() {
  // Construct a leg (they are all the same in this default robot)
  constexpr size_t num_joints = 3;
  Leg::Dims leg_dims{0.2f, 0.4f, 0.6f};

  Joint joints[num_joints];
  joints[0] = Joint(-90.0f * M_PI / 180.0, 90.0f * M_PI / 180.0, 0.0);
  joints[1] = Joint(-150.0f * M_PI / 180.0, 150.0f * M_PI / 180.0, M_PI / 2.0);
  joints[2] = Joint(-150.0f * M_PI / 10.0, 150.0f * M_PI / 180.0, M_PI / 4.0);

  Leg leg(leg_dims, joints);

  // Make an array of legs and copy the one we just made into all elements
  constexpr size_t num_legs = 6;
  Leg* legs = new Leg[num_legs];
  for (size_t leg_idx = 0; leg_idx < num_legs; leg_idx++) {
    legs[leg_idx] = leg;
  }

  // Hexapod body dimensions
  constexpr float length = 1.8f;
  constexpr float width = 1.0f;
  constexpr float depth = 0.2f;
  const Hexapod::Dims hex_dims{length, width, depth};

  // Transformations between body frame and leg base frames
  // LHS leg base frames need to be rotated by 90, RHS by -90
  Transform* tf_body_to_leg = new Transform[num_legs];

  Vector3 front((hex_dims.length / 2.0f) * 0.6666666f, 0.0f, 0.0f);
  Vector3 back = -front;
  Vector3 left(0.0f, hex_dims.width / 2.0f, 0.0f);
  Vector3 right = -left;
  Vector3 middle(0.0f, 0.0f, 0.0f);

  // Legs MUST be ordered this way i.e. left then right, then next row left ...
  enum { FRONT_LEFT = 0, FRONT_RIGHT, MIDDLE_LEFT, MIDDLE_RIGHT, BACK_LEFT, BACK_RIGHT };

  tf_body_to_leg[FRONT_LEFT].t_ = front + left;
  tf_body_to_leg[FRONT_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[MIDDLE_LEFT].t_ = middle + left;
  tf_body_to_leg[MIDDLE_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[BACK_LEFT].t_ = back + left;
  tf_body_to_leg[BACK_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[FRONT_RIGHT].t_ = front + right;
  tf_body_to_leg[FRONT_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[MIDDLE_RIGHT].t_ = middle + right;
  tf_body_to_leg[MIDDLE_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[BACK_RIGHT].t_ = back + right;
  tf_body_to_leg[BACK_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  return Hexapod(num_legs, hex_dims, tf_body_to_leg, legs);
}

Hexapod buildDefaultHexapod2() {
  // Construct a leg (they are all the same in this default robot)
  constexpr size_t num_joints = 3;
  Leg::Dims leg_dims{0.2f, 0.4f, 0.6f};

  Joint joints[num_joints];
  joints[0] = Joint(-90.0f * M_PI / 180.0, 90.0f * M_PI / 180.0, 0.0);
  joints[1] = Joint(-150.0f * M_PI / 180.0, 150.0f * M_PI / 180.0, M_PI / 2.0);
  joints[2] = Joint(-150.0f * M_PI / 10.0, 150.0f * M_PI / 180.0, M_PI / 4.0);

  Leg leg(leg_dims, joints);

  // Make an array of legs and copy the one we just made into all elements
  constexpr size_t num_legs = 6;
  Leg* legs = new Leg[num_legs];
  for (size_t leg_idx = 0; leg_idx < num_legs; leg_idx++) {
    legs[leg_idx] = leg;
  }

  // Hexapod body dimensions
  constexpr float length = 1.8f;
  constexpr float width = 1.0f;
  constexpr float depth = 0.2f;
  const Hexapod::Dims hex_dims{length, width, depth};

  // Transformations between body frame and leg base frames
  // LHS leg base frames need to be rotated by 90, RHS by -90
  Transform* tf_body_to_leg = new Transform[num_legs];

  Vector3 front(hex_dims.length / 2.0f, 0.0f, 0.0f);
  Vector3 back = -front;
  Vector3 left(0.0f, hex_dims.width / 2.0f, 0.0f);
  Vector3 right = -left;
  Vector3 middle(0.0f, 0.0f, 0.0f);

  // Legs MUST be ordered this way i.e. left then right, then next row left ...
  enum { FRONT_LEFT = 0, FRONT_RIGHT, MIDDLE_LEFT, MIDDLE_RIGHT, BACK_LEFT, BACK_RIGHT };

  tf_body_to_leg[FRONT_LEFT].t_ = front + left;
  tf_body_to_leg[FRONT_LEFT].R_.setRPYExtr(0, 0, M_PI / 3.0);

  tf_body_to_leg[MIDDLE_LEFT].t_ = middle + left;
  tf_body_to_leg[MIDDLE_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[BACK_LEFT].t_ = back + left;
  tf_body_to_leg[BACK_LEFT].R_.setRPYExtr(0, 0, 2.0 * M_PI / 3.0);

  tf_body_to_leg[FRONT_RIGHT].t_ = front + right;
  tf_body_to_leg[FRONT_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 3.0);

  tf_body_to_leg[MIDDLE_RIGHT].t_ = middle + right;
  tf_body_to_leg[MIDDLE_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[BACK_RIGHT].t_ = back + right;
  tf_body_to_leg[BACK_RIGHT].R_.setRPYExtr(0, 0, -2.0 * M_PI / 3.0);

  return Hexapod(num_legs, hex_dims, tf_body_to_leg, legs);
}

Hexapod buildDefaultOctapod() {
  // Construct a leg (they are all the same in this default robot)
  constexpr size_t num_joints = 3;
  Leg::Dims leg_dims{0.2f, 0.4f, 0.6f};

  Joint joints[num_joints];
  joints[0] = Joint(-90.0f * M_PI / 180.0, 90.0f * M_PI / 180.0, 0.0);
  joints[1] = Joint(-150.0f * M_PI / 180.0, 150.0f * M_PI / 180.0, M_PI / 2.0);
  joints[2] = Joint(-150.0f * M_PI / 10.0, 150.0f * M_PI / 180.0, M_PI / 4.0);

  Leg leg(leg_dims, joints);

  // Make an array of legs and copy the one we just made into all elements
  constexpr size_t num_legs = 8;
  Leg* legs = new Leg[num_legs];
  for (size_t leg_idx = 0; leg_idx < num_legs; leg_idx++) {
    legs[leg_idx] = leg;
  }

  // Hexapod body dimensions
  constexpr float length = 1.8f;
  constexpr float width = 1.0f;
  constexpr float depth = 0.2f;
  const Hexapod::Dims hex_dims{length, width, depth};

  // Transformations between body frame and leg base frames
  // LHS leg base frames need to be rotated by 90, RHS by -90
  Transform* tf_body_to_leg = new Transform[num_legs];

  Vector3 front((hex_dims.length / 2.0f), 0.0f, 0.0f);
  Vector3 front_mid((hex_dims.length / 2.0f) - (length / (num_legs / 2 - 1)), 0.0f, 0.0f);
  Vector3 back_mid = -front_mid;
  Vector3 back = -front;
  Vector3 left(0.0f, hex_dims.width / 2.0f, 0.0f);
  Vector3 right = -left;

  // Legs MUST be ordered this way i.e. left then right, then next row left ...
  enum {
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    FRONT_MIDDLE_LEFT,
    FRONT_MIDDLE_RIGHT,
    BACK_MIDDLE_LEFT,
    BACK_MIDDLE_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  };

  tf_body_to_leg[FRONT_LEFT].t_ = front + left;
  tf_body_to_leg[FRONT_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[FRONT_MIDDLE_LEFT].t_ = front_mid + left;
  tf_body_to_leg[FRONT_MIDDLE_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[BACK_MIDDLE_LEFT].t_ = back_mid + left;
  tf_body_to_leg[BACK_MIDDLE_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[BACK_LEFT].t_ = back + left;
  tf_body_to_leg[BACK_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[FRONT_RIGHT].t_ = front + right;
  tf_body_to_leg[FRONT_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[FRONT_MIDDLE_RIGHT].t_ = front_mid + right;
  tf_body_to_leg[FRONT_MIDDLE_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[BACK_MIDDLE_RIGHT].t_ = back_mid + right;
  tf_body_to_leg[BACK_MIDDLE_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[BACK_RIGHT].t_ = back + right;
  tf_body_to_leg[BACK_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  return Hexapod(num_legs, hex_dims, tf_body_to_leg, legs);
}