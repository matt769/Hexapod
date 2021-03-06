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
 * @param update_frequency
 */
Hexapod::Hexapod(const uint8_t num_legs, Dims hex_dims, Transform* tf_body_to_leg, Leg* legs, const uint16_t update_frequency)
    : dims_(hex_dims), num_legs_(num_legs), update_frequency_(update_frequency), height_(hex_dims.depth / 2.0f) {
  tf_base_to_body_ = Transform();
  tf_base_movement_ = Transform();
  tf_base_to_new_base_target_ = Transform();
  tf_base_to_body_target_ = Transform();

  legs_ = legs;
  tf_body_to_leg_ = tf_body_to_leg;

  updateMovementParameters();
  setUpdateFrequency(update_frequency_);
  setMovementIncrements();

  populateGaitInfo();

  printMovementParameters();
}

Hexapod::~Hexapod() {
  delete[] legs_;
  delete[] tf_body_to_leg_;
  for (uint8_t g = 0; g < 4; ++g) {
    delete[] gaits_[g].order;
    delete[] gaits_[g].offset;
  }
  // extra gait for 6 legged version
  if (num_legs_ == 6) {
    delete[] gaits_[4].order;
    delete[] gaits_[4].offset;
  }
}

/**
 * @details Expects to be called immediately after creating the hexapod object
 * @param update_frequency
 */
void Hexapod::setUpdateFrequency(const uint16_t update_frequency) {
  update_frequency_ = update_frequency;
  foot_air_time_default_ = (update_frequency_ / 4) * 2; // 0.5s
  foot_air_time_min_ = 2;
  foot_air_time_max_ = update_frequency_ * 2; // Double default - 2.0s
  foot_air_time_ = foot_air_time_default_;
}

void Hexapod::updateMovementParameters() {
  // set various movement parameters based on body/leg dimensions
  const float leg_length_full_extension = legs_[0].dims_.a + legs_[0].dims_.b + legs_[0].dims_.c;
  walk_height_default_ = leg_length_full_extension * 0.33f;
  leg_lift_height_min_ = walk_height_default_ * 0.1f;
  leg_lift_height_max_ = walk_height_default_;
  leg_lift_height_default_ = walk_height_default_ * 0.3f;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    legs_[leg_idx].updateMovementLimits(walk_height_default_, walk_height_default_ - leg_lift_height_default_);
  }
  const Leg::MovementLimits lml = legs_[0].calculateMovementLimits(walk_height_default_); // just for print out
  stance_width_default_ = lml.x_min + ((lml.x_max - lml.x_min) * 0.45);
  stance_width_min_ = lml.x_min;
  stance_width_max_ = lml.x_max;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    legs_[leg_idx].getNeutralPosition().x() = stance_width_default_;
  }

  allowed_foot_position_diameter_ = fmin(lml.x_max-lml.x_min, lml.y_max - lml.y_min) * 0.8; // TODO review

  stance_width_ = stance_width_default_;
  leg_lift_height_ = leg_lift_height_default_;
}

/**
 * @brief Derived from physical dimensions and (expected) update frequency
 */
void Hexapod::setMovementIncrements() {
  rising_increment_ = (walk_height_default_ - height_) / static_cast<float>(update_frequency_);

  // Movements applied every time step
  walk_translation_increment_ = (dims_.length / 4.0f) / static_cast<float>(update_frequency_);
  walk_turn_increment_ = (3.0f * M_PI / 180.0) / static_cast<float>(update_frequency_);
  // Movements applied each button press
  body_translation_increment_ = (dims_.width / 20.0f);
  stance_width_increment_ = (dims_.width / 20.0f);
  leg_raise_increment_ = leg_lift_height_default_ / 10.0f;
}

void Hexapod::printMovementParameters() {
  const float leg_length_full_extension = legs_[0].dims_.a + legs_[0].dims_.b + legs_[0].dims_.c;
  Vector3 neutral = legs_[0].getNeutralPosition();
  const Leg::MovementLimits lml = legs_[0].calculateMovementLimits(walk_height_default_);
#ifndef __AVR__
  std::cout << "body dimensions\t" << dims_.length << '\t' << dims_.width << '\t' << dims_.depth << '\n';
  std::cout << "leg neutral\t" << neutral.x() << '\t' << neutral.y() << '\t' << neutral.z() << '\n';
  std::cout << "leg_length_full_extension\t" << leg_length_full_extension << '\n';
  std::cout << "movement limits\t" << lml.x_max << '\t'  << lml.x_min << '\t' << lml.y_max << '\t' << lml.y_min << '\n';
  std::cout << "foot_air_time_default_\t" << foot_air_time_default_ << '\n';
  std::cout << "foot_air_time_min_\t" << foot_air_time_min_ << '\n';
  std::cout << "walk_height_default_\t" << walk_height_default_ << '\n';
  std::cout << "stance_width_default_\t" << stance_width_default_ << '\n';
  std::cout << "stance_width_min_\t" << stance_width_min_ << '\n';
  std::cout << "stance_width_max_\t" << stance_width_max_ << '\n';
  std::cout << "leg_lift_height_min_\t" << leg_lift_height_min_ << '\n';
  std::cout << "leg_lift_height_max_\t" << leg_lift_height_max_ << '\n';
  std::cout << "leg_lift_height_default_\t" << leg_lift_height_default_ << '\n';
  std::cout << "allowed_foot_position_diameter_\t" << allowed_foot_position_diameter_ << '\n';
  std::cout << "rising_increment_\t" << rising_increment_ << '\n';
#endif
}


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

uint16_t Hexapod::getUpdateFrequency() const {
  return update_frequency_;
}


/**
 * @details
 * Calculate the required angles for all legs on the ground to accommodate the desired walk/turn
 *  movement plus any body offset/rotation
 * @return true if an IK solution is found for all legs
 */
bool Hexapod::calculateGroundedLegs() {
  const Transform tf_base_to_new_body = tf_base_to_new_base_target_ * tf_base_to_body_target_;
  bool ik_success_all = true;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (legs_[leg_idx].state_ == Leg::State::ON_GROUND) {
      // current foot position in base frame - this is not going to change
      const Vector3 foot_in_base =
          tf_base_to_body_ * tf_body_to_leg_[leg_idx] * legs_[leg_idx].getFootPosition();
      // position of foot in the updated leg frame (base on walk and body movement)
      const Vector3 leg_to_foot_new =
          (tf_base_to_new_body * tf_body_to_leg_[leg_idx]).inverse() * foot_in_base;
      const bool ik_success = legs_[leg_idx].calculateJointAngles(leg_to_foot_new, Leg::IKMode::WALK);
      ik_success_all &= ik_success;
      if (!ik_success) {
#ifndef __AVR__
        std::cout << "Unable to find IK solution for grounded leg " << (int)leg_idx << '\n';
#endif
      }
    }
  }

  if (!ik_success_all) {
    for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
      if (legs_[leg_idx].state_ == Leg::State::ON_GROUND) {
        legs_[leg_idx].resetStagedAngles();
      }
    }
  }

  return ik_success_all;
}

void Hexapod::applyChangesGroundedLegs() {
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (legs_[leg_idx].state_ == Leg::State::ON_GROUND) {
      legs_[leg_idx].applyStagedAngles();
    }
  }
}

void Hexapod::commitLegJointChanges() {
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    legs_[leg_idx].applyStagedAngles();
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
  // change approach a bit, so instead of checked the number raised vs max
  //  we check that the previous leg has finished
  // So we actually need to do this first make sure all raised legs that have finished have their status update before
  //  we try and raise anything else
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (legs_[leg_idx].state_ == Leg::State::RAISED) {
      legs_[leg_idx].updateStatus(false);
    }
  }

  // now see if we want to request a raise
  // TODO could this be simplified a little?
  // Go through in the order of the gait, not the leg indices
  //  and start with next in gait sequence
  uint8_t seq_no = gait_next_leg_seq_no_;
  for (uint8_t i = 0; i < num_legs_; ++i, seq_no = (seq_no+1) % num_legs_) {
    uint8_t prev_seq_no = (seq_no + num_legs_ - 1) % num_legs_;
    uint8_t prev_leg_idx = gaits_[current_gait_seq_].order[prev_seq_no];
    uint8_t leg_idx= gaits_[current_gait_seq_].order[seq_no];

    // if the previous leg has finished i.e. is grounded, then can raise the new one
    // Use the offset value to determine when during the the previous leg's trajectory the next one can start lifting
    // (offset = 0 -> straight away, offset 1 -> only when back on ground)
    bool prev_leg_complete = legs_[prev_leg_idx].getCurrentStepProgress() >= gaits_[current_gait_seq_].offset[prev_seq_no];

    if (base_change_ && prev_leg_complete && legs_[leg_idx].state_ == Leg::State::ON_GROUND) {
      updateFootTarget(leg_idx); // TODO I think this can (and should) be removed because it's already called for
                                 //  all legs in updateFootTargets called in update() (unless there's anything
                                 //  significant happening inbetween but I don't think so
      bool raise_result = legs_[leg_idx].updateStatus(true);
      if (raise_result) {
        ++gait_next_leg_seq_no_;
        gait_next_leg_seq_no_ %= num_legs_;
      }
      // TODO for multi-leg gaits, what happens if some but not all legs can't raise
      //  I expect things will get weird
    } else {
      break; // stop at the first non-raise result, there can't be any more
    }
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
  const Vector3 base_to_neutral_new = tf_base_to_new_base_target_ * base_to_neutral;
  const Vector3 step = base_to_neutral_new - base_to_neutral;
  return step;
}

Vector3 Hexapod::legToBase(const uint8_t leg_idx, const Vector3& v) const {
  return tf_base_to_body_ * tf_body_to_leg_[leg_idx] * v;
}

/**
 * @details
 * The leg itself doesn't know how high the robot is, so leg::getNeutralPosition just has z = 0
 * so we just add it, but need to modify using tf_base_to_new_base_target_ in case there's a change pending.
 *
 * The neutral position does not change with the body, so we only need to apply the body_to_leg
 * transform to get the neutral position in the base frame i.e. as if the base to body transform is identity
 *
 * @param leg_idx
 * @return Vector3
 */
Vector3 Hexapod::getNeutralPosition(const uint8_t leg_idx) const {
  Vector3 leg_neutral = legs_[leg_idx].getNeutralPosition(); // TODO this is actually callng the non-const version and returning a modifyable ref
  leg_neutral.z() = -height_ + tf_base_to_new_base_target_.t_.z();
  return tf_body_to_leg_[leg_idx] * leg_neutral;
}

/**
 * @details
 * Calculate the required angles for all legs on the ground to accommodate the desired walk/turn.\n
 * If all successful then the required angles will be staged in the grounded legs.
 *
 * @return true if an IK solution was found for all grounded legs
 */
bool Hexapod::handleGroundedLegs() {
  // TODO this function has been mostly made redundant, probably remove later on in the refactor
  bool ik_result = calculateGroundedLegs();
  return ik_result;
}

/**
 * @details
 * Update the leg raise time, apex position and target position of a step.\n
 * The target is based upon the direction of the step vector.\n
 * The apex is a point half way between (in xy) the current and target position.
 *
 * @param leg_idx - calculate for which leg
 */
void Hexapod::updateFootTarget(const uint8_t leg_idx) {
  const Vector3 combined_step = calculateFootVector(leg_idx);
  float speed = combined_step.norm();
  Vector3 step_unit;
  if (compareFloat(speed, 0.0f, 0.0001f)) {
    step_unit = Vector3(0.0f, 0.0f, 0.0f);  // not actually a unit vector obviously
  } else {
    step_unit = combined_step.unit();
  }

  Vector3 raised_pos;
  Vector3 target_pos;
  // if the leg has already been lifted, use the already calculated targets
  //    we just need to modify them to account to changes in body position
  if (legs_[leg_idx].state_ == Leg::State::RAISED && legs_[leg_idx].getStepIdx() > 0) {
    // perhaps move this calculation 'higher' to avoid repeating (although will only repeat for
    // raised legs)
    const Transform tf_update = (tf_base_to_body_target_ * tf_body_to_leg_[leg_idx]).inverse() *
                          tf_base_to_body_ * tf_body_to_leg_[leg_idx];
    target_pos = tf_update * legs_[leg_idx].getTargetPosition();
    raised_pos = tf_update * legs_[leg_idx].getRaisedPosition();
    // also need to update the current position to account for the body change
    // I think I need to re-think the whole leg raise movement approach really
    // because it could probably be a lot simpler
    const Vector3 upd_current_pos = tf_update * legs_[leg_idx].getFootPosition();
    Leg::JointAngles angles;
    if (legs_[leg_idx].calculateJointAngles(upd_current_pos, Leg::IKMode::WALK, angles)) {
      legs_[leg_idx].setStagedAngles(angles); // (P)REFACTOR should I really be doing this here?
    }
  }
  // if the leg is only just about to become raised then need to calculate targets for first time
  // also do this if the foot is on the ground i.e. target update doesn't relate to a step
  //  (this allows the leg to respond to changes in e.g. stance width)
  else {
    const Vector3 neutral_pos = getNeutralPosition(leg_idx);
    // Target is ahead of neutral along step unit direction
    const Vector3 target_pos_in_base = neutral_pos + (allowed_foot_position_diameter_/2.0f) * step_unit;
    // Raised point half way between current position and target
    const Vector3 current_pos_in_base = getFootPosition(leg_idx);
    Vector3 raised_pos_in_base = current_pos_in_base + 0.5f * (target_pos_in_base - current_pos_in_base);
    raised_pos_in_base.z() = neutral_pos.z() + leg_lift_height_;

    // transform the step vector from base frame to leg base
    // can ignore the new base to base stuff since we only care about the relative position to the
    // base, wherever it is
    const Transform tf_leg_to_base = (tf_base_to_body_ * tf_body_to_leg_[leg_idx]).inverse();
    raised_pos = tf_leg_to_base * raised_pos_in_base;
    target_pos = tf_leg_to_base * target_pos_in_base;
  }

  legs_[leg_idx].updateTargets(target_pos, raised_pos, foot_air_time_);
}

void Hexapod::updateRaisedFootTargets() {
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

bool Hexapod::handleRaisedLegs() {
  bool leg_movement_result = true;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    if (legs_[leg_idx].state_ == Leg::State::RAISED) {
      leg_movement_result &= legs_[leg_idx].stepUpdate();
    }
  }
  return leg_movement_result;
}


bool Hexapod::setWalk(const Vector3& walk_step, const float angle_step, const bool force) {
  // don't set if not in walking state, or if trying to leave walking state
  if (force || (state_ == State::WALKING && requested_state_ == State::WALKING)) {
    current_walk_translation_ = walk_step;
    current_walk_turn_ = angle_step;
    return true; // TODO add checks on input(?)
  }
  return false;
}

bool Hexapod::setWalk(const Vector3& walk_step) { return setWalk(walk_step, 0.0f); }

bool Hexapod::setWalk(const float angle_step) { return setWalk(Vector3(0.0f, 0.0f, 0.0f), angle_step); }

bool Hexapod::changeWalk(const Vector3& walk_step, float angle_step) {
  return setWalk(current_walk_translation_ + walk_step, current_walk_turn_ + angle_step);
}

bool Hexapod::changeWalk(const Vector3& walk_step) { return changeWalk(walk_step, 0.0f); }

bool Hexapod::changeWalk(float angle_step)  { return changeWalk(Vector3(0.0f, 0.0f, 0.0f), angle_step); }

bool Hexapod::setWalkingTargets() {
  if (state_ != State::WALKING) {
    return false;
  }

  if (move_mode_ == MoveMode::HEADLESS) {
    float x =
        cos(-total_base_rotation_) * current_walk_translation_.x() - sin(-total_base_rotation_) * current_walk_translation_.y();
    float y =
        sin(-total_base_rotation_) * current_walk_translation_.x() + cos(-total_base_rotation_) * current_walk_translation_.y();
    walk_step_target_ = Vector3(x, y, 0);
  } else {
    walk_step_target_ = current_walk_translation_;
  }
  turn_step_target_ = current_walk_turn_;
  tf_base_to_new_base_target_.R_.setRPYExtr(0.0f, 0.0f, turn_step_target_);
  tf_base_to_new_base_target_.t_ = walk_step_target_;
  base_change_ = true;
  // If there's a change, need to update raised feet target, unless now stopped in which case update
  // all to allow feet to return to neutral position
  // Current Vector3 comparison allows some tolerance, here explicity test if zero
  if (walk_step_current_ != walk_step_target_ || turn_step_current_ != turn_step_target_) {
    if (walk_step_target_ == Vector3{0.0f, 0.0f, 0.0f} && turn_step_target_ == 0.0f) {
      recalculate_all_feet_targets_ = true;
    } else {
      recalculate_raised_feet_targets_ = true;
    }
  }
  return true;
}

bool Hexapod::setBody(const Transform& tf_base_to_body_target, const bool force) {
  if (force || (state_ == State::WALKING && requested_state_ == State::WALKING)) {
    tf_base_to_body_target_ = tf_base_to_body_target;
    body_change_ = true;
    recalculate_raised_feet_targets_ = true;
    return true;
  }
  return false;
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

bool Hexapod::clearWalk() { return setWalk(Vector3(0.0f, 0.0f, 0.0f), 0.0f); }

/**
 * @details
 * The hexapod will not move until another movement type command (e.g. setWalk, changeBody) is given
 *
 */
void Hexapod::clearTargets() {
  base_change_ = false;
  body_change_ = false;
  tf_base_to_body_target_ = tf_base_to_body_;
  tf_base_to_new_base_target_ = Transform();
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
  setWalkingTargets();

  bool grounded_legs_result = true;
  bool raised_legs_result = true;

  if (state_ == State::UNSUPPORTED) {
    updateMoveLegs();
  } else if (state_ == State::STANDING) {
    // nothing to do here at the moment
  } else if (state_ == State::RAISING) {
    changeBase(Vector3(0, 0, rising_increment_));
    grounded_legs_result = handleGroundedLegs();
  } else if (state_ == State::LOWERING) {
    changeBase(Vector3(0, 0, -rising_increment_));
    grounded_legs_result = handleGroundedLegs();
  } else if (state_ == State::WALKING) {
    grounded_legs_result = handleGroundedLegs();
    if (!grounded_legs_result) {
      clearTargets(); // couldn't achieve the desired movement
    }
    updateRaisedFootTargets();  // Update foot targets if required
    raised_legs_result = handleRaisedLegs();

  } else {
    // state_ == State::FULL_MANUAL
    // The legs have already been modified directly though the manualMoveFoot and manualChangeJoint functions
  }

  if (grounded_legs_result && raised_legs_result) {
    commitTargets();
  }
  // only check the raise leg results because if the grounded legs couldn't be updated, they just won't change
  //  and we can keep moving the raised legs even if we couldn't move the grounded ones
  if (raised_legs_result) {
    commitLegJointChanges();
    if (state_ == State::WALKING) {
      updateLegsStatus();  // Allow them (based on conditions) to change state between ON_GROUND and RAISED
    }
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
 * This does not do anything special when changing between single and multileg gaits. For mulit -> single
 *  it works naturally anyway. For single -> multi it looks very slightly odd but won't lift more than the
 *  defined in the multi-leg gait anyway.
 *
 * @param gait
 * @return true if the gait was changed
 */
bool Hexapod::changeGait(const Gait new_gait) {
  if (new_gait == current_gait_seq_ || new_gait >= Gait::NUM_GAITS) return false;

  // if there's currently a leg raised, we should set the next leg to the one after that in the new gait order
  // can do if nothing raised either, doesn't really matter
  uint8_t current_gait_current_leg_seq_no = (gait_next_leg_seq_no_ + num_legs_ -1) % num_legs_; // decrement with wrap
  uint8_t current_leg_idx = gaits_[current_gait_seq_].order[current_gait_current_leg_seq_no];
  // find this leg in the new gait
  for (uint8_t seq_no = 0; seq_no < num_legs_; ++seq_no) {
    if (gaits_[new_gait].order[seq_no] == current_leg_idx) {
      gait_next_leg_seq_no_ = seq_no; // current leg
    }
  }
  // and increment to point at the next leg again
  ++gait_next_leg_seq_no_;
  gait_next_leg_seq_no_ %= num_legs_;
  // and finally change the gait type
  current_gait_seq_ = new_gait;
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

bool Hexapod::setLegRaiseTime(uint16_t time) {
  if (time < foot_air_time_min_) time = foot_air_time_min_;
  else if (time > foot_air_time_max_) time = foot_air_time_max_;
  if (foot_air_time_ == time) {
    return false;
  } else {
    foot_air_time_ = time;
    return true;
  }
}

bool Hexapod::changeLegRaiseTime(const int16_t change) {
  const int16_t new_time = (int16_t)foot_air_time_ + change;
  if (new_time > 0) {
    return setLegRaiseTime((uint16_t)new_time);
  }
  else {
    return false;
  }
}

bool Hexapod::resetLegRaiseTime() { return setLegRaiseTime(foot_air_time_default_); }


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


bool Hexapod::setLegTarget(const uint8_t leg_idx, const Leg::JointAngles& joint_targets, const uint16_t duration) {
  bool res = legs_[leg_idx].jointsWithinLimits(joint_targets);
  if (!res) return false;

  const Leg::JointAngles current_angles = legs_[leg_idx].getJointAngles();
  const Leg::JointAngles angle_range{joint_targets.theta_1 - current_angles.theta_1,
                                     joint_targets.theta_2 - current_angles.theta_2,
                                     joint_targets.theta_3 - current_angles.theta_3};
  const Leg::JointAngles joint_increments{angle_range.theta_1 / static_cast<float>(duration),
                                          angle_range.theta_2 / static_cast<float>(duration),
                                          angle_range.theta_3 / static_cast<float>(duration)};
  // To fit existing setup will need to calculate midpoint although not strictly required
  const Leg::JointAngles
      midpoint{current_angles.theta_1 + joint_increments.theta_1 * static_cast<float>(duration/2),
               current_angles.theta_2 + joint_increments.theta_2 * static_cast<float>(duration/2),
               current_angles.theta_3 + joint_increments.theta_3 * static_cast<float>(duration/2)};

  legs_[leg_idx].setTrajectory(joint_targets,
                               joint_increments,
                               midpoint,
                               joint_increments,
                               duration);
  return true;
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
 * Facilitates the transition from STANDING state to WALKING state (via RISING state)
 * @return true if requested state set to RISING
 */
bool Hexapod::riseToWalk() {
  if (state_ == State::STANDING) {
    requested_state_ = State::RAISING;
    return true;
  }
  return false;
}

bool Hexapod::lowerToGround() {
  if (state_ == State::WALKING) {
    requested_state_ = State::LOWERING;
    return true;
  }
  return false;
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
  tf_base_to_new_base_target_.t_ = move_base;
  base_change_ = true;
  return true;
}

/**
 * @details
 * One of several basic functions for getting the hexapod to move from a starting position to an
 * upright position supported by the legs from which it can start walking.
 * Even if the target angles cannot be set for some of the legs, they will still be set for the others.
 *
 * @return true if targets were set for ALL legs
 */
bool Hexapod::setAllLegTargetsToGround(uint16_t duration) {
  bool result = true;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    result &= setLegTargetToGround(leg_idx, duration);
  }
  return result;
}

bool Hexapod::setAllLegTargetsToGround() {
  return setAllLegTargetsToGround(update_frequency_);
}

/**
 * @details Doesn't care whether the other legs are achievable or not. Will only consider this leg in isolation.
 * @param leg_idx
 * @param duration
 * @return
 */
bool Hexapod::setLegTargetToGround(const uint8_t leg_idx, const uint16_t duration)  {
  if (state_ != State::UNSUPPORTED) {
    return false;
  }

  // some default position
  Vector3 grounded_position = legs_[leg_idx].getNeutralPosition(); // TODO this is actually calling the non-const version and returning a modifyable ref
  grounded_position.z() = -height_;
  Leg::JointAngles grounded_angles;
  bool ik_result = legs_[leg_idx].calculateJointAngles(grounded_position, Leg::IKMode::WALK, grounded_angles);


  if (ik_result) {
    if (setLegTarget(leg_idx, grounded_angles, duration)) {
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
      Serial.print(F("State: STANDING\n"));
#else
      std::cout << "State changed to: STANDING\n";
#endif
    }
  }

  if (state_ == State::STANDING && requested_state_ == State::RAISING) {
    // immediately transition
    state_ = State::RAISING;
    requested_state_ = State::WALKING;
#ifdef __AVR__
    Serial.print(F("State: RISING\n"));
#else
    std::cout << "State changed to: RISING\n";
#endif
  }

  // go from standing to walking if base it at some predefined position
  // hexapod doesn't actually know base position except the height
  if (state_ == State::RAISING && requested_state_ == State::WALKING &&
      height_ >= walk_height_default_) {
    state_ = requested_state_;
#ifdef __AVR__
    Serial.print(F("State: WALKING\n"));
#else
    std::cout << "State changed to: WALKING\n";
#endif
  }

  if (state_ == State::WALKING && requested_state_ == State::LOWERING) {
    setWalk(Vector3{0.0f, 0.0f, 0.0f}, 0.0f, true);
    setBody(Transform(), true); // ideally do more smoothly
    // ensure that legs are allowed to finish current step
    bool ready_to_change = true;
    for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
      ready_to_change &= legs_[leg_idx].state_ == Leg::State::ON_GROUND;
    }
    if (ready_to_change) {
      state_ = requested_state_;
      requested_state_ = State::STANDING;
#ifdef __AVR__
      Serial.print(F("State: LOWERING\n"));
#else
      std::cout << "State changed to: LOWERING\n";
#endif
    }
  }

  if (state_ == State::LOWERING && requested_state_ == State::STANDING &&
      height_ <= dims_.depth / 2.0f) {
    state_ = requested_state_;
#ifdef __AVR__
    Serial.print(F("State: STANDING\n"));
#else
    std::cout << "State changed to: STANDING\n";
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

const Transform Hexapod::getBaseToLeg(const uint8_t leg_idx) {
  return tf_base_to_body_ * tf_body_to_leg_[leg_idx];
}

const Leg::MovementLimits& Hexapod::getMovementLimits(uint8_t leg_idx) {
  return legs_[leg_idx].movement_limits_grounded_;
}

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
    Leg::JointAngles angles;
    const bool ik_result = legs_[manual_leg_idx_].calculateJointAngles(new_pos, Leg::IKMode::FULL, angles);
    if (ik_result) {
      legs_[manual_leg_idx_].setStagedAngles(angles);
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
    legs_[manual_leg_idx_].setStagedAngles(current_joint_angles);
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

uint8_t Hexapod::gaitNextLeg() { return gaits_[current_gait_seq_].order[gait_next_leg_seq_no_]; }
// Check this still works for default gait
// Add back the exist gaits
// Add check on progress of previous leg, and implement use of offset
// Add tripod gait



uint8_t Hexapod::gaitMaxRaised() {
  return gaits_[current_gait_seq_].max_raised;
}

void Hexapod::commitTargets() {
    walk_step_current_ = walk_step_target_;
    turn_step_current_ = turn_step_target_;
    if (move_mode_ == MoveMode::HEADLESS) {
      total_base_rotation_ += turn_step_target_;
    }
    if (base_change_) {
      tf_base_movement_ = tf_base_to_new_base_target_;
      height_ += tf_base_to_new_base_target_.t_.z(); // (P)REFACTOR I think that only getNeutralPosition will be affected
    }
    if (body_change_) {
      tf_base_to_body_ = tf_base_to_body_target_;
    }
}

void Hexapod::populateGaitInfo() {
  Gait gait_type;
  gait_type = Gait::RIPPLE;
  gaits_[gait_type] = GaitDefinition{new uint8_t[num_legs_], new float[num_legs_], 1};
  uint8_t leg = 0;
  for (uint8_t seq_no = 0; seq_no < num_legs_; seq_no++) {
    gaits_[gait_type].order[seq_no] = leg;
    gaits_[gait_type].offset[seq_no] = 0.34;
    // next leg is on the other side and 1 'row' further back
    if (leg % 2 == 0) {
      // on left
      leg += 3;
    } else {
      leg += 1;
    }
    if (leg >= num_legs_) {
      if (leg % 2 == 0) {
        if (num_legs_ / 2 % 2 == 0) {
          leg = 1;
        } else {
          leg = 0;
        }
      } else {
        if (num_legs_ / 2 % 2 == 0) {
          leg = 0;
        } else {
          leg = 1;
        }
      }
    }
  }

  gait_type = Gait::LEFT_RIGHT_LEFT_RIGHT;
  gaits_[gait_type] = GaitDefinition{new uint8_t[num_legs_], new float[num_legs_], 1};
  leg = 0;
  for (uint8_t seq_no = 0; seq_no < num_legs_; seq_no++) {
    gaits_[gait_type].order[seq_no] = leg;
    gaits_[gait_type].offset[seq_no] = 1.0;
    ++leg;
    leg = leg % num_legs_; // redundant
  }

  gait_type = Gait::LHS_THEN_RHS;
  gaits_[gait_type] = GaitDefinition{new uint8_t[num_legs_], new float[num_legs_], 1};
  leg = 0;
  for (uint8_t seq_no = 0; seq_no < num_legs_; seq_no++) {
    gaits_[gait_type].order[seq_no] = leg;
    gaits_[gait_type].offset[seq_no] = 1.0;
    leg += 2;
    if (leg >= num_legs_) {
      if (leg % 2 == 0) {
        leg = 1;
      } else {
        leg = 0;
      }
    }
  }

  gait_type = Gait::AROUND_THE_CLOCK;
  gaits_[gait_type] = GaitDefinition{new uint8_t[num_legs_], new float[num_legs_], 1};
  leg = 0;
  for (uint8_t seq_no = 0; seq_no < num_legs_; seq_no++) {
    gaits_[gait_type].order[seq_no] = leg;
    gaits_[gait_type].offset[seq_no] = 1.0;
    if (leg % 2 == 0) {
      leg += 2;
      if (leg == num_legs_) {
        leg = num_legs_ - 1;
      }
    } else {
      if (leg == 1) {
        leg = 0;
      } else {
        leg -= 2;
      }
    }
  }

  if (num_legs_ == 6) {
    gait_type = Gait::TRIPOD;
    gaits_[gait_type] = GaitDefinition{new uint8_t[num_legs_], new float[num_legs_], 3};
    gaits_[gait_type].order[0] = 0;
    gaits_[gait_type].order[1] = 3;
    gaits_[gait_type].order[2] = 4;
    gaits_[gait_type].order[3] = 1;
    gaits_[gait_type].order[4] = 2;
    gaits_[gait_type].order[5] = 5;
    gaits_[gait_type].offset[0] = 0.0;
    gaits_[gait_type].offset[1] = 0.0;
    gaits_[gait_type].offset[2] = 1.0;
    gaits_[gait_type].offset[3] = 0.0;
    gaits_[gait_type].offset[4] = 0.0;
    gaits_[gait_type].offset[5] = 1.0;
  }
}

} // namespace hexapod