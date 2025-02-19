#include "hexapod_core/hexapod.h"

#include "hexapod_core/build_hexapod.h"
#include "hexapod_core/joint.h"
#include "hexapod_core/kinematics_support.h"
#include "hexapod_core/leg.h"
#include "hexapod_core/transformations.h"

#ifdef __AVR__
#include <Arduino.h>
#else
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#endif

namespace hexapod {

using namespace util;

bool Hexapod::BaseTranslationLevels::isZero() const { return x == 0 && y == 0; }

bool Hexapod::BaseMovementLevels::isZero() const { return t.isZero() && r == 0; }

// Note: may get narrowing conversion warnings when compiling these 4 functions not on microcontroller
Hexapod::BaseTranslationLevels operator+(const Hexapod::BaseTranslationLevels& a,
                                         const Hexapod::BaseTranslationLevels& b) {
  return Hexapod::BaseTranslationLevels{a.x + b.x, a.y + b.y};
}

Hexapod::BaseTranslationLevels operator-(const Hexapod::BaseTranslationLevels& a,
                                         const Hexapod::BaseTranslationLevels& b) {
  return Hexapod::BaseTranslationLevels{a.x - b.x, a.y - b.y};
}

bool operator==(const Hexapod::BaseTranslationLevels& a, const Hexapod::BaseTranslationLevels& b) {
  return a.x == b.x && a.y == b.y;
}
bool operator!=(const Hexapod::BaseTranslationLevels& a, const Hexapod::BaseTranslationLevels& b) { return !(a == b); }

Hexapod::BaseMovementLevels operator+(const Hexapod::BaseMovementLevels& a, const Hexapod::BaseMovementLevels& b) {
  return Hexapod::BaseMovementLevels{a.t + b.t, a.r + b.r};
}

Hexapod::BaseMovementLevels operator-(const Hexapod::BaseMovementLevels& a, const Hexapod::BaseMovementLevels& b) {
  return Hexapod::BaseMovementLevels{a.t - b.t, a.r - b.r};
}

bool operator==(const Hexapod::BaseMovementLevels& a, const Hexapod::BaseMovementLevels& b) {
  return a.t == b.t && a.r == b.r;
}
bool operator!=(const Hexapod::BaseMovementLevels& a, const Hexapod::BaseMovementLevels& b) { return !(a == b); }

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
Hexapod::Hexapod(const uint8_t num_legs, Dims hex_dims, Transform* tf_body_to_leg, Leg* legs,
                 const uint16_t update_frequency)
    : dims_(hex_dims), num_legs_(num_legs), update_frequency_(update_frequency), base_height_(hex_dims.depth / 2.0f) {
  tf_base_to_body_ = Transform();
  tf_base_movement_ = Transform();
  tf_base_to_new_base_target_ = Transform();
  tf_base_to_body_target_ = Transform();

  legs_ = legs;
  tf_body_to_leg_ = tf_body_to_leg;

  updateMovementParameters();
  populateGaitInfo();
  setUpdateFrequency(update_frequency_);
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
  foot_air_time_default_ = (update_frequency_ / 4) * 2;  // 0.5s
  foot_air_time_min_ = 2;
  foot_air_time_max_ = update_frequency_ * 2;  // Double default - 2.0s
  foot_air_time_ = foot_air_time_default_;
  setMovementIncrements();
  printMovementParameters();
}

// This should only be called in the initial setup
// TODO improve design
void Hexapod::updateMovementParameters() {
  // set various movement parameters based on body/leg dimensions
  // NOTE: Assumption that all legs are the same

  // Limits and defaults
  const float leg_length_full_extension = legs_[0].dims_.a + legs_[0].dims_.b + legs_[0].dims_.c;
  walk_height_default_ = leg_length_full_extension * 0.33f;
  leg_lift_height_min_ = walk_height_default_ * 0.1f;
  leg_lift_height_max_ = walk_height_default_;
  leg_lift_height_default_ = walk_height_default_ * 0.3f;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    // Leg calculates something hopefully reasonable on its own, but it does know exactly how it will be used so need
    //  to provide some extra info (walking heights)
    legs_[leg_idx].updateMovementLimits(walk_height_default_, walk_height_default_ - leg_lift_height_default_);
  }
  const Leg::MovementLimits lml = legs_[0].movement_limits_grounded_;
  stance_width_default_ = leg_length_full_extension * 0.6f;
  stance_width_min_ = lml.x_min;
  stance_width_max_ = lml.x_max;
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    // TODO getNeutralPosition shouldn’t be used to set things (it doesn’t look like a setter from the name)
    legs_[leg_idx].getNeutralPosition().x() = stance_width_default_;
  }

  allowed_foot_position_diameter_ = fmin(lml.x_max - lml.x_min, lml.y_max - lml.y_min) * 0.8;  // TODO review

  // Current values
  stance_width_ = stance_width_default_;
  leg_lift_height_ = leg_lift_height_default_;
}

/**
 * @brief Derived from physical dimensions and (expected) update frequency
 */
void Hexapod::setMovementIncrements() {
  const float time_step_duration_seconds = 1.0f / static_cast<float>(update_frequency_);

  // i.e. would raise from ground to walking height in ~1 second
  rising_increment_ = (walk_height_default_ - base_height_) * time_step_duration_seconds;

  // max leg movement on ground / time steps on ground (in ripple gait (and tripod) a foot is on the ground half the
  // time)
  walk_translation_max_ = allowed_foot_position_diameter_ * time_step_duration_seconds * 2.0f;
  // let's say 30 degrees per second for now
  walk_turn_max_ = (30.0f * static_cast<float>(M_PI) / 180.0f) * time_step_duration_seconds;

  // Movements applied every time step
  walk_translation_increment_ = walk_translation_max_ / static_cast<float>(walk_translation_num_increments_);
  walk_turn_increment_ = walk_turn_max_ / static_cast<float>(walk_turn_num_increments_);

  // Movements applied each button press
  body_translation_increment_ = (dims_.width / 20.0f);
  stance_width_increment_ = (dims_.width / 20.0f);
  leg_raise_increment_ = leg_lift_height_default_ / 10.0f;

  walk_translation_max_per_leg_step_ = walk_translation_increment_;
  // need to determine how much to reduce air time as speed level changes
  time_steps_per_speed_level_ = static_cast<float>(foot_air_time_default_ - foot_air_time_min_) /
                                static_cast<float>(walk_translation_num_increments_);
}

void Hexapod::printMovementParameters() {
  const float leg_length_full_extension = legs_[0].dims_.a + legs_[0].dims_.b + legs_[0].dims_.c;
  Vector3 neutral = legs_[0].getNeutralPosition();
  const Leg::MovementLimits lml = legs_[0].calculateMovementLimits(walk_height_default_);
#ifndef __AVR__
  std::cout << "update_frequency_\t" << update_frequency_ << '\n';
  std::cout << "body dimensions\t" << dims_.length << '\t' << dims_.width << '\t' << dims_.depth << '\n';
  std::cout << "leg neutral\t" << neutral.x() << '\t' << neutral.y() << '\t' << neutral.z() << '\n';
  std::cout << "leg_length_full_extension\t" << leg_length_full_extension << '\n';
  std::cout << "movement limits\t" << lml.x_max << '\t' << lml.x_min << '\t' << lml.y_max << '\t' << lml.y_min << '\n';
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
  std::cout << "walk_translation_max_\t" << walk_translation_max_ << '\n';
  std::cout << "walk_translation_num_increments_\t" << (int)walk_translation_num_increments_ << '\n';
  std::cout << "walk_translation_increment_\t" << walk_translation_increment_ << '\n';
  std::cout << "walk_turn_max_\t" << walk_turn_max_ << '\n';
  std::cout << "walk_turn_num_increments_\t" << (int)walk_turn_num_increments_ << '\n';
  std::cout << "walk_turn_increment_\t" << walk_turn_increment_ << '\n';
#else
//  Serial.print(F("update_frequency_\t"));Serial.println(update_frequency_);
//  Serial.print(F("body
//  dimensions\t"));Serial.print(dims_.length,4);Serial.print('\t');Serial.print(dims_.width,4);Serial.print('\t');Serial.println(dims_.depth,4);
//  Serial.print(F("leg
//  neutral\t"));Serial.print(neutral.x(),4);Serial.print('\t');Serial.print(neutral.y(),4);Serial.print('\t');Serial.println(neutral.z(),4);
//  Serial.print(F("leg_length_full_extension\t"));Serial.println(leg_length_full_extension,4);
//  Serial.print(F("movement
//  limits\t"));Serial.print(lml.x_max,4);Serial.print('\t');Serial.print(lml.x_min,4);Serial.print('\t');Serial.print(lml.y_max,4);Serial.print('\t');Serial.println(lml.y_min,4);
//  Serial.print(F("foot_air_time_default_\t"));Serial.println(foot_air_time_default_);
//  Serial.print(F("foot_air_time_min_\t"));Serial.println(foot_air_time_min_);
//  Serial.print(F("walk_height_default_\t"));Serial.println(walk_height_default_,4);
//  Serial.print(F("stance_width_default_\t"));Serial.println(stance_width_default_,4);
//  Serial.print(F("stance_width_min_\t"));Serial.println(stance_width_min_,4);
//  Serial.print(F("stance_width_max_\t"));Serial.println(stance_width_max_,4);
//  Serial.print(F("leg_lift_height_min_\t"));Serial.println(leg_lift_height_min_,4);
//  Serial.print(F("leg_lift_height_max_\t"));Serial.println(leg_lift_height_max_,4);
//  Serial.print(F("leg_lift_height_default_\t"));Serial.println(leg_lift_height_default_,4);
//  Serial.print(F("allowed_foot_position_diameter_\t"));Serial.println(allowed_foot_position_diameter_,4);
//  Serial.print(F("rising_increment_\t"));Serial.println(rising_increment_,4);
//  Serial.print(F("walk_translation_max_\t"));Serial.println(walk_translation_max_,4);
//  Serial.print(F("walk_translation_num_increments_\t"));Serial.println(walk_translation_num_increments_,4);
//  Serial.print(F("walk_translation_increment_\t"));Serial.println(walk_translation_increment_,4);
//  Serial.print(F("walk_turn_max_\t"));Serial.println(walk_turn_max_,4);
//  Serial.print(F("walk_turn_num_increments_\t"));Serial.println(walk_turn_num_increments_,4);
//  Serial.print(F("walk_turn_increment_\t"));Serial.println(walk_turn_increment_,4);
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

uint16_t Hexapod::getUpdateFrequency() const { return update_frequency_; }

int16_t Hexapod::getMovementNumIncrements() const { return walk_translation_num_increments_; }
float Hexapod::getWalkSpeedMax() const { return walk_translation_max_; }
float Hexapod::getTurnSpeedMax() const { return walk_turn_max_; }

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
      const Vector3 foot_in_base = tf_base_to_body_ * tf_body_to_leg_[leg_idx] * legs_[leg_idx].getFootPosition();
      // position of foot in the updated leg frame (base on walk and body movement)
      const Vector3 leg_to_foot_new = (tf_base_to_new_body * tf_body_to_leg_[leg_idx]).inverse() * foot_in_base;
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
  // So we actually need to do this first make sure all raised legs that have finished have their
  // status update before
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
  for (uint8_t i = 0; i < num_legs_; ++i, seq_no = (seq_no + 1) % num_legs_) {
    uint8_t prev_seq_no = (seq_no + num_legs_ - 1) % num_legs_;
    uint8_t prev_leg_idx = gaits_[current_gait_seq_].order[prev_seq_no];
    uint8_t leg_idx = gaits_[current_gait_seq_].order[seq_no];

    // if the previous leg has finished i.e. is grounded, then can raise the new one
    // Use the offset value to determine when during the the previous leg's trajectory the next one
    // can start lifting (offset = 0 -> straight away, offset 1 -> only when back on ground)
    bool prev_leg_complete =
        legs_[prev_leg_idx].getCurrentStepProgress() >= gaits_[current_gait_seq_].offset[prev_seq_no];

    if (base_change_ && prev_leg_complete && legs_[leg_idx].state_ == Leg::State::ON_GROUND) {
      updateFootTarget(leg_idx);  // TODO I think this can (and should) be removed because it's already called
                                  // for
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
      break;  // stop at the first non-raise result, there can't be any more
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
 * so we just add it, but need to modify using tf_base_to_new_base_target_ in case there's a change
 * pending.
 *
 * The neutral position does not change with the body, so we only need to apply the body_to_leg
 * transform to get the neutral position in the base frame i.e. as if the base to body transform is
 * identity
 *
 * @param leg_idx
 * @return Vector3
 */
Vector3 Hexapod::getNeutralPosition(const uint8_t leg_idx) const {
  Vector3 leg_neutral = legs_[leg_idx].getNeutralPosition();  // TODO this is actually callng the non-const version
                                                              // and returning a modifyable ref
  leg_neutral.z() = -base_height_ + tf_base_to_new_base_target_.t_.z();
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
  const float speed = combined_step.norm();
  Vector3 direction;
  if (compareFloat(speed, 0.0f, 0.0001f)) {
    direction = Vector3(0.0f, 0.0f, 0.0f);  // not actually a unit vector obviously
  } else {
    direction = combined_step.unit();
  }

  Vector3 raised_pos;
  Vector3 target_pos;
  // if the leg has already been lifted, use the already calculated targets
  //    we just need to modify them to account to changes in body position
  if (legs_[leg_idx].state_ == Leg::State::RAISED && legs_[leg_idx].getStepIdx() > 0) {
    // perhaps move this calculation 'higher' to avoid repeating (although will only repeat for
    // raised legs)
    const Transform tf_update =
        (tf_base_to_body_target_ * tf_body_to_leg_[leg_idx]).inverse() * tf_base_to_body_ * tf_body_to_leg_[leg_idx];
    target_pos = tf_update * legs_[leg_idx].getTargetPosition();
    raised_pos = tf_update * legs_[leg_idx].getRaisedPosition();
    // also need to update the current position to account for the body change
    // I think I need to re-think the whole leg raise movement approach really
    // because it could probably be a lot simpler
    const Vector3 upd_current_pos = tf_update * legs_[leg_idx].getFootPosition();
    Leg::JointAngles angles;
    if (legs_[leg_idx].calculateJointAngles(upd_current_pos, Leg::IKMode::WALK, angles)) {
      legs_[leg_idx].setStagedAngles(angles);  // (P)REFACTOR should I really be doing this here?
    }
  }
  // if the leg is only just about to become raised then need to calculate targets for first time
  // also do this if the foot is on the ground i.e. target update doesn't relate to a step
  //  (this allows the leg to respond to changes in e.g. stance width)
  else {
    const Vector3 neutral_pos = getNeutralPosition(leg_idx);

    // Target is ahead of neutral along step unit direction
    // How far do we expect this leg to travel on the ground?
    // distance per time step * num time steps a leg is in the air * num legs that will be in the air while this is on
    // the ground
    const float expected_stride_length_at_current_speed = speed * static_cast<float>(getLegGroundedTime(leg_idx));
    const Vector3 target_pos_in_base = neutral_pos + (expected_stride_length_at_current_speed / 2.0f) * direction;

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

bool Hexapod::setWalk(const BaseMovementLevels& speeds_requested, bool force) {
#ifndef __AVR__
  using namespace std;  // for min
#endif
  // don't set if not in walking state, or if trying to leave walking state
  if (force || (state_ == State::WALKING && requested_state_ == State::WALKING)) {
    // Clamp to max speed level
    //    speeds_requested_.t.x = (speeds_requested_.t.x >= 0 ? 1 : -1) * min(abs(speeds_requested.t.x),
    //    walk_translation_num_increments_); speeds_requested_.t.y = (speeds_requested_.t.y >= 0 ? 1 : -1) *
    //    min(abs(speeds_requested.t.y), walk_translation_num_increments_); speeds_requested_.r = (speeds_requested_.r
    //    >= 0 ? 1 : -1) * min(abs(speeds_requested.r), walk_turn_num_increments_);
    if (speeds_requested_.t.x > walk_translation_num_increments_) {
      speeds_requested_.t.x = walk_translation_num_increments_;
    } else if (speeds_requested_.t.x < -walk_translation_num_increments_) {
      speeds_requested_.t.x = -walk_translation_num_increments_;
    } else {
      speeds_requested_.t.x = speeds_requested.t.x;
    }
    if (speeds_requested_.t.y > walk_translation_num_increments_) {
      speeds_requested_.t.y = walk_translation_num_increments_;
    } else if (speeds_requested_.t.y < -walk_translation_num_increments_) {
      speeds_requested_.t.y = -walk_translation_num_increments_;
    } else {
      speeds_requested_.t.y = speeds_requested.t.y;
    }
    if (speeds_requested_.r > walk_turn_num_increments_) {
      speeds_requested_.r = walk_turn_num_increments_;
    } else if (speeds_requested_.r < -walk_turn_num_increments_) {
      speeds_requested_.r = -walk_turn_num_increments_;
    } else {
      speeds_requested_.r = speeds_requested.r;
    }
    return true;
  }
  return false;
}

bool Hexapod::setWalk(const BaseTranslationLevels& translation_speed_levels, const int16_t rotation_speed_level) {
  return setWalk(BaseMovementLevels{translation_speed_levels, rotation_speed_level});
}

bool Hexapod::setWalk(const BaseTranslationLevels& translation_speed_levels) {
  return setWalk(translation_speed_levels, 0);
}

bool Hexapod::setWalk(const int16_t rotation_speed_level) {
  return setWalk(BaseTranslationLevels{0, 0}, rotation_speed_level);
}

bool Hexapod::changeWalk(const BaseMovementLevels& speed_level_changes) {
  return setWalk(speeds_current_ + speed_level_changes);
}

bool Hexapod::changeWalk(const BaseTranslationLevels& translation_speed_level_changes,
                         const int16_t rotation_speed_level_changes) {
  return changeWalk(BaseMovementLevels{translation_speed_level_changes, rotation_speed_level_changes});
}

bool Hexapod::changeWalk(const BaseTranslationLevels& translation_speed_level_changes) {
  return changeWalk(translation_speed_level_changes, 0);
}

bool Hexapod::changeWalk(const int16_t rotation_speed_level_change) {
  return changeWalk(BaseTranslationLevels{0, 0}, rotation_speed_level_change);
}

Hexapod::BaseMovementLevels Hexapod::getWalk() { return speeds_current_; }

bool Hexapod::increaseWalkForward() { return changeWalk(BaseTranslationLevels{1, 0}); }
bool Hexapod::decreaseWalkForward() { return changeWalk(BaseTranslationLevels{-1, 0}); }
bool Hexapod::increaseWalkLeft() { return changeWalk(BaseTranslationLevels{0, 1}); }
bool Hexapod::decreaseWalkLeft() { return changeWalk(BaseTranslationLevels{0, -1}); }
bool Hexapod::increaseRotationCCW() { return changeWalk(1); }
bool Hexapod::decreaseRotationCCW() { return changeWalk(-1); }
bool Hexapod::setWalkForward(const uint16_t speed_level) {
  auto modified_request = speeds_requested_;
  modified_request.t.x = speed_level;
  return setWalk(modified_request);
}
bool Hexapod::setWalkLeft(const uint16_t speed_level) {
  auto modified_request = speeds_requested_;
  modified_request.t.y = speed_level;
  return setWalk(modified_request);
}
bool Hexapod::setRotationCCW(const uint16_t speed_level) {
  auto modified_request = speeds_requested_;
  modified_request.r = speed_level;
  return setWalk(modified_request);
}

bool Hexapod::setWalkingTargets() {
#ifndef __AVR__
  using namespace std;  // for min, sin, cos
#endif
  if (state_ != State::WALKING) {
    return false;
  }

  // TODO having to handle headless mode is a bit annoying in my nice new 'speed level' set up
  // maybe we do the headless transformation later, and when checking speed changes we just assume that
  //  it won't rotate sufficiently fast to cause errors in speed change throttling
  //  if (move_mode_ == MoveMode::HEADLESS) {
  //    float x =
  //        cos(-total_base_rotation_) * walk_step_requested_.x() - sin(-total_base_rotation_) *
  //        walk_step_requested_.y();
  //    float y =
  //        sin(-total_base_rotation_) * walk_step_requested_.x() + cos(-total_base_rotation_) *
  //        walk_step_requested_.y();
  //    walk_step_requested_ = Vector3(x, y, 0);
  //  }

  // Now limit the requests if necessary
  for (uint8_t leg_idx = 0; leg_idx < num_legs_; ++leg_idx) {
    if (legs_[leg_idx].state_ == Leg::State::ON_GROUND && legs_[leg_idx].prev_state_ == Leg::State::RAISED) {
      // a leg has just finished its step, so reset the accumulated motion limit
      //      walk_step_applied_this_leg_step_ = {0.0, 0.0, 0.0};
      speed_change_applied_so_far_this_leg_step_ = {{0, 0}, 0};
      break;
    }
  }

  const auto allowed_change_abs_x = speed_change_max_each_leg_step_ - speed_change_applied_so_far_this_leg_step_.t.x;
  const auto requested_change_x = speeds_requested_.t.x - speeds_current_.t.x;
  const auto change_abs_x = min(allowed_change_abs_x, abs(requested_change_x));
  const auto change_x = requested_change_x >= 0 ? change_abs_x : -change_abs_x;
  speed_change_applied_so_far_this_leg_step_.t.x += change_x;
  // surely this can be a bit nicer??

  const auto allowed_change_abs_y = speed_change_max_each_leg_step_ - speed_change_applied_so_far_this_leg_step_.t.y;
  const auto requested_change_y = speeds_requested_.t.y - speeds_current_.t.y;
  const auto change_abs_y = min(allowed_change_abs_y, abs(requested_change_y));
  const auto change_y = requested_change_y >= 0 ? change_abs_y : -change_abs_y;
  speed_change_applied_so_far_this_leg_step_.t.y += change_y;

  speeds_target_.t.x = speeds_current_.t.x + change_x;
  speeds_target_.t.y = speeds_current_.t.y + change_y;
  // We're not restricting change in turning speed currently
  speeds_target_.r = speeds_requested_.r;

  // also adjust the step time
  if (speeds_target_.t.isZero()) {
    setLegRaiseTime(foot_air_time_default_);
  } else {
    // rough combined measure
    // or convert to float and calculate accurate combined value sqrt(x^2+y^2)
    const auto combined_xy_speed_level = abs(speeds_target_.t.x) + abs(speeds_target_.t.y);
    auto leg_raise_reduction =
        static_cast<uint16_t>(static_cast<float>(combined_xy_speed_level) * time_steps_per_speed_level_);
    // round down to multiple of 2
    leg_raise_reduction = (leg_raise_reduction / 2) * 2;
    //    setLegRaiseTime(foot_air_time_default_ - leg_raise_reduction); // TODO temporarily disable
  }

  // now we need to convert the speed levels to usable measurements

  float target_x = static_cast<float>(speeds_target_.t.x) * walk_translation_increment_;
  float target_y = static_cast<float>(speeds_target_.t.y) * walk_translation_increment_;
  if (move_mode_ == MoveMode::HEADLESS) {
    target_x = cos(-total_base_rotation_) * target_x - sin(-total_base_rotation_) * target_y;
    target_y = sin(-total_base_rotation_) * target_x + cos(-total_base_rotation_) * target_y;
  }

  tf_base_to_new_base_target_.R_.setRPYExtr(0.0f, 0.0f, static_cast<float>(speeds_target_.r) * walk_turn_increment_);
  tf_base_to_new_base_target_.t_.x() = target_x;
  tf_base_to_new_base_target_.t_.y() = target_y;
  base_change_ = true;

  // If there's a change, need to update raised feet target, unless now stopped in which case update
  // all to allow feet to return to neutral position
  if (speeds_current_ != speeds_target_) {
    if (speeds_target_.isZero()) {
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
    tf_base_to_body_target =
        headless_correction.inverse() * tf_base_to_body_change * headless_correction * tf_base_to_body_;
  } else {
    tf_base_to_body_target = tf_base_to_body_change * tf_base_to_body_;
  }
  return setBody(tf_base_to_body_target);
}

void Hexapod::clearWalk() {
  // Clear all the movement variables to prevent changes being throttled
  speeds_current_ = {{0, 0}, 0};
  speeds_target_ = {{0, 0}, 0};
  speeds_requested_ = {{0, 0}, 0};
  speed_change_applied_so_far_this_leg_step_ = {{0, 0}, 0};
}

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
      clearTargets();  // couldn't achieve the desired movement
    }
    updateRaisedFootTargets();  // Update foot targets if required
    raised_legs_result = handleRaisedLegs();

  } else {
    // state_ == State::FULL_MANUAL
    // The legs have already been modified directly though the manualMoveFoot and manualChangeJoint
    // functions
  }

  if (grounded_legs_result && raised_legs_result) {
    commitTargets();
  }
  // only check the raise leg results because if the grounded legs couldn't be updated, they just
  // won't change
  //  and we can keep moving the raised legs even if we couldn't move the grounded ones
  if (raised_legs_result) {
    commitLegJointChanges();
    if (state_ == State::WALKING) {
      updateLegsStatus();  // Allow them (based on conditions) to change state between ON_GROUND and
                           // RAISED
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

bool Hexapod::increaseStanceWidth() { return changeStanceWidth(stance_width_increment_); }
bool Hexapod::decreaseStanceWidth() { return changeStanceWidth(-stance_width_increment_); }

/**
 * @details
 *
 * @return true if the value was changed
 */
bool Hexapod::resetStanceWidth() { return setStanceWidth(stance_width_default_); }

/**
 * @details
 * This does not do anything special when changing between single and multileg gaits. For mulit ->
 * single it works naturally anyway. For single -> multi it looks very slightly odd but won't lift
 * more than the defined in the multi-leg gait anyway.
 *
 * @param gait
 * @return true if the gait was changed
 */
bool Hexapod::changeGait(const Gait new_gait) {
  if (new_gait == current_gait_seq_ || new_gait >= Gait::NUM_GAITS) return false;

  // if there's currently a leg raised, we should set the next leg to the one after that in the new
  // gait order can do if nothing raised either, doesn't really matter
  uint8_t current_gait_current_leg_seq_no = (gait_next_leg_seq_no_ + num_legs_ - 1) % num_legs_;  // decrement with wrap
  uint8_t current_leg_idx = gaits_[current_gait_seq_].order[current_gait_current_leg_seq_no];
  // find this leg in the new gait
  for (uint8_t seq_no = 0; seq_no < num_legs_; ++seq_no) {
    if (gaits_[new_gait].order[seq_no] == current_leg_idx) {
      gait_next_leg_seq_no_ = seq_no;  // current leg
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

bool Hexapod::increaseFootGroundTravelRatio() { return changeFootGroundTravelRatio(ftgr_increment_); }

bool Hexapod::decreaseFootGroundTravelRatio() { return changeFootGroundTravelRatio(-ftgr_increment_); }

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

bool Hexapod::increaseLegRaiseHeight() { return changeLegRaiseHeight(leg_raise_increment_); }

bool Hexapod::decreaseLegRaiseHeight() { return changeLegRaiseHeight(-leg_raise_increment_); }

/**
 * @details
 *
 * @return true if the value was changed
 */
bool Hexapod::resetLegRaiseHeight() { return setLegRaiseHeight(leg_lift_height_default_); }

uint16_t Hexapod::getLegRaiseTime() { return foot_air_time_; }

bool Hexapod::setLegRaiseTime(uint16_t time) {
  if (time < foot_air_time_min_)
    time = foot_air_time_min_;
  else if (time > foot_air_time_max_)
    time = foot_air_time_max_;
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
  } else {
    return false;
  }
}

bool Hexapod::increaseLegRaiseTime() { return changeLegRaiseTime(leg_raise_time_increment_); }
bool Hexapod::decreaseLegRaiseTime() { return changeLegRaiseTime(-leg_raise_time_increment_); }

bool Hexapod::resetLegRaiseTime() { return setLegRaiseTime(foot_air_time_default_); }

uint16_t Hexapod::getLegGroundedTime(const uint8_t leg_idx) {
  // estimated time that a leg will be on the ground
  // function of gait offset, num legs and foot_air_time
  const float cycle_time = static_cast<float>(foot_air_time_ * num_legs_) * gaits_[current_gait_seq_].offset[leg_idx];
  return static_cast<uint16_t>(cycle_time) - foot_air_time_;
}

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
  const Leg::JointAngles midpoint{current_angles.theta_1 + joint_increments.theta_1 * static_cast<float>(duration / 2),
                                  current_angles.theta_2 + joint_increments.theta_2 * static_cast<float>(duration / 2),
                                  current_angles.theta_3 + joint_increments.theta_3 * static_cast<float>(duration / 2)};

  legs_[leg_idx].setTrajectory(joint_targets, joint_increments, midpoint, joint_increments, duration);
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
 * Even if the target angles cannot be set for some of the legs, they will still be set for the
 * others.
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

bool Hexapod::setAllLegTargetsToGround() { return setAllLegTargetsToGround(update_frequency_); }

/**
 * @details Doesn't care whether the other legs are achievable or not. Will only consider this leg
 * in isolation.
 * @param leg_idx
 * @param duration
 * @return
 */
bool Hexapod::setLegTargetToGround(const uint8_t leg_idx, const uint16_t duration) {
  if (state_ != State::UNSUPPORTED) {
    return false;
  }

  const Leg::MovementLimits lml = legs_[0].calculateMovementLimits(base_height_);
  // There is a risk that if this point is far enough from the 'normal' neutral position that the
  // hexapod cannot reach
  //  the desired standing height while the feet are places here (and would need to adjust before
  //  moving further up)
  const float new_x = lml.x_min + (lml.x_max - lml.x_min) * 0.45f;
  // Remember this is in leg frame, so actual floor is slightly below
  Vector3 grounded_position{new_x, 0.0, -base_height_};

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
      const float floor_height = -base_height_;
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
  if (state_ == State::RAISING && requested_state_ == State::WALKING && base_height_ >= walk_height_default_) {
    state_ = requested_state_;
#ifdef __AVR__
    Serial.print(F("State: WALKING\n"));
#else
    std::cout << "State changed to: WALKING\n";
#endif
  }

  if (state_ == State::WALKING && requested_state_ == State::LOWERING) {
    setWalk(BaseTranslationLevels{0, 0}, true);
    setBody(Transform(), true);  // ideally do more smoothly
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

  if (state_ == State::LOWERING && requested_state_ == State::STANDING && base_height_ <= dims_.depth / 2.0f) {
    state_ = requested_state_;
#ifdef __AVR__
    Serial.print(F("State: STANDING\n"));
#else
    std::cout << "State changed to: STANDING\n";
#endif
  }

  // immediately transition - stop all other movement
  // note that all other state changes so far were triggered from internal changes - this one will
  // be external
  if (state_ != State::FULL_MANUAL && requested_state_ == State::FULL_MANUAL) {
    state_ = State::FULL_MANUAL;
  }
  // TODO need sensible way to go back
}

Hexapod::State Hexapod::getState() const { return state_; }

const Leg& Hexapod::getLeg(const uint8_t leg_idx) const { return legs_[leg_idx]; }

const Transform& Hexapod::getBaseToBody() const { return tf_base_to_body_; }

const Transform& Hexapod::getBaseMovement() const { return tf_base_movement_; }

float Hexapod::getHeight() const { return base_height_ + tf_base_to_body_.t_.z(); }

const Transform Hexapod::getBaseToLeg(const uint8_t leg_idx) { return tf_base_to_body_ * tf_body_to_leg_[leg_idx]; }

const Leg::MovementLimits& Hexapod::getMovementLimits(uint8_t leg_idx) {
  return legs_[leg_idx].movement_limits_grounded_;
}

void Hexapod::setFullManualControl(const bool control_on) {
  if (state_ != State::FULL_MANUAL && control_on) {
    requested_state_ = State::FULL_MANUAL;
    setManualLegControl(0);  // default
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
  if (state_ == State::FULL_MANUAL && manual_control_type_ == ManualControlType::SINGLE_LEG) {
    const Vector3 new_pos = legs_[manual_leg_idx_].getFootPosition() + movement;
    Leg::JointAngles angles;
    const bool ik_result = legs_[manual_leg_idx_].calculateJointAngles(new_pos, Leg::IKMode::FULL, angles);
    if (ik_result) {
      legs_[manual_leg_idx_].setStagedAngles(angles);
    }
  }
}

void Hexapod::manualChangeJoint(const float angle_change) {
  if (state_ == State::FULL_MANUAL && manual_control_type_ == ManualControlType::SINGLE_JOINT) {
    // TODO this is very awkward/awful! Maybe some refactoring required (make JointAngles an
    // indexable array)
    const Joint joint = legs_[manual_leg_idx_].joints_[manual_joint_idx_];
    const float new_angle = joint.clampToLimits(joint.angle_ + angle_change);
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

Hexapod::ManualControlType Hexapod::getManualControlType() const { return manual_control_type_; }

uint8_t Hexapod::getManualControlLegIdx() const { return manual_leg_idx_; }

uint8_t Hexapod::getManualControlJointIdx() const { return manual_joint_idx_; }

uint8_t Hexapod::gaitNextLeg() { return gaits_[current_gait_seq_].order[gait_next_leg_seq_no_]; }
// Check this still works for default gait
// Add back the exist gaits
// Add check on progress of previous leg, and implement use of offset
// Add tripod gait

uint8_t Hexapod::gaitMaxRaised() { return gaits_[current_gait_seq_].max_raised; }

void Hexapod::commitTargets() {
  speeds_current_ = speeds_target_;
  if (move_mode_ == MoveMode::HEADLESS) {
    total_base_rotation_ += static_cast<float>(speeds_target_.r) * walk_turn_increment_;
  }
  if (base_change_) {
    tf_base_movement_ = tf_base_to_new_base_target_;
    base_height_ +=
        tf_base_to_new_base_target_.t_.z();  // (P)REFACTOR I think that only getNeutralPosition will be affected
  }
  if (body_change_) {
    const bool body_change_z = tf_base_to_body_target_.t_.z() - tf_base_to_body_.t_.z() != 0.0;
    tf_base_to_body_ = tf_base_to_body_target_;
    // TODO *** this may not be suitable for arduino version ***
    if (body_change_z) {
      for (uint8_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
        legs_[leg_idx].updateMovementLimits(getHeight(), getHeight() - leg_lift_height_default_);
      }
    }
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
    leg = leg % num_legs_;  // redundant
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

}  // namespace hexapod