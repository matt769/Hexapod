#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "leg.h"
#include "transformations.h"

#include <stddef.h>
#include <cmath>

namespace Tfm = Transformations;

class Hexapod {
 public:
  static constexpr float length_ = 1.8f;
  static constexpr float width_ = 1.0f;
  static constexpr float depth_ = 0.2f;
  static constexpr size_t num_legs_ = 6;
  enum { FRONT_LEFT = 0, FRONT_RIGHT, MIDDLE_LEFT, MIDDLE_RIGHT, BACK_LEFT, BACK_RIGHT };
  enum class MoveMode { STANDARD, HEADLESS };
  enum class State { UNSUPPORTED, STANDING, WALKING };
  enum class SubState { NOT_STARTED, IN_PROGRESS, FINISHED };
  enum Gait { RIPPLE = 0, LEFT_RIGHT_LEFT_RIGHT = 1, LHS_THEN_RHS = 2, AROUND_THE_CLOCK = 3 };
  static constexpr float walk_height_default_ = 0.4;
  struct GaitSequence {
    size_t order[num_legs_];  // index corresponds to leg index, value corresponds to order
    size_t max_raised;
    size_t current_position;
  };
  static constexpr size_t num_gaits_ = 4;
  GaitSequence gait_seq_[num_gaits_] = {{{0, 3, 4, 1, 2, 5}, 1, 0},
                                        {{0, 1, 2, 3, 4, 5}, 1, 0},
                                        {{0, 2, 4, 1, 3, 5}, 1, 0},
                                        {{0, 2, 4, 5, 3, 1}, 1, 0}};
  static constexpr size_t foot_air_time_default_ = 20;
  static constexpr size_t foot_air_time_min_ = 6;
  static constexpr float stance_width_min_ = 0.2f;  // TODO REVIEW - maybe this should come from Leg
  static constexpr float stance_width_max_ = 0.9f;  // TODO REVIEW
  static constexpr float stance_width_default_ = 0.6f;
  static constexpr float leg_lift_height_min_ = 0.03f;
  static constexpr float leg_lift_height_max_ = 0.3f;
  static constexpr float leg_lift_height_default_ = 0.05f;
  static constexpr float fgtr_min_ = 0.1f;
  static constexpr float fgtr_max_ = 0.9f;
  static constexpr float fgtr_default_ = 0.5f;



  Hexapod();
  bool setWalk(const Tfm::Vector3& walk_step, float angle_step);
  bool setWalk(const Tfm::Vector3& walk_step);
  bool setWalk(float angle_step);
  bool setBody(const Tfm::Transform& tf_base_to_body_target);
  bool changeBody(const Tfm::Transform& tf_base_to_body_change);
  bool clearMovement();
  bool resetBody();
  bool update();  // main update function to be called every iteration
  bool setStanceWidth(float stance_width);
  bool changeStanceWidth(float change);
  bool resetStanceWidth();
  bool changeGait(Gait gait);
  bool setFootGroundTravelRatio(float ratio);
  bool changeFootGroundTravelRatio(float change);
  bool resetFootGroundTravelRatio();
  bool setLegRaiseHeight(float height);
  bool changeLegRaiseHeight(float change);
  bool resetLegRaiseHeight();
  void setMoveMode(MoveMode move_mode);
  bool setStartingPosition(Leg::JointAngles starting_angles);
  bool setLegsToGround();
  bool riseToWalk();
  State getState() const;
  //for visualisation
  const Leg& getLeg(size_t leg_idx) const;
  const Tfm::Transform& getBaseToBody() const;
  const Tfm::Transform& getBaseMovement() const;
  float getHeight() const;

 private:
  Leg legs_[num_legs_]; 
  float height_ = depth_ / 2.0f;  // height of base frame above ground
  Tfm::Transform tf_base_to_body_, tf_base_to_body_prev_;  // relationship between base link and body
  Tfm::Transform tf_body_to_leg_[num_legs_];  // relationship between body link and leg base
  MoveMode move_mode_ = MoveMode::STANDARD;
  State state_ = State::UNSUPPORTED;
  State requested_state_ = State::UNSUPPORTED;
  SubState sub_state_ = SubState::NOT_STARTED;
  Tfm::Transform
      tf_base_movement_;  // last movement of the base frame (new base frame in previous base
                          // frame) - for visualisation
  Tfm::Transform tf_base_to_new_base_;  // target walk movement
  Tfm::Transform tf_base_to_body_target_;
  Leg::JointAngles staged_angles_[6];  // TODO probably move to Leg class
  Tfm::Vector3 walk_step_current_, walk_step_new_;
  float turn_step_current_, turn_step_new_;
  bool body_change_ = false;
  bool base_change_ = false;
  bool recalculate_all_feet_targets_ = false;
  bool recalculate_raised_feet_targets_ = false;
  // for unsupported movement of all legs together
  Leg::JointAngles joint_targets_;
  Leg::JointAngles joint_increments_;
  size_t total_movement_steps_, current_movement_step_;  

  // TODO review this - rather hardcoded atm
  // defines a circle around point at x=0.6, y=0
  // are the min/max even used atm? No
  // Only using dia - but not modifying if neutral point has changed (via stance width)
  struct AllowedFootPosition {
    float x_min = 0.4f;
    float y_min = -0.2f;
    float dia = 0.4f;
  } allowed_foot_position_;

  size_t current_gait_seq_ = Gait::RIPPLE;

  float stance_width_ = stance_width_default_;
  float leg_lift_height_ = leg_lift_height_default_;
  float foot_ground_travel_ratio_ =
      fgtr_default_;  // What proportion of the maximum foot travel distance
                      // will we actually get the foot to (relatively) travel
                      // while on ground during walk
  float total_base_rotation_ = 0.0f;  

  bool calculateGroundedLegs();
  void applyChangesGroundedLegs();
  size_t getNumLegsRaised() const;
  bool handleGroundedLegs();
  Tfm::Vector3 calculateFootVector(size_t leg_idx) const;
  void handleRaisedLegs();
  void updateLegs();
  Tfm::Vector3 getNeutralPosition(size_t leg_idx) const;
  void updateFootTarget(size_t leg_idx);
  void updateFootTargets();
  Tfm::Vector3 getFootPosition(size_t leg_idx) const;
  void clearTargets();
  bool setTargetsMoveLegs(Leg::JointAngles joint_targets);  // set targets to move all legs to
                                                            // specified joint positions when in
                                                            // UNSUPPORTED state
  bool updateMoveLegs();  // corresponding update function for movements in UNSUPPORTED state
  bool changeBase(Tfm::Vector3 move_base);
  void handleStateChange();
  void clearVisualisationChanges();

};

#endif
