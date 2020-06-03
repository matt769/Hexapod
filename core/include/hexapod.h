#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "leg.h"
#include "transformations.h"

#include <stddef.h>
#include <cmath>

namespace Tfm = Transformations;

class Hexapod {
 public:
  struct Dims {
    float length;
    float width;
    float depth;
  };
  Dims dims_;
  size_t num_legs_;
  /** @brief Heading moves with body frame (Standard) or stays fixed (Headless) */
  enum class MoveMode { STANDARD, HEADLESS };
  /** @brief Walking is main state, others are used in start up routine */
  enum class State { UNSUPPORTED, STANDING, WALKING };
  /** @brief Gait identifier. Also used as index into gait_seq_ */
  enum Gait { RIPPLE = 0, LEFT_RIGHT_LEFT_RIGHT, LHS_THEN_RHS, AROUND_THE_CLOCK, NUM_GAITS };
  /** @brief During start up, hexapod will rise to this height before entering walking state */
  static constexpr float walk_height_default_ = 0.4;
  /** @brief Number of periods leg will be in air if raised while walk speed is zero */
  static constexpr size_t foot_air_time_default_ = 20;
  /** @brief Minimum time foot can be in air during a raise i.e. fastest raise movement */
  static constexpr size_t foot_air_time_min_ = 6;
  static constexpr float stance_width_min_ = 0.2f;  // TODO REVIEW - maybe this should come from Leg
  static constexpr float stance_width_max_ = 0.9f;  // TODO REVIEW
  static constexpr float stance_width_default_ = 0.6f;
  static constexpr float leg_lift_height_min_ = 0.03f;
  static constexpr float leg_lift_height_max_ = 0.3f;
  static constexpr float leg_lift_height_default_ = 0.05f;
  static constexpr float fgtr_min_ = 0.1f;
  static constexpr float fgtr_max_ = 0.9f;
  /** @brief Default stride length as proportion of allowed foot movement */
  static constexpr float fgtr_default_ = 0.5f;

  Hexapod(size_t num_legs, Dims hex_dims, Tfm::Transform* tf_body_to_leg, Leg* legs);
  ~Hexapod();
  Hexapod(const Hexapod&) = delete;
  Hexapod(Hexapod&&) = default;
  Hexapod& operator=(const Hexapod&) = delete;
  Hexapod& operator=(Hexapod&&) = default;
  /** @brief Set walk or turn movement for the next period */
  bool setWalk(const Tfm::Vector3& walk_step, float angle_step);
  /** @brief Set walk or turn movement for the next period */
  bool setWalk(const Tfm::Vector3& walk_step);
  /** @brief Set walk or turn movement for the next period */
  bool setWalk(float angle_step);
  /** @brief Set absolute body rotation and translation relative to the base */
  bool setBody(const Tfm::Transform& tf_base_to_body_target);
  /** @brief Apply incremental rotation and translation to the body, relative to the base */
  bool changeBody(const Tfm::Transform& tf_base_to_body_change);
  /** @brief Resets any walk or turn commands. Does not affect body rotation or translation. */
  bool clearMovement();
  /** @brief Resets any body tilt or translation. Does not affect walk or turn. */
  bool resetBody();
  /** @brief main update function to be called every period. Will apply any movement currently set. */
  bool update();
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
  /** @brief From unsupported state set feet targets to the ground. */
  bool setLegsToGround();
  /** @brief Set the base to move upwards until walk_height_default_ reached */
  bool riseToWalk();
  State getState() const;
  /** @brief For visualisation */
  const Leg& getLeg(size_t leg_idx) const;
  /** @brief For visualisation */
  const Tfm::Transform& getBaseToBody() const;
  /** @brief For visualisation */
  const Tfm::Transform& getBaseMovement() const;
  /** @brief For visualisation */
  float getHeight() const;

 private:
   /** @brief Used to control movements while in unsupported state */
  enum class SubState { NOT_STARTED, IN_PROGRESS, FINISHED };
  Leg* legs_; 
  /** @brief Height of base frame above ground */
  float height_;
  /** @brief Relationship between base frame and body frame */
  Tfm::Transform tf_base_to_body_, tf_base_to_body_prev_;  // 
  /** @brief Fixed relationship between body frame and leg frames */
  Tfm::Transform* tf_body_to_leg_;
  MoveMode move_mode_ = MoveMode::STANDARD;
  State state_ = State::UNSUPPORTED;
  State requested_state_ = State::UNSUPPORTED;
  SubState sub_state_ = SubState::NOT_STARTED;
  /** @brief For visualisation. New base expressed in old base frame. */
  Tfm::Transform tf_base_movement_;
  /** @brief Target walk movement expressed in current base frame */
  Tfm::Transform tf_base_to_new_base_;
  /** @brief Target base to body transform expressed in current base frame */
  Tfm::Transform tf_base_to_body_target_;
  /** @brief Hold calculated angles for all legs before applying them.
   * TODO - probably move to leg class
   */
  Leg::JointAngles staged_angles_[6];
  /** @brief Current walk vector. TODO review naming. */
  Tfm::Vector3 walk_step_current_;
  /** @brief Requested walk vector. */
  Tfm::Vector3  walk_step_new_;
  /** @brief Current walk vector. TODO review naming. */
  float turn_step_current_;
  /** @brief Requested turn angle. */
  float turn_step_new_;
  /** @brief Flag to indicate there's been a change in base to body */
  bool body_change_ = false;
  /** @brief Flag to indicate there's been a change in base to 'new' base' */
  bool base_change_ = false;
  /** @brief Flag to recalculate targets for all feet. */
  bool recalculate_all_feet_targets_ = false;
  /** @brief Flag to recalculate targets for any raised feet. */
  bool recalculate_raised_feet_targets_ = false;
  /** @brief Joint targets when in unsupported state. */
  Leg::JointAngles joint_targets_;
  /** @brief Joint movement increments when in unsupported state. */
  Leg::JointAngles joint_increments_;
  /** @brief Number periods over which unsupported movements. */
  size_t total_movement_steps_;
  /** @brief Current progress of unsupported movements. */
  size_t current_movement_step_;  

  /**
   * @brief Describes a circle of allowed foot movement around the neutral point.
   * 
   * @details
   * TODO review this. Don't think min and max are even used.
   * Ideally have better solution to managing range of movement.
   * Only using dia - but not modifying if neutral point has changed (via stance width)
   */
  struct AllowedFootPosition {
    float x_min = 0.4f;
    float y_min = -0.2f;
    float dia = 0.4f;
  } allowed_foot_position_;

  size_t current_gait_seq_ = Gait::RIPPLE;
  size_t gait_current_pos_ = 0;
  float stance_width_ = stance_width_default_;
  float leg_lift_height_ = leg_lift_height_default_;
  float foot_ground_travel_ratio_ = fgtr_default_;
  float total_base_rotation_ = 0.0f;  

  /** @brief Calculate the required joint angles for all grounded legs */
  bool calculateGroundedLegs();
  /** @brief Applies the pre-calculated joint angles for all grounded legs. */
  void applyChangesGroundedLegs();
  size_t getNumLegsRaised() const;
  /** @brief Includes everything necessary to manage the grounded legs. */
  bool handleGroundedLegs();
  /** @brief Calculates vector induced at neutral position due to movement. */
  Tfm::Vector3 calculateFootVector(size_t leg_idx) const;
  /** @brief Includes everything necessary to manage the raised legs. */
  void handleRaisedLegs();
  /** @brief Update status of each leg and request they raise if conditions met. */
  void updateLegs();
  /** @brief Return neutral position for the leg in the base frame. */
  Tfm::Vector3 getNeutralPosition(size_t leg_idx) const;
  /** @brief Update foot targets (if required)*/
  void updateFootTarget(size_t leg_idx);
  void updateFootTargets();
  /** @brief Return foot position in the base frame. */
  Tfm::Vector3 getFootPosition(size_t leg_idx) const;
    /** @brief Clear all movement targets (walk, turn, body etc). */
  void clearTargets();
    /** @brief Set robot-wide targets to move all legs to specified joint position
     * when in unsupported state.
    */
  bool setTargetsMoveLegs(Leg::JointAngles joint_targets);
    /** @brief Update function for movements in UNSUPPORTED state. */
  bool updateMoveLegs();
  /** @brief Set a movement of the base. Only used in specific situtations e.g. start up. */
  bool changeBase(Tfm::Vector3 move_base);
  /** @brief Check for state change conditions and apply if met. */
  void handleStateChange();
  /** @brief Clear variables used by visualisation. */
  void clearVisualisationChanges();
  void advanceGait();
  size_t gaitNextLeg();
  size_t gaitMaxRaised();

};

Hexapod buildDefaultHexapod();
Hexapod buildDefaultHexapod2();

#endif
