#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "leg.h"
#include "transformations.h"

#ifdef __AVR__
#include <math.h>
#else
#include <cstddef>
#include <cmath>
#include <iostream>
#endif

namespace Tfm = Transformations;

/** @class Hexapod
 * @brief A hexapod contains a body and a number of legs (not actually limited to 6), and manages
 * the legs in order to move the body around.
*/
class Hexapod {
 public:
  /** @brief Describes a basic cuboid body shape */
  struct Dims {
    float length;
    float width;
    float depth;
  };
  /** @brief The hexapod body */
  Dims dims_;
  uint8_t num_legs_;
  /** @brief Heading moves with body frame (Standard) or stays fixed (Headless) */
  enum class MoveMode { STANDARD, HEADLESS };
  /** @brief Walking is main state, others are used in start up routine. Unsupported implies legs do
   * not any resetriction on foot position. */
  enum class State { UNSUPPORTED, STANDING, WALKING };
  /** @brief Gait identifier. Also used as index into gait_seq_ */
  enum Gait { RIPPLE = 0, LEFT_RIGHT_LEFT_RIGHT, LHS_THEN_RHS, AROUND_THE_CLOCK, NUM_GAITS };
  /** @brief During start up, hexapod will rise to this height before entering walking state */
  static constexpr float walk_height_default_ = 0.4;
  /** @brief Number of periods leg will be in air if raised while walk speed is zero */
  static constexpr uint16_t foot_air_time_default_ = 20;
  /** @brief Minimum time foot can be in air during a raise i.e. fastest raise movement */
  static constexpr uint16_t foot_air_time_min_ = 6;
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

  /** @brief Construct a new Hexapod object */
  Hexapod(uint8_t num_legs, Dims hex_dims, Tfm::Transform* tf_body_to_leg, Leg* legs);
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
  /** @brief main update function to be called every period. Will apply any movement currently set.
   */
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
  const Leg& getLeg(uint8_t leg_idx) const;
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
  Leg::JointAngles* staged_angles_;
  /** @brief Current walk vector. TODO review naming. */
  Tfm::Vector3 walk_step_current_;
  /** @brief Requested walk vector. */
  Tfm::Vector3 walk_step_new_;
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
  uint16_t total_movement_steps_;
  /** @brief Current progress of unsupported movements. */
  uint16_t current_movement_step_;

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

  uint8_t current_gait_seq_ = Gait::RIPPLE;
  uint8_t gait_current_pos_ = 0;
  float stance_width_ = stance_width_default_;
  float leg_lift_height_ = leg_lift_height_default_;
  float foot_ground_travel_ratio_ = fgtr_default_;
  float total_base_rotation_ = 0.0f;

  /** @brief Calculate the required joint angles for all grounded legs */
  bool calculateGroundedLegs();
  /** @brief Applies the pre-calculated joint angles for all grounded legs. */
  void applyChangesGroundedLegs();
  /** @brief Returns number of legs currently raised. */
  uint8_t getNumLegsRaised() const;
  /** @brief Includes everything necessary to manage the grounded legs. */
  bool handleGroundedLegs();
  /** @brief Calculates vector induced at neutral position due to movement. */
  Tfm::Vector3 calculateFootVector(uint8_t leg_idx) const;
  /** @brief Includes everything necessary to manage the raised legs. */
  void handleRaisedLegs();
  /** @brief Update status of each leg and request they raise if conditions met. */
  void updateLegs();
  /** @brief Converts a vector in a leg frame to the correspondng vector in the base frame. */
  Tfm::Vector3 legToBase(uint8_t leg_idx, const Tfm::Vector3& v) const;
  /** @brief Return neutral position for a leg in the base frame. */
  Tfm::Vector3 getNeutralPosition(uint8_t leg_idx) const;
  /** @brief Return target position for a leg in the base frame. */
  Tfm::Vector3 getTargetPosition(uint8_t leg_idx) const;
  /** @brief Return raised target position for a leg in the base frame. */
  Tfm::Vector3 getRaisedPosition(uint8_t leg_idx) const;
  /** @brief Update foot targets (if required) for single leg*/
  void updateFootTarget(uint8_t leg_idx);
  /** @brief Call updateFootTarget for all raised legs */
  void updateFootTargets();
  /** @brief Return foot position in the base frame. */
  Tfm::Vector3 getFootPosition(uint8_t leg_idx) const;
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
  /** @brief To be called after any leg raise event. */
  void advanceGait();
  /** @brief Returns the index of the next leg to be raised. */
  uint8_t gaitNextLeg();
  /** @brief Returns the maximum number of legs that can be raised during the current gait. */
  uint8_t gaitMaxRaised();
};

/** @brief Returns a Hexapod object consistent with example file hexapod.urdf.xacro */
Hexapod buildDefaultHexapod();
/** @brief Returns a Hexapod object consistent with example file hexapod2.urdf.xacro */
Hexapod buildDefaultHexapod2();
/** @brief Returns a Hexapod object consistent with example file octapod.urdf.xacro */
Hexapod buildDefaultOctapod();

#endif
