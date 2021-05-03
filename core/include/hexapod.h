#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "leg.h"
#include "transformations.h"

#ifdef __AVR__
#include <Arduino.h>
#else
#include <cstddef>
#endif

namespace hexapod {

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
  enum class State { UNSUPPORTED, STANDING, WALKING, FULL_MANUAL };
  /** @brief Gait identifier. Also used as index into gait_seq_ */
  enum Gait { RIPPLE = 0, LEFT_RIGHT_LEFT_RIGHT, LHS_THEN_RHS, AROUND_THE_CLOCK, NUM_GAITS };

  /** @brief Construct a new Hexapod object */
  Hexapod(uint8_t num_legs, Dims hex_dims, Transform *tf_body_to_leg, Leg *legs, uint16_t update_frequency = 50);
  ~Hexapod();
  Hexapod(const Hexapod&) = delete;
  Hexapod(Hexapod&&) = default;
  Hexapod& operator=(const Hexapod&) = delete;
  Hexapod& operator=(Hexapod&&) = default;
  /** @brief Set walk or turn movement for the next period */
  bool setWalk(const Vector3& walk_step, float angle_step);
  /** @brief Set walk or turn movement for the next period */
  bool setWalk(const Vector3& walk_step);
  /** @brief Set walk or turn movement for the next period */
  bool setWalk(float angle_step);
  /** @brief Set absolute body rotation and translation relative to the base */
  bool setBody(const Transform& tf_base_to_body_target);
  /** @brief Apply incremental rotation and translation to the body, relative to the base */
  bool changeBody(const Transform& tf_base_to_body_change);
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
  bool setLegJoints(uint8_t leg_idx, const Leg::JointAngles& joint_angles);
  bool setLegJointsPhysical(uint8_t leg_idx, const Leg::JointAngles& physical_joint_angles);
  uint16_t getUpdateFrequency() const;


  /** @brief From unsupported state set feet targets to the ground. */
  bool setAllLegTargetsToGround(uint16_t duration);
  bool setAllLegTargetsToGround();
  bool setLegTargetToGround(uint8_t leg_idx, uint16_t duration);
  /** @brief Set the base to move upwards until walk_height_default_ reached */
  bool riseToWalk();
  State getState() const;
  /** @brief For visualisation */
  const Leg& getLeg(uint8_t leg_idx) const;
  /** @brief For visualisation */
  const Transform& getBaseToBody() const;
  /** @brief For visualisation */
  const Transform& getBaseMovement() const;
  /** @brief For visualisation */
  float getHeight() const;
  /** @brief Toggle full manual control mode (allowing various manual movements) */
  void setFullManualControl(bool control_on);
  /** @brief Manual control sub type determines what movements are allowed */
  enum class ManualControlType { ALL_LEGS, SINGLE_LEG, SINGLE_JOINT };
  /** @brief Set ManualControlType to ALL_LEGS */
  void setManualLegControl();
  /** @brief Set ManualControlType to SINGLE_LEG */
  void setManualLegControl(uint8_t leg_idx);
  /** @brief Set ManualControlType to SINGLE_JOINT using joint index into leg */
  void setManualJointControl(uint8_t joint_idx);
  /** @brief Position control of the selected foot when in ALL_LEGS or SINGLE_LEG manual mode */
  void manualMoveFoot(const Vector3& movement);
  /** @brief Angle control of the selected joint when in SINGLE_JOINT manual mode */
  void manualChangeJoint(float angle_change);
  ManualControlType getManualControlType() const;
  uint8_t getManualControlLegIdx() const;
  uint8_t getManualControlJointIdx() const;

  /** @brief Set robot-wide targets to move all legs to specified joint positions when in unsupported state. */
  bool setAllLegTargets(const Leg::JointAngles *joint_targets, uint16_t duration);
  /** @brief Set robot-wide targets to move all legs to common joint position when in unsupported state. */
  bool setAllLegTargets(const Leg::JointAngles& joint_targets, uint16_t duration);
  bool setLegTarget(uint8_t leg_idx, const Leg::JointAngles& joint_targets, uint16_t duration);

 private:
  /** @brief At what frequency (per second) will update() be called. Used to set speed of some movements. */
  uint16_t update_frequency_;
  /** @brief During start up, hexapod will rise to this height before entering walking state */
  float walk_height_default_;
  /** @brief Number of periods leg will be in air if raised while walk speed is zero */
  uint16_t foot_air_time_default_;
  /** @brief Minimum time foot can be in air during a raise i.e. fastest raise movement */
  uint16_t foot_air_time_min_;
  float stance_width_min_;
  float stance_width_max_;
  float stance_width_default_;
  float leg_lift_height_min_;
  float leg_lift_height_max_;
  float leg_lift_height_default_;
  static constexpr float fgtr_min_ = 0.1f;
  static constexpr float fgtr_max_ = 0.9f;
  /** @brief Default stride length as proportion of allowed foot movement */
  static constexpr float fgtr_default_ = 0.5f;
  /** @brief For moving the body up from the ground to a walking position */
  float rising_increment_;

  Leg* legs_;
  /** @brief Height of base frame above ground */
  float height_;
  /** @brief Relationship between base frame and body frame */
  Transform tf_base_to_body_;  //
  /** @brief Fixed relationship between body frame and leg frames */
  Transform *tf_body_to_leg_;
  MoveMode move_mode_ = MoveMode::STANDARD;
  State state_ = State::UNSUPPORTED;
  State requested_state_ = State::UNSUPPORTED;
  /** @brief For visualisation. New base expressed in old base frame. */
  Transform tf_base_movement_;
  /** @brief Target walk movement expressed in current base frame */
  Transform tf_base_to_new_base_target_;
  /** @brief Target base to body transform expressed in current base frame */
  Transform tf_base_to_body_target_;
  /** @brief Current walk vector. */
  Vector3 walk_step_current_;
  /** @brief Requested walk vector. */
  Vector3 walk_step_target_;
  /** @brief Current walk vector. */
  float turn_step_current_;
  /** @brief Requested turn angle. */
  float turn_step_target_;
  /** @brief Flag to indicate there's been a change in base to body */
  bool body_change_ = false;
  /** @brief Flag to indicate there's been a change in base to 'new' base' */
  bool base_change_ = false;
  /** @brief Flag to recalculate targets for all feet. */
  bool recalculate_all_feet_targets_ = false;
  /** @brief Flag to recalculate targets for any raised feet. */
  bool recalculate_raised_feet_targets_ = false;

  /**
   * @brief Describes a circle of allowed foot movement around the neutral point.
   *
   * @details
   * TODO review this. Ideally have better solution to managing range of movement.
   */
  float allowed_foot_position_diameter_;

  uint8_t current_gait_seq_ = Gait::RIPPLE;
  uint8_t gait_current_pos_ = 0;
  float stance_width_ = stance_width_default_;
  float leg_lift_height_ = leg_lift_height_default_;
  float foot_ground_travel_ratio_ = fgtr_default_;
  float total_base_rotation_ = 0.0f;

  ManualControlType manual_control_type_;
  /** @brief The currently selected leg if under Manual SINGLE_LEG control */
  uint8_t manual_leg_idx_;
  /** @brief The currently selected leg if under Manual SINGLE_JOINT control */
  uint8_t manual_joint_idx_; // index into Leg::JointAngles





  /** @brief Calculate the required joint angles for all grounded legs */
  bool calculateGroundedLegs();
  /** @brief Applies the pre-calculated joint angles for all grounded legs. */
  void applyChangesGroundedLegs();
  /** @brief Apply all the staged leg angles and maybe update some current hexapod state variables */
  void commitLegJointChanges();
  /** @brief Returns number of legs currently raised. */
  uint8_t getNumLegsRaised() const;
  /** @brief Includes everything necessary to manage the grounded legs. */
  bool handleGroundedLegs();
  /** @brief Calculates vector induced at neutral position due to movement. */
  Vector3 calculateFootVector(uint8_t leg_idx) const;
  /** @brief Includes everything necessary to manage the raised legs. */
  bool handleRaisedLegs();
  /** @brief Update status of each leg and request they raise if conditions met. */
  void updateLegsStatus();
  /** @brief Converts a vector in a leg frame to the correspondng vector in the base frame. */
  Vector3 legToBase(uint8_t leg_idx, const Vector3& v) const;
  /** @brief Return neutral position for a leg in the base frame. */
  Vector3 getNeutralPosition(uint8_t leg_idx) const;
  /** @brief Return target position for a leg in the base frame. */
  Vector3 getTargetPosition(uint8_t leg_idx) const;
  /** @brief Return raised target position for a leg in the base frame. */
  Vector3 getRaisedPosition(uint8_t leg_idx) const;
  /** @brief Update foot targets (if required) for single leg*/
  void updateFootTarget(uint8_t leg_idx);
  /** @brief Call updateFootTarget for all raised legs */
  void updateRaisedFootTargets();
  /** @brief Return foot position in the base frame. */
  Vector3 getFootPosition(uint8_t leg_idx) const;
  /** @brief Clear all movement targets (walk, turn, body etc). */
  void clearTargets();
  /** @brief Update function for movements in UNSUPPORTED state. */
  bool updateMoveLegs();
  /** @brief Set a movement of the base. Only used in specific situtations e.g. start up. */
  bool changeBase(const Vector3& move_base);
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
  void commitTargets();

};

} // namespace hexapod

#endif // HEXAPOD_H
