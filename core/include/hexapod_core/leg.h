/** @file leg.h
    @brief Header for Joint and Leg classes
*/

#ifndef HEXAPOD_LEG_H
#define HEXAPOD_LEG_H

#include "transformations.h"
#include "joint.h"

#ifdef __AVR__
#include <Arduino.h>
#else
#include <cstddef>
#include <cmath>
#endif

namespace hexapod {

/** @class Leg
    @brief A leg consists of a series of links and joints, from the leg base along to the foot.

    @details
    A leg can be attached to a robot at the leg base frame - this frame is centred on joint 1, with
   its Z axis aligned with joint 1 axis of rotation and X axis pointing in the joint 1 = 0 degrees
   direction.

    In general, if a leg is on the ground (State::ON_GROUND) it is the responsibility of the parent
   robot to update its joint angles if required i.e. if the leg base moves.

    But when a leg is in the air, calling stepUpdate() will let it handle its own trajectory.
    The trajectory can be updated by calling updateTargets().

    updateState() must always be called every period.
*/
class Leg {
 public:
  /** @brief Coxa, Femur, Tibia length (metres) TODO change units */
  struct Dims {
    float a;
    float b;
    float c;
  };
  /**
 * @brief Describes a rough area where the leg can move.
 * @details Likely calculated for a specific height (i.e. foot z value)
 */
  struct MovementLimits {
    float x_min;
    float x_max;
    float y_min;
    float y_max;
  };
  Dims dims_;
  MovementLimits movement_limits_grounded_, movement_limits_raised_;
  /** @brief Tolerance when checking if the foot is at the target position */
  static constexpr float target_tolerance = 0.001;  // TODO review
  /** @brief Struct used to represent all joint angles of a leg */
  struct JointAngles {
    float theta_1, theta_2, theta_3;
  };
  /** @brief Joint identifiers, used as indexes into joint array.
   * @details
   * Also known as base, hip and knee joints
   * @see joints_
   */
  enum { JOINT_1 = 0, JOINT_2, JOINT_3, NUM_JOINTS }; // TODO name this?
  /** @brief Leg state */
  enum class State { ON_GROUND, RAISED };
  /** @brief Inverse kinematics mode
   * WALK is most restrictive
   */
  enum class IKMode { FULL, WALK };
  /** @brief Current state */
  State state_ = State::ON_GROUND;
  /** @brief Previous state */
  State prev_state_ = State::ON_GROUND;
  /** @brief All joint objects for the leg */
  Joint joints_[NUM_JOINTS];

  /** @brief Need default constructor to allow us to create an array of Leg objects */
  Leg();
  /** @brief Create Leg with specific dimensions and joints
   * @details
   * If the leg is fully stretched out, all joint centres and the foot will be located directly
   * along the x-axis of the leg base frame.
   * The foot will be at distance a+b+c (no offsets in y or z) from the leg base.
   */
  Leg(Dims dims, Joint *joints);
  /** @brief Calculate joint angles for a given foot position */
  bool calculateJointAngles(const Vector3& pos, const IKMode ik_mode, JointAngles& calculated_angles) const;
  /** @brief Calculate joint angles for a given foot position and stage the result */
  bool calculateJointAngles(const Vector3& pos, const IKMode ik_mode);
  /** @brief Calculate foot position for given joint angles */
  void calculateFootPosition(const JointAngles& angles, Vector3& pos) const;
  /** @brief Check if a set of joint angles are allowed by the leg joint limits */
  bool jointsWithinLimits(const JointAngles& joint_angles) const;
  /** @brief Sets all leg joints */
  void setJointAngles(const JointAngles& angles);
  /** @brief Sets all leg joints using the physical joint angle values */
  void setJointAnglesFromPhysical(const JointAngles& angles);
  /** @brief Return staged angles */
  JointAngles getStagedAngles() const;
  /** @brief Sets staged angles */
  void setStagedAngles(const JointAngles& angles);
  /** @brief Sets all leg joints' angles to previously staged values. */
  void applyStagedAngles();
  /** @brief Sets staged angles to current joint angle values. */
  void resetStagedAngles();
  /** @brief Returns the current foot position in the leg frame */
  Vector3 getFootPosition() const;
  /** @brief Returns the neutral foot position in the leg frame. Z unknown by the leg, and is always
   * zero. */
  Vector3 getNeutralPosition() const;
  /** @brief Returns the neutral foot position for modification. */
  Vector3& getNeutralPosition();
  /** @brief Returns the current target position in the leg frame */
  Vector3 getTargetPosition() const;
  /** @brief Returns the current raised target position in the leg frame */
  Vector3 getRaisedPosition() const;
  /** @brief Returns the current joint angles */
  JointAngles getJointAngles() const;
  /** @brief Returns the current joint angles taking into account offset and axis flip for external use. */
  JointAngles getJointAnglesPhysical() const;

  /** @brief Conversion from physical to model joint angles, based on the specific joints for this leg. **/
  JointAngles fromPhysicalAngles(const Leg::JointAngles& physical_angles) const;
  /** @brief Conversion from model to physical joint angles, based on the specific joints for this leg. **/
  JointAngles toPhysicalAngles(const Leg::JointAngles& model_angles) const;

  /** @brief Updates joint angles as required by current trajectory. Must be called every period
   * when leg raised. */
  bool stepUpdate();
  /** @brief Updates leg status. Must be called every period. */
  bool updateStatus(bool raise);
  /** @brief Update leg raise trajectory targets */
  void updateTargets(const Vector3& target_pos, const Vector3& raised_pos,
                     uint16_t foot_air_time);
  /** @brief Initialise leg angles, position and targets */
  void setStartingAngles(JointAngles starting_angles);
  /** @brief Return step index (progress through a step). 0 if on the ground. */
  uint16_t getStepIdx() const;
  /** @brief Returns the duration of the currently set trajectory. */
  uint16_t getCurrentStepDuration() const;
  /** @brief Returns percentage progress through trajectory */
  float getCurrentStepProgress() const;

  /** @brief Sets a manually calculated trajectory */
  void setTrajectory(const JointAngles& target,
                     const JointAngles& increment_up,
                     const JointAngles& midpoint,
                     const JointAngles& increment_down,
                     uint16_t duration);
  /** @brief Clear current trajectory. Not to be called during normal use. Maybe during startup. */
  void clearTrajectory();
  /** @brief Updates the current joint angles according to the current joint targets and increments */
  void incrementLeg();
  /** @brief Calculate the ROUGH movement limits for the leg when leg base is at a given height above the foot. */
  MovementLimits calculateMovementLimits(float height) const;
  void updateMovementLimits(float walk_height, float raised_height);
  /** @brief Modifies target position to be within roughly estimated movement limits */
  Vector3 clampTarget(const Vector3& target_position, const MovementLimits& limits) const;


 private:
  /** @brief Current foot position relative to the leg base frame */
  Vector3 current_pos_;
  /** @brief Neutral position relative to the leg base frame. Z unknown by leg (always zero). */
  Vector3 neutral_pos_;
  /** @brief The final position (in joint space) of the foot during a step */
  JointAngles target_angles_;
  /** @brief The apex position (in joint space) of the foot during a step */
  JointAngles step_apex_angles_;
  /** @brief Incremental movement per time step while moving towards the apex during a step */
  JointAngles inc_up_angles_;
  /** @brief Incremental movement per time step while moving from the apex to the final position */
  JointAngles inc_down_angles_;
  /** @brief Store calculated joint angles (without implementing) */
  JointAngles staged_angles_;
  /** @brief Track leg movement while raised */
  uint16_t step_idx_ = 0;
  /** @brief The number of time steps that the leg will spend raised for the current step trajectory */
  uint16_t current_step_duration_;
  /** @brief The final position (in leg base frame) of the foot during a step */
  Vector3 target_pos_;
  /** @brief The apex position (in leg base frame) of the foot during a step */
  Vector3 raised_pos_;
  /** @brief Holds the latest calculated time for the leg step trajectory */
  uint16_t new_step_duration_;
  /** @brief Indicates whether target_pos_ or raised_pos_ have been updated */
  bool target_updated_ = false;
  /** @brief Calculate joint angles for a given position (up to 2 solutions) */
  uint8_t calculateJointAnglesFull(const Vector3& pos, JointAngles angles[2]) const;
  /** @brief Calculate joint angles for a given position while walking (joint 2 restricted) */
  uint8_t calculateJointAnglesWalk(const Vector3& pos, JointAngles& angles) const;
  /** @brief Checks whether a given set of joint angles give a specificied foot position */
  bool validateJointAngles(const JointAngles& angles, const Vector3& pos) const;
  /** @brief Updates the foot position member current_pos_ to be consistent with the current joint
   * angles */
  void updateFootPosition();
  /** @brief Returns the index of the angles closest to the reference angles */
  uint8_t chooseJointAnglesNearest(const JointAngles angle_options[2], uint8_t num_valid,
                                   const JointAngles& ref_angles) const;
  /** @brief Calculates the requires incremental joint movements for a given current joint position,
   * raised position and target position */
  void calculateTrajectory();
};

} // namespace hexapod

#endif // HEXAPOD_LEG_H