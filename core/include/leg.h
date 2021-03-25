/** @file leg.h
    @brief Header for Joint and Leg classes
*/

#ifndef LEG_H
#define LEG_H

#include "transformations.h"

#include <cstddef>
#include <cmath>

namespace Tfm = Transformations;

/** @class Joint
    @brief Wraps current angle and limits, plus some basic utility functions.
*/
class Joint {
 public:
  float lower_limit_;
  float upper_limit_;
  float angle_;
  Joint();
  Joint(float lower_limit, float upper_limit);
  Joint(float lower_limit, float upper_limit, float angle);
  bool isWithinLimits(float angle) const;
  float clampToLimts(float angle) const;
};

/** @class Leg
    @brief A leg consists of a series of links and joints, from the leg base along to the foot.

    @details
    A leg can be attached to a robot at the leg base frame - this frame is centred on joint 1, with
   its
     Z axis aligned with joint 1 axis of rotation and X axis pointing in the joint 1 = 0 degrees
   direction.

    In general, if a leg is on the ground (State::ON_GROUND) it is the responsibility of the parent
   robot
    to update its joint angles if required i.e. if the leg base moves. TODO - might change this a
   bit later.

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
  Dims dims_;
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
  enum { JOINT_1 = 0, JOINT_2, JOINT_3, NUM_JOINTS };
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
  Leg(Dims dims, Joint* joints);
  /** @brief Calculate joint angles for a given foot position */
  bool calculateJointAngles(const Tfm::Vector3& pos, const IKMode ik_mode);
  /** @brief Calculate foot position for given joint angles */
  bool calculateFootPosition(const JointAngles& angles, Tfm::Vector3& pos);
  /** @brief Check if a set of joint angles are allowed by the leg joint limits */
  bool jointsWithinLimits(const JointAngles& joint_angles) const;
  /** @brief Sets all leg joints */
  bool setJointAngles(const JointAngles& angles);
  /** @brief Return staged angles */
  JointAngles getStagedAngles() const;
  /** @brief Sets staged angles */
  bool setStagedAngles(const JointAngles& angles);
  /** @brief Sets all leg joints */
  bool applyStagedAngles();
  /** @brief Returns the current foot position in the leg frame */
  Tfm::Vector3 getFootPosition() const;
  /** @brief Returns the neutral foot position in the leg frame. Z unknown by the leg, and is always
   * zero. */
  Tfm::Vector3 getNeutralPosition() const;
  /** @brief Returns the neutral foot position for modification. */
  Tfm::Vector3& getNeutralPosition();
  /** @brief Returns the current target position in the leg frame */
  Tfm::Vector3 getTargetPosition() const;
  /** @brief Returns the current raised target position in the leg frame */
  Tfm::Vector3 getRaisedPosition() const;
  /** @brief Returns the current joint angles */
  JointAngles getJointAngles() const;
  /** @brief Updates joint angles as required by current trajectory. Must be called every period
   * when leg raised. */
  bool stepUpdate();
  /** @brief Updates leg status. Must be called every period. */
  bool updateStatus(bool raise);
  /** @brief Update leg raise trajectory targets */
  void updateTargets(const Tfm::Vector3& target_pos, const Tfm::Vector3& raised_pos,
                     size_t foot_air_time);
  /** @brief Initialise leg angles, position and targets */
  bool setStartingAngles(JointAngles starting_angles);
  /** @brief Return step index (progress through a step). 0 if on the ground. */
  size_t getStepIdx() const;

 private:
  /** @brief Current foot position relative to the leg base frame */
  Tfm::Vector3 current_pos_;
  /** @brief Neutral position relative to the leg base frame. Z unknown by leg (always zero). */
  Tfm::Vector3 neutral_pos_;
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
  size_t step_idx_ = 0;
  /** @brief The number of time steps that the leg will spend raised for the current step */
  size_t current_step_duration_;
  /** @brief The final position (in leg base frame) of the foot during a step */
  Tfm::Vector3 target_pos_;
  /** @brief The apex position (in leg base frame) of the foot during a step */
  Tfm::Vector3 raised_pos_;
  /** @brief Holds the latest calculated time for the leg step */
  size_t new_step_duration_;
  /** @brief Indicates whether target_pos_ or raised_pos_ have been updated */
  bool target_updated_ = false;
  /** @brief Calculate joint angles for a given position (up to 2 solutions) */
  size_t calculateJointAnglesFull(const Tfm::Vector3& pos, JointAngles angles[2]);
  /** @brief Calculate joint angles for a given position while walking (joint 2 restricted) */
  size_t calculateJointAnglesWalk(const Tfm::Vector3& pos, JointAngles& angles);
  /** @brief Checks whether a given set of joint angles give a specificied foot position */
  bool validateJointAngles(const JointAngles& angles, const Tfm::Vector3& pos);
  /** @brief Updates the foot position member current_pos_ to be consistent with the current joint
   * angles */
  bool updateFootPosition();
  /** @brief Returns the index of the angles closest to the reference angles */
  size_t chooseJointAnglesNearest(const JointAngles angle_options[2], size_t num_valid,
                                  const JointAngles& ref_angles) const;
  // size_t chooseJointAnglesWalk(const JointAngles angle_options[2], size_t num_valid,
  //                               const JointAngles& ref_angles) const;
  /** @brief Calculates the requires incremental joint movements for a given current joint position,
   * raised position and target position */
  void calculateTrajectory();
  /** @brief Updates the current joint angles according to the current joint targets and increments
   */
  void incrementLeg();
};

#endif