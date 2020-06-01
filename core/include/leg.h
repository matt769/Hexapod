/** @file leg.h
    @brief Header for Joint and Leg classes
*/

#ifndef LEG_H
#define LEG_H

#include "transformations.h"

#include <stddef.h>
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
    A leg can be attached to a robot at the leg base frame - this frame is centred on joint 1, with its 
     Z axis aligned with joint 1 axis of rotation and X axis pointing in the joint 1 = 0 degrees direction.

    In general, if a leg is on the ground (State::ON_GROUND) it is the responsibility of the parent robot 
    to update its joint angles if required i.e. if the leg base moves. TODO - might change this a bit later.

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
  static constexpr float target_tolerance =
      0.001;  // TODO should base this on allowed range of movement
  /** @brief Struct used to represent all joint angles of a leg */
  struct JointAngles {
    float theta_1, theta_2, theta_3;
  };
  /** @brief Joint identifiers, used as indexes into joints_.
   * Also known as base, hip and knee joint
   * @see joints_
   */
  enum { JOINT_1 = 0, JOINT_2, JOINT_3, NUM_JOINTS };
  /** @brief Leg state */
  enum class State { ON_GROUND, RAISED };
  /** @brief Inverse kinematics mode
   * WALK is most restrictive
   */
  enum class IKMode { FULL, WALK};
  /** @brief Current state */
  State state_ = State::ON_GROUND;
  /** @brief Previous state */
  State prev_state_ = State::ON_GROUND;
  /** @brief All joint objects for the leg */
  Joint joints_[NUM_JOINTS];

  /** @brief Need default constructor to allow us to create an array of Leg objects */
  Leg();
  /** @brief Create Leg with specific dimensions and joints */
  Leg(Dims dims, Joint* joints);
  /** @brief Calculate joint angles for a given foot position */
  bool calculateJointAngles(const Tfm::Vector3& pos, JointAngles& angles, const IKMode ik_mode);
  /** @brief Calculate foot position for given joint angles */
  bool calculateFootPosition(const JointAngles& angles, Tfm::Vector3& pos);
  /** @brief Check if a set of joint angles are allowed by the leg joint limits */
  bool jointsWithinLimits(const JointAngles& joint_angles) const;
  /** @brief Sets all leg joints */
  bool setJointAngles(const JointAngles& angles);  // no limit check
  /** @brief Returns the current foot position in the leg frame */
  Tfm::Vector3 getFootPosition() const;
  /** @brief Returns the current joint angles */
  JointAngles getJointAngles() const;
  /** @brief Updates joint angles as required by current trajectory. Must be called every period when leg raised. */
  bool stepUpdate();
  /** @brief Updates leg status. Must be called every period. */
  bool updateStatus(bool raise);
  /** @brief Update leg raise trajectory targets */
  void updateTargets(const Tfm::Vector3& target_pos, const Tfm::Vector3& raised_pos,
                     size_t foot_air_time);
  /** @brief Initialise leg angles, position and targets */
  bool setStartingAngles(JointAngles starting_angles);

 private:
  /** @brief Current foot position relative to the leg base frame */
  Tfm::Vector3 pos_; 
  /** @brief Used to describe the trajectory of the leg (in joint space) while raised
   * 
   * It will go move from its current position on the ground, up to joint_targets_raised_
   *  and then down to joint_targets_, moving at the specified increments each step.
   */
  JointAngles joint_targets_, joint_targets_raised_, joint_inc_up_, joint_inc_down_;
  /** @brief Track leg movement while raised */
  size_t step_no_ = 0;
  /** @brief The number of steps that the leg will spend raised for the current step */
  size_t current_foot_air_time_;
  /** @brief Used to describe the trajectory of the leg (in leg base frame) while raised */
  Tfm::Vector3 target_pos_, raised_pos_;  // TODO check target_pos_ is being initialised properly
  /** @brief TODO */
  size_t foot_air_time_new_;
  /** @brief Indicates whether target_pos_ or raised_pos_ have been updated */
  bool target_updated_ = false;
  /** @brief Calculate joint angles for a given position (up to 2 solutions) */
  size_t calculateJointAnglesFull(const Tfm::Vector3& pos, JointAngles angles[2]);
  /** @brief Calculate joint angles for a given position while walking (joint 2 restricted) */
  size_t calculateJointAnglesWalk(const Tfm::Vector3& pos, JointAngles& angles);
  /** @brief Checks whether a given set of joint angles give a specificied foot position */
  bool validateJointAngles(const JointAngles& angles, const Tfm::Vector3& pos);
  /** @brief Updates the foot position member pos_ to be consistent with the current joint angles */
  bool updateFootPosition();
  /** @brief Returns the index of the angles closest to the reference angles */
  size_t chooseJointAnglesNearest(const JointAngles angle_options[2], size_t num_valid,
                                const JointAngles& ref_angles) const;
  // size_t chooseJointAnglesWalk(const JointAngles angle_options[2], size_t num_valid,
  //                               const JointAngles& ref_angles) const;
  /** @brief Calculates the requires incremental joint movements for a given current joint position, raised position and target position */
  void calculateTrajectory();
  /** @brief Updates the current joint angles according to the current joint targets and increments */
  void incrementLeg();

};

#endif