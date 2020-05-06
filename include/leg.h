#ifndef LEG_H
#define LEG_H

#include "transformations.h"

#include <stddef.h>
#include <cmath>

namespace Tfm = Transformations;

class Joint {
 public:
  float offset_;
  float lower_limit_;
  float upper_limit_;
  float angle_;
  Joint();
  Joint(float offset, float lower_limit, float upper_limit);
  Joint(float offset, float lower_limit, float upper_limit, float angle);
  bool isWithinLimits(float angle) const;
  float clampToLimts(float angle) const;
};

// This is a leg!
// Numbers are joints, letters are link length
//
//            3      -
//     a    b/ \     -
// 1 ------2/   \c   -
//               \   -

class Leg {
 public:
  static constexpr float a = 0.2f;
  static constexpr float b = 0.4f;
  static constexpr float c = 0.6f;
  static constexpr size_t num_joints_ = 3;
  static constexpr float target_tolerance =
      0.001;  // TODO should base this on allowed range of movement
  struct JointAngles {
    float theta_1, theta_2, theta_3;
  };
  enum { JOINT_1 = 0, JOINT_2, JOINT_3 };
  enum class State { ON_GROUND, RAISED };
  uint8_t id_;  // TODO initialise properly
  State state_ = State::ON_GROUND;
  State prev_state_ = State::ON_GROUND;

  Leg();
  Leg(JointAngles joint_angles);
  // given a desired foot position relative to the leg base link
  // calculate the required joint angles (up to 2 possibilities)
  // returns number of solutions found
  size_t calculateJointAngles(const Tfm::Vector3& pos, JointAngles angles[2]); // IK
  bool calculateFootPosition(const JointAngles& angles, Tfm::Vector3& pos); // FK
  bool jointsWithinLimits(const JointAngles& joint_angles) const;
  bool setJointAngles(const JointAngles& angles);  // no limit check
  bool setFootPosition(const Tfm::Vector3& pos);  // will have to pick one set of angles if multiple
  // checks if position is feasible and if so return those joint angles
  bool testSetFootPosition(const Tfm::Vector3& pos, JointAngles& ret_angles_if_valid);
  Tfm::Vector3 getFootPosition() const;
  JointAngles getJointAngles() const;
  bool stepUpdate();
  bool updateStatus(bool raise);
  void updateTargets(const Tfm::Vector3& target_pos, const Tfm::Vector3& raised_pos,
                     size_t foot_air_time);
  bool setStartingAngles(JointAngles starting_angles);

 private:
  Joint joints_[num_joints_];
  Tfm::Vector3 pos_; 
  JointAngles joint_targets_, joint_inc_up_, joint_inc_down_;  // for movement when raised
  size_t step_no_ = 0;
  size_t current_foot_air_time_;          // for current step
  Tfm::Vector3 target_pos_, raised_pos_;  // TODO target_pos_ NEEDS TO BE INITIALISED PROPERLY
  size_t foot_air_time_new_;
  bool target_updated_ = false;
  bool validateJointAngles(const JointAngles& angles, const Tfm::Vector3& pos);
  bool updateFootPosition();
  size_t chooseJointAngleOption(const JointAngles angle_options[2], size_t num_valid,
                                const JointAngles& ref_angles) const;


};

#endif