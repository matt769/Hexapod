//
// Created by matt on 24/03/2021.
//

#include <iostream>
#include <ostream>
#include <vector>

#include "hexapod_core/build_hexapod.h"
#include "hexapod_core/hexapod.h"
#include "hexapod_core/joint.h"
#include "hexapod_core/leg.h"
#include "hexapod_core/transformations.h"

using namespace hexapod;

std::ostream& operator<<(std::ostream& os, const Vector3& v) {
  os << v.x() << ' ' << v.y() << ' ' << v.z();
  return os;
}

std::ostream& operator<<(std::ostream& os, const Leg::JointAngles& j) {
  os << j.theta_1 << ' ' << j.theta_2 << ' ' << j.theta_3;
  return os;
}

Vector3 ToPhysicalDeg(const Leg& l, const Leg::JointAngles& a) {
  return Vector3{l.joints_[0].toPhysicalAngle(a.theta_1) * 180.0f / (float)M_PI,
                 l.joints_[1].toPhysicalAngle(a.theta_2) * 180.0f / (float)M_PI,
                 l.joints_[2].toPhysicalAngle(a.theta_3) * 180.0f / (float)M_PI};
}

int main() {
  //  Hexapod h1 = buildDefaultHexapod();
  //  Hexapod h2 = buildDefaultHexapod2();
  //  Hexapod h3 = buildDefaultOctapod();
  //  Hexapod h4 = buildPhantomXForVis();
  //  Hexapod h5 = buildPhantomX();

  Joint j1 = JointBuilder(0, false).addPhysicalLimits(1, 3).setPhysicalAngle(2).create();
  Joint j2 = JointBuilder(0, false).addModelLimits(1, 3).setModelAngle(2).create();
  Joint j3 = JointBuilder(0, false).addModelLimits(1, 3).setPhysicalAngle(2).create();

  //    {
  //        constexpr float kDegToRad = M_PI / 180.0;
  //        constexpr float joint_2_offset = 14.0 * kDegToRad;
  //        constexpr float joint_3_offset = 46.0 * kDegToRad;
  //        constexpr float joint_3_offset_mod = joint_3_offset - joint_2_offset;
  //        auto jr = hexapod::Joint(-120.0f * kDegToRad, 88.0f * kDegToRad, joint_3_offset_mod,
  //        joint_3_offset_mod); auto jl = hexapod::Joint(-88.0f * kDegToRad, 120.0f * kDegToRad,
  //        -joint_3_offset_mod, -joint_3_offset_mod,
  //                                 true);
  //
  //        std::cout << jr.lower_limit_ << '\t' << jr.upper_limit_ << '\t' << jr.angle_ <<
  //        std::endl; std::cout << jl.lower_limit_ << '\t' << jl.upper_limit_ << '\t' << jl.angle_
  //        << std::endl;
  //
  //        return 0;
  //    }
  //  Hexapod h6 = buildPhantomXForVis();

  Hexapod hexapod = buildPhantomXForVis();

  //  const float sq = sqrtf(0.00119999994*0.00119999994);
  //  const float new_air_time = sq * 20000.002;
  //  std::cout << sq << '\t' << new_air_time << '\n';
  //

  //  hexapod.getLeg(0).calculateMovementLimits(0.0659999996)

  //  Hexapod px2 = buildPhantomX();
  //  auto left_leg = px2.getLeg(0);
  //  auto right_leg = px2.getLeg(1);

  //  auto l = hexapod.getLeg(0);
  //  Vector3 grounded_position{0.174964011, 0.0, 0.07/2.0};
  //    std::cout << "pos" << '\t' << grounded_position << '\n';
  //    Leg::JointAngles grounded_angles;
  //    bool ik_result = l.calculateJointAngles(grounded_position, Leg::IKMode::WALK,
  //    grounded_angles); if (ik_result) {
  //        bool res = l.jointsWithinLimits(grounded_angles);
  //        std::cout << res << '\t' << grounded_angles << '\n';
  ////        auto pd = ToPhysicalDeg(l, grounded_angles);
  //        std::cout << "pd\t" << ToPhysicalDeg(l, grounded_angles) << '\n';
  //    }

  const Leg& test_leg = hexapod.getLeg(0);
  const float walk_height_default = test_leg.dims_.c / 2.0;
  const float leg_lift_height_default = walk_height_default * 0.3f;
  Leg::MovementLimits ml_ground = test_leg.calculateMovementLimits(walk_height_default);
  Leg::MovementLimits ml_raised = test_leg.calculateMovementLimits(walk_height_default - leg_lift_height_default);
  std::cout << "grounded movement limits\t" << ml_ground.x_max << '\t' << ml_ground.x_min << '\t' << ml_ground.y_max
            << '\t' << ml_ground.y_min << '\n';
  std::cout << "raised movement limits\t" << ml_raised.x_max << '\t' << ml_raised.x_min << '\t' << ml_raised.y_max
            << '\t' << ml_raised.y_min << '\n';

  Leg::JointAngles starting_angles{0.0, M_PI / 2.0, M_PI / 4.0};
  for (uint8_t leg_idx = 0; leg_idx < 6; ++leg_idx) {
    hexapod.setLegJoints(leg_idx, starting_angles);
  }

  hexapod.setAllLegTargetsToGround();
  while (hexapod.getState() != Hexapod::State::STANDING) {
    hexapod.update();
  }
  std::cout << "finished setAllLegTargetsToGround\n";

  hexapod.riseToWalk();  // the raising has been triggered but need to wait for it to get there

  while (hexapod.getState() != Hexapod::State::WALKING) {
    hexapod.update();
  }
  std::cout << "finished riseToWalk\n";

  std::cout << (int)hexapod.getState() << '\n';
  //  hexapod.setWalk(Vector3(-0.001f, -0.001f, 0.0f), 0.0);
  //  for (int i =0; i < 1; ++i) {
  //    hexapod.setWalk(Vector3(-0.001f, -0.001f, 0.0f), 0.0);
  //    hexapod.update();
  //  }

  // Let it settle in case it needs to re-adjust feet after standing up
  for (int i = 0; i < 200; ++i) {
    hexapod.update();
  }
  for (size_t leg_idx = 0; leg_idx < hexapod.num_legs_; ++leg_idx) {
    std::cout << leg_idx << '\t' << (int)hexapod.getLeg(leg_idx).state_ << '\n';
  }

  std::cout << hexapod.getWalk().t.x << '\t' << hexapod.getWalk().t.y << '\n';

  std::cout << "leg raise time: " << hexapod.getLegRaiseTime() << '\n';
  hexapod.increaseWalkForward();
  hexapod.update();
  std::cout << hexapod.getWalk().t.x << '\t' << hexapod.getWalk().t.y << '\n';
  for (int i = 0; i < 200; ++i) {
    //    hexapod.increaseWalkForward();
    //    hexapod.decreaseWalkLeft();
    hexapod.update();
    std::cout << i << '\t';
    for (size_t leg_idx = 0; leg_idx < hexapod.num_legs_; ++leg_idx) {
      if (hexapod.getLeg(leg_idx).state_ == Leg::State::RAISED) {
        std::cout << leg_idx << '\t' << (int)hexapod.getLeg(leg_idx).state_;
        std::cout << '\t' << (int)hexapod.getLeg(leg_idx).step_idx_;
        std::cout << '\t' << (int)hexapod.getLeg(leg_idx).current_step_duration_;
      }
      std::cout << '\t';
    }
    std::cout << '\n';
  }

  hexapod.increaseWalkForward();
  hexapod.update();
  std::cout << hexapod.getWalk().t.x << '\t' << hexapod.getWalk().t.y << '\n';
  for (int i = 0; i < 10; ++i) {
    //    hexapod.increaseWalkForward();
    //    hexapod.decreaseWalkLeft();
    hexapod.update();
  }

  // make sure in steady state
  for (int i = 0; i < 200; ++i) {
    hexapod.update();
  }
  // look at the target position and distance travelled by a given leg (2)
  for (int i = 0; i < 100; ++i) {
    hexapod.update();
    const auto& leg = hexapod.getLeg(2);
    //    if (leg.state_ == Leg::State::RAISED) {
    std::cout << i << '\t' << (int)leg.state_ << '\t' << leg.step_idx_ << '\t' << leg.current_step_duration_ << '\t'
              << leg.target_pos_.y() << '\t' << leg.current_pos_.y() << '\n';
    //    }
  }
  // then change speed, make sure in steady state and check again

  hexapod.increaseWalkForward();
  hexapod.update();
  std::cout << hexapod.getWalk().t.x << '\t' << hexapod.getWalk().t.y << '\n';
  for (int i = 0; i < 200; ++i) {
    hexapod.update();
  }
  for (int i = 0; i < 100; ++i) {
    hexapod.update();
    const auto& leg = hexapod.getLeg(2);
    if (leg.state_ == Leg::State::RAISED) {
      std::cout << leg.step_idx_ << '\t' << leg.current_step_duration_ << '\t' << leg.target_pos_.y() << '\t'
                << leg.current_pos_.y() << '\n';
    }
  }

  exit(0);

  Vector3 small_step = Vector3(-0.001f, -0.001f, 0.0f);
  float angle_step = (-5.0 * M_PI / 180.0) / 50.0;
  hexapod.decreaseWalkForward();
  hexapod.decreaseWalkLeft();
  Transform tf_base_to_body_new;
  tf_base_to_body_new.R_.setRPYExtr(0.1f, 0.1f, 0.1f);
  hexapod.setBody(tf_base_to_body_new);

  int x = 0;
  for (int i = 0; i < 1000; i++) {
    hexapod.update();
    x++;
  }
  std::cout << x << '\n';

  // example of extracting joint angles
  float joint_array[18];
  uint8_t ja_idx = 0;
  for (uint8_t leg_idx = 0; leg_idx < hexapod.num_legs_; leg_idx++) {
    Leg::JointAngles lja = hexapod.getLeg(leg_idx).getJointAngles();
    joint_array[ja_idx++] = lja.theta_1;
    joint_array[ja_idx++] = lja.theta_2;
    joint_array[ja_idx++] = lja.theta_3;
  }
  //  for (uint8_t idx = 0; idx < 18; idx++) {
  //    std::cout << joint_array[idx] << '\n';
  //  }

  // see if we can test the manual mode
  hexapod.setFullManualControl(true);
  hexapod.update();
  std::cout << static_cast<int>(hexapod.getState()) << '\n';
  std::cout << hexapod.getManualControlLegIdx() << '\n';
  std::cout << hexapod.getLeg(0).getFootPosition() << '\n';
  hexapod.manualMoveFoot(Vector3{0.05, 0.05, 0.05});
  std::cout << hexapod.getLeg(0).getFootPosition() << '\n';
  hexapod.update();
  std::cout << hexapod.getLeg(0).getFootPosition() << '\n';

  hexapod.setManualJointControl(0);
  std::cout << static_cast<int>(hexapod.getManualControlJointIdx()) << '\n';
  hexapod.setManualJointControl(1);
  std::cout << static_cast<int>(hexapod.getManualControlJointIdx()) << '\n';
  hexapod.setManualJointControl(2);
  std::cout << static_cast<int>(hexapod.getManualControlJointIdx()) << '\n';
  hexapod.setManualJointControl(3);
  std::cout << static_cast<int>(hexapod.getManualControlJointIdx()) << '\n';

  std::cout << hexapod.getLeg(0).getJointAngles().theta_1 << '\t' << hexapod.getLeg(0).getJointAngles().theta_2 << '\t'
            << hexapod.getLeg(0).getJointAngles().theta_3 << '\n';
  hexapod.manualChangeJoint(0.1);
  std::cout << hexapod.getLeg(0).getJointAngles().theta_1 << '\t' << hexapod.getLeg(0).getJointAngles().theta_2 << '\t'
            << hexapod.getLeg(0).getJointAngles().theta_3 << '\n';

  hexapod.setManualJointControl(1);
  std::cout << static_cast<int>(hexapod.getManualControlJointIdx()) << '\n';
  hexapod.manualChangeJoint(0.1);
  std::cout << hexapod.getLeg(0).getJointAngles().theta_1 << '\t' << hexapod.getLeg(0).getJointAngles().theta_2 << '\t'
            << hexapod.getLeg(0).getJointAngles().theta_3 << '\n';

  Hexapod px = buildPhantomX();

  std::cout << hexapod.getLeg(0).getJointAnglesPhysical().theta_1 << '\t'
            << hexapod.getLeg(0).getJointAnglesPhysical().theta_2 << '\t'
            << hexapod.getLeg(0).getJointAnglesPhysical().theta_3 << '\n';
  std::cout << hexapod.getLeg(1).getJointAnglesPhysical().theta_1 << '\t'
            << hexapod.getLeg(1).getJointAnglesPhysical().theta_2 << '\t'
            << hexapod.getLeg(1).getJointAnglesPhysical().theta_3 << '\n';

  // test movement clamping
  hexapod::Leg::Dims leg_dims{0.05f, 0.066f, 0.132f};
  constexpr float kDegToRad = M_PI / 180.0;
  constexpr float joint_2_offset = 14.0 * kDegToRad;
  constexpr float joint_3_offset = 46.0 * kDegToRad;
  constexpr float joint_3_offset_mod = joint_3_offset - joint_2_offset;
  hexapod::Joint test_joints[3];
  test_joints[0] = hexapod::Joint(-80.0f * kDegToRad, 80.0f * kDegToRad, 0.0, 0.0);
  test_joints[1] = hexapod::Joint(-95.0f * kDegToRad, 95.0f * kDegToRad, joint_2_offset, joint_2_offset);
  test_joints[2] = hexapod::Joint(-120.0f * kDegToRad, 88.0f * kDegToRad, joint_3_offset_mod, joint_3_offset_mod);

  hexapod::Leg leg(leg_dims, test_joints);
  leg.updateMovementLimits(leg_dims.c / 2.0, 0.7 * (leg_dims.c / 2.0));
  Vector3 neutral2 = leg.getNeutralPosition();
  std::cout << neutral2.x() << '\t' << neutral2.y() << '\t' << neutral2.z() << '\n';
  std::cout << leg.movement_limits_grounded_.x_min << '\t' << leg.movement_limits_grounded_.x_max << '\t'
            << leg.movement_limits_grounded_.y_min << '\t' << leg.movement_limits_grounded_.y_max << '\n';
  Vector3 neutral = leg.getNeutralPosition();

  std::vector<Vector3> test_positions;
  test_positions.push_back(neutral);
  test_positions.push_back(neutral + Vector3{0.01, 0.0, 0.0});
  test_positions.push_back(neutral + Vector3{0.0, 0.01, 0.0});
  test_positions.push_back(neutral + Vector3{-0.01, 0.0, 0.0});
  test_positions.push_back(neutral + Vector3{0.0, -0.01, 0.0});
  test_positions.push_back(neutral + Vector3{0.01, 0.01, 0.0});
  test_positions.push_back(neutral + Vector3{0.01, -0.01, 0.0});
  test_positions.push_back(neutral + Vector3{-0.01, 0.01, 0.0});
  test_positions.push_back(neutral + Vector3{-0.01, -0.01, 0.0});

  test_positions.push_back(neutral + Vector3{0.2, 0.0, 0.0});
  test_positions.push_back(neutral + Vector3{0.0, 0.2, 0.0});
  test_positions.push_back(neutral + Vector3{-0.2, 0.0, 0.0});
  test_positions.push_back(neutral + Vector3{0.0, -0.2, 0.0});
  test_positions.push_back(neutral + Vector3{0.2, 0.2, 0.0});
  test_positions.push_back(neutral + Vector3{0.2, -0.2, 0.0});
  test_positions.push_back(neutral + Vector3{-0.2, 0.2, 0.0});
  test_positions.push_back(neutral + Vector3{-0.2, -0.2, 0.0});

  for (const auto& pos : test_positions) {
    Vector3 clamped_result = leg.clampTarget(pos, leg.movement_limits_grounded_);
    std::cout << pos.x() << '\t' << pos.y() << '\t' << pos.z() << '\n';
    std::cout << clamped_result.x() << '\t' << clamped_result.y() << '\t' << clamped_result.z() << '\n';
  }

  // check inside and out in all 4 quadrants, and on axes
  // check zero
  // check Z value makes no difference
}
