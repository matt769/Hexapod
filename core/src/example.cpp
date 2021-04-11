//
// Created by matt on 24/03/2021.
//

#include <iostream>
#include <ostream>

#include "hexapod.h"
#include "transformations.h"
#include "build_hexapod.h"

using namespace hexapod;

std::ostream& operator<<(std::ostream& os, const Vector3& v) {
  os << v.x() << ' '  << v.y() << ' ' << v.z();
  return os;
}


int main() {
  Hexapod h1 = buildDefaultHexapod();
  Hexapod h2 = buildDefaultHexapod2();
  Hexapod h3 = buildDefaultOctapod();
  Hexapod h4 = buildPhantomXForVis();
  Hexapod h5 = buildPhantomX();

  Hexapod hexapod = buildDefaultHexapod();
  Leg::JointAngles starting_angles{0.0, M_PI / 2.0, M_PI / 4.0};
  hexapod.setStartingPosition(starting_angles);

  hexapod.setLegsToGround();
  while (hexapod.getState() != Hexapod::State::STANDING) {
    hexapod.update();
  }

  while (hexapod.getState() == Hexapod::State::STANDING) {
    hexapod.riseToWalk();
    hexapod.update();
  }

  Vector3 small_step = Vector3(-0.001f, -0.001f, 0.0f);
  float angle_step = (-5.0 * M_PI / 180.0) / 50.0;
  hexapod.setWalk(small_step, angle_step);
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

  std::cout << hexapod.getLeg(0).getJointAngles().theta_1 << '\t'
    << hexapod.getLeg(0).getJointAngles().theta_2 << '\t'
    << hexapod.getLeg(0).getJointAngles().theta_3 << '\n';
  hexapod.manualChangeJoint(0.1);
  std::cout << hexapod.getLeg(0).getJointAngles().theta_1 << '\t'
            << hexapod.getLeg(0).getJointAngles().theta_2 << '\t'
            << hexapod.getLeg(0).getJointAngles().theta_3 << '\n';

  hexapod.setManualJointControl(1);
  std::cout << static_cast<int>(hexapod.getManualControlJointIdx()) << '\n';
  hexapod.manualChangeJoint(0.1);
  std::cout << hexapod.getLeg(0).getJointAngles().theta_1 << '\t'
            << hexapod.getLeg(0).getJointAngles().theta_2 << '\t'
            << hexapod.getLeg(0).getJointAngles().theta_3 << '\n';


  Hexapod px = buildPhantomX();

  std::cout << hexapod.getLeg(0).getJointAnglesPhysical().theta_1 << '\t'
            << hexapod.getLeg(0).getJointAnglesPhysical().theta_2 << '\t'
            << hexapod.getLeg(0).getJointAnglesPhysical().theta_3 << '\n';
  std::cout << hexapod.getLeg(1).getJointAnglesPhysical().theta_1 << '\t'
            << hexapod.getLeg(1).getJointAnglesPhysical().theta_2 << '\t'
            << hexapod.getLeg(1).getJointAnglesPhysical().theta_3 << '\n';


}

