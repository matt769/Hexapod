//
// Created by matt on 24/03/2021.
//

#include <iostream>

#include "hexapod.h"
#include "transformations.h"

using namespace Transformations;

int main() {
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
  for (uint8_t idx = 0; idx < 18; idx++) {
    std::cout << joint_array[idx] << '\n';
  }


}

