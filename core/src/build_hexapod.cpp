//
// Created by matt on 12/04/2021.
//

#include "build_hexapod.h"

namespace hexapod {

hexapod::Hexapod buildDefaultHexapod() {
  // Construct a leg (they are all the same in this default robot)
  constexpr uint8_t num_joints = 3;
  hexapod::Leg::Dims leg_dims{0.2f, 0.4f, 0.6f};

  hexapod::Joint joints[num_joints];
  joints[0] = hexapod::Joint(-90.0f * M_PI / 180.0, 90.0f * M_PI / 180.0, 0.0);
  joints[1] = hexapod::Joint(-150.0f * M_PI / 180.0, 150.0f * M_PI / 180.0, M_PI / 2.0);
  joints[2] = hexapod::Joint(-150.0f * M_PI / 10.0, 150.0f * M_PI / 180.0, M_PI / 4.0);

  hexapod::Leg leg(leg_dims, joints);

  // Make an array of legs and copy the one we just made into all elements
  constexpr uint8_t num_legs = 6;
  hexapod::Leg *legs = new hexapod::Leg[num_legs];
  for (uint8_t leg_idx = 0; leg_idx < num_legs; leg_idx++) {
    legs[leg_idx] = leg;
  }

  // Hexapod body dimensions
  constexpr float length = 1.8f;
  constexpr float width = 1.0f;
  constexpr float depth = 0.2f;
  const hexapod::Hexapod::Dims hex_dims{length, width, depth};

  // Transformations between body frame and leg base frames
  // LHS leg base frames need to be rotated by 90, RHS by -90
  hexapod::Transform *tf_body_to_leg = new hexapod::Transform[num_legs];

  hexapod::Vector3 front((hex_dims.length / 2.0f) * 0.6666666f, 0.0f, 0.0f);
  hexapod::Vector3 back = -front;
  hexapod::Vector3 left(0.0f, hex_dims.width / 2.0f, 0.0f);
  hexapod::Vector3 right = -left;
  hexapod::Vector3 middle(0.0f, 0.0f, 0.0f);

  // Legs MUST be ordered this way i.e. left then right, then next row left ...
  enum { FRONT_LEFT = 0, FRONT_RIGHT, MIDDLE_LEFT, MIDDLE_RIGHT, BACK_LEFT, BACK_RIGHT };

  tf_body_to_leg[FRONT_LEFT].t_ = front + left;
  tf_body_to_leg[FRONT_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[MIDDLE_LEFT].t_ = middle + left;
  tf_body_to_leg[MIDDLE_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[BACK_LEFT].t_ = back + left;
  tf_body_to_leg[BACK_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[FRONT_RIGHT].t_ = front + right;
  tf_body_to_leg[FRONT_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[MIDDLE_RIGHT].t_ = middle + right;
  tf_body_to_leg[MIDDLE_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[BACK_RIGHT].t_ = back + right;
  tf_body_to_leg[BACK_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  return Hexapod(num_legs, hex_dims, tf_body_to_leg, legs);
}
hexapod::Hexapod buildDefaultHexapod2() {
  // Construct a leg (they are all the same in this default robot)
  constexpr uint8_t num_joints = 3;
  hexapod::Leg::Dims leg_dims{0.2f, 0.4f, 0.6f};

  hexapod::Joint joints[num_joints];
  joints[0] = hexapod::Joint(-90.0f * M_PI / 180.0, 90.0f * M_PI / 180.0, 0.0);
  joints[1] = hexapod::Joint(-150.0f * M_PI / 180.0, 150.0f * M_PI / 180.0, M_PI / 2.0);
  joints[2] = hexapod::Joint(-150.0f * M_PI / 10.0, 150.0f * M_PI / 180.0, M_PI / 4.0);

  hexapod::Leg leg(leg_dims, joints);

  // Make an array of legs and copy the one we just made into all elements
  constexpr uint8_t num_legs = 6;
  hexapod::Leg *legs = new hexapod::Leg[num_legs];
  for (uint8_t leg_idx = 0; leg_idx < num_legs; leg_idx++) {
    legs[leg_idx] = leg;
  }

  // Hexapod body dimensions
  constexpr float length = 1.8f;
  constexpr float width = 1.0f;
  constexpr float depth = 0.2f;
  const hexapod::Hexapod::Dims hex_dims{length, width, depth};

  // Transformations between body frame and leg base frames
  // LHS leg base frames need to be rotated by 90, RHS by -90
  hexapod::Transform *tf_body_to_leg = new hexapod::Transform[num_legs];

  hexapod::Vector3 front(hex_dims.length / 2.0f, 0.0f, 0.0f);
  hexapod::Vector3 back = -front;
  hexapod::Vector3 left(0.0f, hex_dims.width / 2.0f, 0.0f);
  hexapod::Vector3 right = -left;
  hexapod::Vector3 middle(0.0f, 0.0f, 0.0f);

  // Legs MUST be ordered this way i.e. left then right, then next row left ...
  enum { FRONT_LEFT = 0, FRONT_RIGHT, MIDDLE_LEFT, MIDDLE_RIGHT, BACK_LEFT, BACK_RIGHT };

  tf_body_to_leg[FRONT_LEFT].t_ = front + left;
  tf_body_to_leg[FRONT_LEFT].R_.setRPYExtr(0, 0, M_PI / 3.0);

  tf_body_to_leg[MIDDLE_LEFT].t_ = middle + left;
  tf_body_to_leg[MIDDLE_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[BACK_LEFT].t_ = back + left;
  tf_body_to_leg[BACK_LEFT].R_.setRPYExtr(0, 0, 2.0 * M_PI / 3.0);

  tf_body_to_leg[FRONT_RIGHT].t_ = front + right;
  tf_body_to_leg[FRONT_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 3.0);

  tf_body_to_leg[MIDDLE_RIGHT].t_ = middle + right;
  tf_body_to_leg[MIDDLE_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[BACK_RIGHT].t_ = back + right;
  tf_body_to_leg[BACK_RIGHT].R_.setRPYExtr(0, 0, -2.0 * M_PI / 3.0);

  return Hexapod(num_legs, hex_dims, tf_body_to_leg, legs);
}
/**
 * @details This 'build' is specific to an actual physical robot where some of the servos are flipped / not in
 *  the same orientation as the internal hexapod model assumes, hence the need for flip/offset variables
 * @return
 */
hexapod::Hexapod buildPhantomX() {
  constexpr uint8_t num_joints = 3;
  hexapod::Leg::Dims leg_dims{0.05f, 0.066f, 0.132f};

  constexpr float kDegToRad = M_PI / 180.0;

  constexpr float joint_2_offset = 14.0 * kDegToRad;
  constexpr float joint_3_offset = 46.0 * kDegToRad;
  constexpr float joint_3_offset_mod = joint_3_offset - joint_2_offset;
  hexapod::Joint rhs_leg_joints[num_joints];
  rhs_leg_joints[0] = hexapod::Joint(-80.0f * kDegToRad, 80.0f * kDegToRad, 0.0, 0.0);
  rhs_leg_joints[1] = hexapod::Joint(-95.0f * kDegToRad, 95.0f * kDegToRad, joint_2_offset, joint_2_offset);
  rhs_leg_joints[2] = hexapod::Joint(-120.0f * kDegToRad, 88.0f * kDegToRad, joint_3_offset_mod, joint_3_offset_mod);
  hexapod::Leg rhs_leg(leg_dims, rhs_leg_joints);

  hexapod::Joint lhs_leg_joints[num_joints];
  lhs_leg_joints[0] = hexapod::Joint(-80.0f * kDegToRad, 80.0f * kDegToRad, 0.0, 0.0);
  lhs_leg_joints[1] = hexapod::Joint(-95.0f * kDegToRad, 95.0f * kDegToRad, -joint_2_offset, -joint_2_offset, true);
  lhs_leg_joints[2] =
      hexapod::Joint(-88.0f * kDegToRad, 120.0f * kDegToRad, -joint_3_offset_mod, -joint_3_offset_mod, true);
  hexapod::Leg lhs_leg(leg_dims, lhs_leg_joints);

  // Make an array of legs and copy the one we just made into all elements
  constexpr uint8_t num_legs = 6;
  hexapod::Leg *legs = new hexapod::Leg[num_legs];
  for (uint8_t leg_idx = 0; leg_idx < num_legs; leg_idx += 2) {
    legs[leg_idx] = lhs_leg;
    legs[leg_idx + 1] = rhs_leg;
  }

  // Hexapod body dimensions
  constexpr float length = 0.24f;
  constexpr float width = 0.12f;
  constexpr float width_mid = 0.20f;
  constexpr float depth = 0.07f;
  const hexapod::Hexapod::Dims hex_dims{length, width, depth};

  // Transformations between body frame and leg base frames
  // LHS leg base frames need to be rotated by 90, RHS by -90
  hexapod::Transform *tf_body_to_leg = new hexapod::Transform[num_legs];

  // Legs MUST be ordered this way i.e. left then right, then next row left ...
  enum { FRONT_LEFT = 0, FRONT_RIGHT, MIDDLE_LEFT, MIDDLE_RIGHT, BACK_LEFT, BACK_RIGHT };

  tf_body_to_leg[FRONT_LEFT].t_ = hexapod::Vector3{length / 2.0, width / 2.0, 0.0};
  tf_body_to_leg[FRONT_LEFT].R_.setRPYExtr(0, 0, M_PI / 4.0);

  tf_body_to_leg[MIDDLE_LEFT].t_ = hexapod::Vector3{0.0, width_mid / 2.0, 0.0};
  tf_body_to_leg[MIDDLE_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[BACK_LEFT].t_ = hexapod::Vector3{-length / 2.0, width / 2.0, 0.0};
  tf_body_to_leg[BACK_LEFT].R_.setRPYExtr(0, 0, M_PI * 3.0 / 4.0);

  tf_body_to_leg[FRONT_RIGHT].t_ = hexapod::Vector3{length / 2.0, -width / 2.0, 0.0};
  tf_body_to_leg[FRONT_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 4.0);

  tf_body_to_leg[MIDDLE_RIGHT].t_ = hexapod::Vector3{0.0, -width_mid / 2.0, 0.0};
  tf_body_to_leg[MIDDLE_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[BACK_RIGHT].t_ = hexapod::Vector3{-length / 2.0, -width / 2.0, 0.0};
  tf_body_to_leg[BACK_RIGHT].R_.setRPYExtr(0, 0, -M_PI * 3.0 / 4.0);

  return Hexapod(num_legs, hex_dims, tf_body_to_leg, legs);
}
/**
 * @details This 'build' assumes all joints are oriented as per the URDF (no flipping like on the physical model)
 * @return
 */
hexapod::Hexapod buildPhantomXForVis() {
  constexpr uint8_t num_joints = 3;
  hexapod::Leg::Dims leg_dims{0.05f, 0.066f, 0.132f};

  constexpr float kDegToRad = M_PI / 180.0;

  constexpr float joint_2_offset = 14.0 * kDegToRad;
  constexpr float joint_3_offset = 46.0 * kDegToRad;
  constexpr float joint_3_offset_mod = joint_3_offset - joint_2_offset;
  hexapod::Joint rhs_leg_joints[num_joints];
  // Start off with legs 'flat' / pointing directly out
  rhs_leg_joints[0] = hexapod::Joint(-80.0f * kDegToRad, 80.0f * kDegToRad, 0.0, 0.0);
  rhs_leg_joints[1] = hexapod::Joint(-95.0f * kDegToRad, 95.0f * kDegToRad, joint_2_offset, joint_2_offset);
  rhs_leg_joints[2] = hexapod::Joint(-120.0f * kDegToRad, 88.0f * kDegToRad, joint_3_offset_mod, joint_3_offset_mod);
  hexapod::Leg rhs_leg(leg_dims, rhs_leg_joints);

  hexapod::Joint lhs_leg_joints[num_joints];
  lhs_leg_joints[0] = hexapod::Joint(-80.0f * kDegToRad, 80.0f * kDegToRad, 0.0, 0.0);
  lhs_leg_joints[1] = hexapod::Joint(-95.0f * kDegToRad, 95.0f * kDegToRad, joint_2_offset, joint_2_offset);
  lhs_leg_joints[2] = hexapod::Joint(-120.0f * kDegToRad, 88.0f * kDegToRad, joint_3_offset_mod, joint_3_offset_mod);
  hexapod::Leg lhs_leg(leg_dims, lhs_leg_joints);

  // Make an array of legs and copy the one we just made into all elements
  constexpr uint8_t num_legs = 6;
  hexapod::Leg *legs = new hexapod::Leg[num_legs];
  for (uint8_t leg_idx = 0; leg_idx < num_legs; leg_idx += 2) {
    legs[leg_idx] = lhs_leg;
    legs[leg_idx + 1] = rhs_leg;
  }

  // Hexapod body dimensions
  constexpr float length = 0.24f;
  constexpr float width = 0.12f;
  constexpr float width_mid = 0.20f;
  constexpr float depth = 0.07f;
  const hexapod::Hexapod::Dims hex_dims{length, width, depth};

  // Transformations between body frame and leg base frames
  // LHS leg base frames need to be rotated by 90, RHS by -90
  hexapod::Transform *tf_body_to_leg = new hexapod::Transform[num_legs];

  // Legs MUST be ordered this way i.e. left then right, then next row left ...
  enum { FRONT_LEFT = 0, FRONT_RIGHT, MIDDLE_LEFT, MIDDLE_RIGHT, BACK_LEFT, BACK_RIGHT };

  tf_body_to_leg[FRONT_LEFT].t_ = hexapod::Vector3{length / 2.0, width / 2.0, 0.0};
  tf_body_to_leg[FRONT_LEFT].R_.setRPYExtr(0, 0, M_PI / 4.0);

  tf_body_to_leg[MIDDLE_LEFT].t_ = hexapod::Vector3{0.0, width_mid / 2.0, 0.0};
  tf_body_to_leg[MIDDLE_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[BACK_LEFT].t_ = hexapod::Vector3{-length / 2.0, width / 2.0, 0.0};
  tf_body_to_leg[BACK_LEFT].R_.setRPYExtr(0, 0, M_PI * 3.0 / 4.0);

  tf_body_to_leg[FRONT_RIGHT].t_ = hexapod::Vector3{length / 2.0, -width / 2.0, 0.0};
  tf_body_to_leg[FRONT_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 4.0);

  tf_body_to_leg[MIDDLE_RIGHT].t_ = hexapod::Vector3{0.0, -width_mid / 2.0, 0.0};
  tf_body_to_leg[MIDDLE_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[BACK_RIGHT].t_ = hexapod::Vector3{-length / 2.0, -width / 2.0, 0.0};
  tf_body_to_leg[BACK_RIGHT].R_.setRPYExtr(0, 0, -M_PI * 3.0 / 4.0);

  return Hexapod(num_legs, hex_dims, tf_body_to_leg, legs);
}
hexapod::Hexapod buildDefaultOctapod() {
  // Construct a leg (they are all the same in this default robot)
  constexpr uint8_t num_joints = 3;
  hexapod::Leg::Dims leg_dims{0.2f, 0.4f, 0.6f};

  hexapod::Joint joints[num_joints];
  joints[0] = hexapod::Joint(-90.0f * M_PI / 180.0, 90.0f * M_PI / 180.0, 0.0);
  joints[1] = hexapod::Joint(-150.0f * M_PI / 180.0, 150.0f * M_PI / 180.0, M_PI / 2.0);
  joints[2] = hexapod::Joint(-150.0f * M_PI / 10.0, 150.0f * M_PI / 180.0, M_PI / 4.0);

  hexapod::Leg leg(leg_dims, joints);

  // Make an array of legs and copy the one we just made into all elements
  constexpr uint8_t num_legs = 8;
  hexapod::Leg *legs = new hexapod::Leg[num_legs];
  for (uint8_t leg_idx = 0; leg_idx < num_legs; leg_idx++) {
    legs[leg_idx] = leg;
  }

  // Hexapod body dimensions
  constexpr float length = 1.8f;
  constexpr float width = 1.0f;
  constexpr float depth = 0.2f;
  const hexapod::Hexapod::Dims hex_dims{length, width, depth};

  // Transformations between body frame and leg base frames
  // LHS leg base frames need to be rotated by 90, RHS by -90
  hexapod::Transform *tf_body_to_leg = new hexapod::Transform[num_legs];

  hexapod::Vector3 front((hex_dims.length / 2.0f), 0.0f, 0.0f);
  hexapod::Vector3 front_mid((hex_dims.length / 2.0f) - (length / (num_legs / 2 - 1)), 0.0f, 0.0f);
  hexapod::Vector3 back_mid = -front_mid;
  hexapod::Vector3 back = -front;
  hexapod::Vector3 left(0.0f, hex_dims.width / 2.0f, 0.0f);
  hexapod::Vector3 right = -left;

  // Legs MUST be ordered this way i.e. left then right, then next row left ...
  enum {
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    FRONT_MIDDLE_LEFT,
    FRONT_MIDDLE_RIGHT,
    BACK_MIDDLE_LEFT,
    BACK_MIDDLE_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  };

  tf_body_to_leg[FRONT_LEFT].t_ = front + left;
  tf_body_to_leg[FRONT_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[FRONT_MIDDLE_LEFT].t_ = front_mid + left;
  tf_body_to_leg[FRONT_MIDDLE_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[BACK_MIDDLE_LEFT].t_ = back_mid + left;
  tf_body_to_leg[BACK_MIDDLE_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[BACK_LEFT].t_ = back + left;
  tf_body_to_leg[BACK_LEFT].R_.setRPYExtr(0, 0, M_PI / 2.0);

  tf_body_to_leg[FRONT_RIGHT].t_ = front + right;
  tf_body_to_leg[FRONT_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[FRONT_MIDDLE_RIGHT].t_ = front_mid + right;
  tf_body_to_leg[FRONT_MIDDLE_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[BACK_MIDDLE_RIGHT].t_ = back_mid + right;
  tf_body_to_leg[BACK_MIDDLE_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  tf_body_to_leg[BACK_RIGHT].t_ = back + right;
  tf_body_to_leg[BACK_RIGHT].R_.setRPYExtr(0, 0, -M_PI / 2.0);

  return Hexapod(num_legs, hex_dims, tf_body_to_leg, legs);
}

} // namespace hexapod
