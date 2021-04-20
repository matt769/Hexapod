#include "hexapod.h"
#include "build_hexapod.h"
#include "kinematics_support.h"
#include "transformations.h"
#include "visualisation.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace hexapod;

void demo_walk(Hexapod& hexapod);
void demo_turn(Hexapod& hexapod);
void demo_walk_turn(Hexapod& hexapod);
void demo_walk_turn_body(Hexapod& hexapod);
void demo_body(Hexapod& hexapod);
void demo_body_then_walk(Hexapod& hexapod);
void demo_nothing(Hexapod& hexapod);
void demo_body2(Hexapod& hexapod);
void demo_all(Hexapod& hexapod);
void demo_stand(Hexapod& hexapod);

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_walk");
  ros::NodeHandle nh;

  Hexapod hexapod = buildDefaultHexapod();
  Leg::JointAngles starting_angles{0.0, M_PI / 2.0, M_PI / 4.0};
  hexapod.setStartingAngles(starting_angles);
  Vis visualiser(nh, &hexapod);

  size_t sim_step_no = 0;
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    demo_all(hexapod);

    // move and update
    hexapod.update();
    visualiser.update();

    sim_step_no++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void demo_stand(Hexapod& hexapod) {
  static size_t sim_step = 0;

  if (sim_step == 100) {
    hexapod.setLegTargetsToGround();
  }

  if (hexapod.getState() == Hexapod::State::STANDING) {
    hexapod.riseToWalk();
  }
  if (hexapod.getState() == Hexapod::State::WALKING) {
    hexapod.setWalk(Vector3(0.002f, 0.0f, 0.0f), (0.0 * M_PI / 180.0) / 50.0);
  }

  sim_step++;
}

void demo_walk(Hexapod& hexapod) {
  Vector3 step(0.002f, 0.0f, 0.0f);
  hexapod.setWalk(step);
}

void demo_turn(Hexapod& hexapod) {
  static size_t sim_step_no = 0;
  if (sim_step_no > 100) {
    float angle_step = (5.0 * M_PI / 180.0) / 50.0;
    hexapod.setWalk(angle_step);
  }
  sim_step_no++;
}

void demo_walk_turn(Hexapod& hexapod) {
  static size_t sim_step_no = 0;
  if (sim_step_no == 200) {
    Vector3 small_step = Vector3(0.002f, 0.001f, 0.0f);
    hexapod.setWalk(small_step);
  } else if (sim_step_no == 400) {
    Vector3 small_step = Vector3(0.0f, 0.0f, 0.0f);
    float angle_step = (5.0 * M_PI / 180.0) / 50.0;
    hexapod.setWalk(small_step, angle_step);
  } else if (sim_step_no == 600) {
    Vector3 small_step = Vector3(-0.001f, -0.001f, 0.0f);
    float angle_step = (-5.0 * M_PI / 180.0) / 50.0;
    hexapod.setWalk(small_step, angle_step);
  }
  sim_step_no++;
}

void demo_walk_turn_body(Hexapod& hexapod) {
  static size_t sim_step_no = 0;
  static float t = 0.0f;
  static float r = 0.0f;
  static float r_inc = 0.05f * M_PI / 180.0f;
  static float t_inc = 0.001f;
  Transform tf_base_to_body_new;

  if (sim_step_no > 100) {
    t += t_inc;
    r += r_inc;
    if (fabs(t) > 0.02f) {
      t_inc *= -1.0f;
    }
    if (fabs(r) > 7.5f * M_PI / 180.0f) {
      r_inc *= -1.0f;
    }
    tf_base_to_body_new.R_.setRPYExtr(r, r, r);
    tf_base_to_body_new.t_(0) = t;
    tf_base_to_body_new.t_(1) = t;
    tf_base_to_body_new.t_(2) = t;
    // and walk at same time
    Vector3 small_step = Vector3(0.002f, 0.002f, 0.0f);
    float angle_step = (3.0 * M_PI / 180.0) / 50.0;
    hexapod.setWalk(small_step, angle_step);
    hexapod.setBody(tf_base_to_body_new);
  }
  sim_step_no++;
}

void demo_body(Hexapod& hexapod) {
  static size_t sim_step_no = 0;
  static float t = 0.0f;
  static float r = 0.0f;
  static float r_inc = 0.05f * M_PI / 180.0f;
  static float t_inc = 0.001f;
  Transform tf_base_to_body_new;

  if (sim_step_no > 100) {
    t += t_inc;
    r += r_inc;
    if (fabs(t) > 0.02f) {
      t_inc *= -1.0f;
    }
    if (fabs(r) > 5.0f * M_PI / 180.0f) {
      r_inc *= -1.0f;
    }
    tf_base_to_body_new.R_.setRPYExtr(r, r, r);
    tf_base_to_body_new.t_(0) = t;
    tf_base_to_body_new.t_(1) = t;
    tf_base_to_body_new.t_(2) = t;
  }
  hexapod.setBody(tf_base_to_body_new);
  sim_step_no++;
}

void demo_body_then_walk(Hexapod& hexapod) {
  static size_t sim_step_no = 0;
  static float t = 0.0f;
  static float r = 0.0f;
  Transform tf_base_to_body_new;

  if (sim_step_no <= 100) {
    hexapod.setBody(tf_base_to_body_new);
  } else if (sim_step_no <= 150) {
    t += 0.001f;
    r += (5.0 * M_PI / 180.0) / 50.0;
    tf_base_to_body_new.R_.setRPYExtr(r, r, r);
    tf_base_to_body_new.t_(0) = t;
    tf_base_to_body_new.t_(1) = t;
    tf_base_to_body_new.t_(2) = t;
    hexapod.setBody(tf_base_to_body_new);
  } else if (sim_step_no > 250) {
    Vector3 small_step = Vector3(0.002f, 0.0f, 0.0f);
    hexapod.setWalk(small_step);
  }
  sim_step_no++;
}

void demo_nothing() {}

void demo_body2(Hexapod& hexapod) {
  static float x_translation = 0.0;
  static float y_translation = 0.0;
  static float z_translation = 0.0;
  static float x_increment = 0.01;
  static float y_increment = 0.01;
  static float z_increment = 0.01;
  const float x_translation_limit = 0.06;
  const float y_translation_limit = 0.06;
  const float z_translation_limit = 0.06;

  static float x_angle = 0.0;
  static float y_angle = 0.0;
  static float z_angle = 0.0;
  static float x_angle_increment = 0.3 * M_PI / 180.0;
  static float y_angle_increment = 0.3 * M_PI / 180.0;
  static float z_angle_increment = 0.3 * M_PI / 180.0;
  const float x_angle_limit = 5.0 * M_PI / 180.0;
  const float y_angle_limit = 5.0 * M_PI / 180.0;
  const float z_angle_limit = 5.0 * M_PI / 180.0;

  if ((x_translation > x_translation_limit && x_increment > 0) ||
      (x_translation < -x_translation_limit && x_increment < 0)) {
    x_increment *= -1.0;
  }
  x_translation += x_increment;

  if ((y_translation > y_translation_limit && y_increment > 0) ||
      (y_translation < -y_translation_limit && y_increment < 0)) {
    y_increment *= -1.0;
  }
  y_translation += y_increment;

  if ((z_translation > z_translation_limit && z_increment > 0) ||
      (z_translation < -z_translation_limit && z_increment < 0)) {
    z_increment *= -1.0;
  }
  z_translation += z_increment;

  if ((x_angle > x_angle_limit && x_angle_increment > 0) ||
      (x_angle < -x_angle_limit && x_angle_increment)) {
    x_angle_increment *= -1.0;
  }
  x_angle += x_angle_increment;
  if ((y_angle > y_angle_limit && y_angle_increment > 0) ||
      (y_angle < -y_angle_limit && y_angle_increment)) {
    y_angle_increment *= -1.0;
  }
  y_angle += y_angle_increment;
  if ((z_angle > z_angle_limit && z_angle_increment > 0) ||
      (z_angle < -z_angle_limit && z_angle_increment)) {
    z_angle_increment *= -1.0;
  }
  z_angle += z_angle_increment;

  Transform tf_base_to_body_new;
  tf_base_to_body_new.R_.setRPYExtr(x_angle, y_angle, z_angle);

  tf_base_to_body_new.t_(0) = x_translation;
  tf_base_to_body_new.t_(1) = y_translation;
  tf_base_to_body_new.t_(2) = z_translation;
  hexapod.setBody(tf_base_to_body_new);
}

// update will start returning true when cycle limit is reached
class CycleValue {
 public:
  CycleValue(float val, float inc, float limit, size_t limit_cycles)
      : val_(val), start_val_(val_), inc_(inc), limit_(limit), limit_cycles_(limit_cycles){};
  float value() {
    if (finished()) {
      return val_;
    }
    if ((val_ > fabs(limit_) && inc_ > 0) || (val_ < -fabs(limit_) && inc_ < 0.0f)) {
      inc_ *= -1.0f;  // reverse direction
      times_reversed++;
    }
    val_ += inc_;
    return val_;
  };
  size_t cycles() {
    return times_reversed / 2;  // integer division
  };
  bool finished() { return (cycles() >= limit_cycles_) && (fabs(val_ - start_val_) < fabs(inc_)); };

 private:
  float val_ = 0.0f;
  float start_val_;
  float inc_ = 0.0f;
  float limit_ = 0.0f;
  size_t limit_cycles_ = 0;
  size_t times_reversed = 0;
};

void demo_all(Hexapod& hexapod) {
  static size_t sim_step_no = 0;
  static size_t sim_step_no_target = 0;
  static size_t stage = 0;
  static CycleValue x_pos(0.0f, 0.005f, 0.3f, 1);
  static CycleValue y_pos(0.0f, 0.005f, 0.3f, 1);
  static CycleValue z_pos(0.0f, 0.005f, 0.15f, 1);
  static CycleValue x_ang(0.0f, 0.3 * M_PI / 180.0, 15.0 * M_PI / 180.0, 1);
  static CycleValue y_ang(0.0f, 0.3 * M_PI / 180.0, 15.0 * M_PI / 180.0, 1);
  static CycleValue z_ang(0.0f, 0.3 * M_PI / 180.0, 15.0 * M_PI / 180.0, 1);

  static CycleValue x_pos2(0.0f, 0.005f, 0.2f, 2);
  static CycleValue y_pos2(0.0f, 0.005f, 0.2f, 2);
  static CycleValue z_pos2(0.0f, 0.005f, 0.1f, 2);
  static CycleValue x_ang2(0.0f, 0.3 * M_PI / 180.0, 7.0 * M_PI / 180.0, 3);
  static CycleValue y_ang2(0.0f, 0.3 * M_PI / 180.0, 7.0 * M_PI / 180.0, 3);
  static CycleValue z_ang2(0.0f, 0.3 * M_PI / 180.0, 7.0 * M_PI / 180.0, 3);

  static CycleValue x_pos3(0.0f, 0.003f, 0.1f, 2);
  static CycleValue y_pos3(0.0f, 0.003f, 0.1f, 2);
  static CycleValue z_pos3(0.0f, 0.003f, 0.1f, 2);
  static CycleValue x_ang3(0.0f, 0.1 * M_PI / 180.0, 5.0 * M_PI / 180.0, 3);
  static CycleValue y_ang3(0.0f, 0.1 * M_PI / 180.0, 5.0 * M_PI / 180.0, 3);
  static CycleValue z_ang3(0.0f, 0.1 * M_PI / 180.0, 5.0 * M_PI / 180.0, 3);

  static Transform tf_base_to_body_new;

  if (stage == 0 && sim_step_no == 100) {
    hexapod.setLegTargetsToGround();
  }
  if (stage == 0 && hexapod.getState() == Hexapod::State::STANDING) {
    hexapod.riseToWalk();
  }
  if (stage == 0 && hexapod.getState() == Hexapod::State::WALKING) {
    stage++;
  }

  // Start moving the body around a bit
  // Single dimensions at a time
  if (stage == 1) {
    tf_base_to_body_new.t_(2) = z_pos.value();
    hexapod.setBody(tf_base_to_body_new);
    if (z_pos.finished()) stage++;
  }
  if (stage == 2) {
    tf_base_to_body_new.t_(1) = y_pos.value();
    hexapod.setBody(tf_base_to_body_new);
    if (y_pos.finished()) stage++;
  }
  if (stage == 3) {
    tf_base_to_body_new.t_(0) = x_pos.value();
    hexapod.setBody(tf_base_to_body_new);
    if (x_pos.finished()) stage++;
  }
  if (stage == 4) {
    tf_base_to_body_new.R_.setRPYExtr(x_ang.value(), 0, 0);
    hexapod.setBody(tf_base_to_body_new);
    if (x_ang.finished()) stage++;
  }
  if (stage == 5) {
    tf_base_to_body_new.R_.setRPYExtr(0, y_ang.value(), 0);
    hexapod.setBody(tf_base_to_body_new);
    if (y_ang.finished()) stage++;
  }
  if (stage == 6) {
    tf_base_to_body_new.R_.setRPYExtr(0, 0, z_ang.value());
    hexapod.setBody(tf_base_to_body_new);
    if (z_ang.finished()) stage++;
  }
  // All dimensions at once
  if (stage == 7) {
    tf_base_to_body_new.t_(0) = x_pos2.value();
    tf_base_to_body_new.t_(1) = y_pos2.value();
    tf_base_to_body_new.t_(2) = z_pos2.value();
    tf_base_to_body_new.R_.setRPYExtr(x_ang2.value(), y_ang2.value(), z_ang2.value());
    hexapod.setBody(tf_base_to_body_new);
    if (x_ang2.finished() && y_ang2.finished() && z_ang2.finished() && x_pos2.finished() &&
        y_pos2.finished() && z_pos2.finished()) {
      stage++;
    }
  }
  // wait
  if (stage == 8) {
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 50;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }

  if (stage == 9) {
    hexapod.changeStanceWidth(0.2);
    stage++;
  }
  if (stage == 10) {
    hexapod.setWalk(Vector3(0.0, 0, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 100;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }

  // Walk
  if (stage == 11) {
    hexapod.setWalk(Vector3(0.003, 0, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 200;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // pause
  if (stage == 12) {
    hexapod.setWalk(Vector3(0.0, 0, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 150;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // backwards
  if (stage == 13) {
    hexapod.setWalk(Vector3(-0.003, 0, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 200;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // pause
  if (stage == 14) {
    hexapod.setWalk(Vector3(0.0, 0, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 250;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // side
  if (stage == 15) {
    hexapod.setWalk(Vector3(0.0, -0.003, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 200;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // and back
  if (stage == 16) {
    hexapod.setWalk(Vector3(0.0, 0.003, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 300;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // angled
  if (stage == 17) {
    hexapod.setWalk(Vector3(-0.002, -0.002, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 300;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // Turn 135ish
  if (stage == 18) {
    hexapod.setWalk((7.0 * M_PI / 180.0) / 50.0);
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 300;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  if (stage == 19) {
    hexapod.setWalk((15.0 * M_PI / 180.0) / 50.0);
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 300;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // Walk forward
  if (stage == 20) {
    hexapod.setWalk(Vector3(0.004, 0, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 150;
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // Change gait
  if (stage == 21) {
    hexapod.setWalk(Vector3(0.004, 0, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 300;
      hexapod.changeGait(Hexapod::Gait::LEFT_RIGHT_LEFT_RIGHT);
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // Change gait
  if (stage == 22) {
    hexapod.setWalk(Vector3(0.004, 0, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 300;
      hexapod.changeGait(Hexapod::Gait::LHS_THEN_RHS);
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // Change gait
  if (stage == 23) {
    hexapod.setWalk(Vector3(0.004, 0, 0));
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 300;
      hexapod.changeGait(Hexapod::Gait::AROUND_THE_CLOCK);
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }
  }
  // Walk and turn
  if (stage == 24) {
    if (sim_step_no_target < sim_step_no) {
      sim_step_no_target = sim_step_no + 600;
      hexapod.changeGait(Hexapod::Gait::RIPPLE);
    }
    if (sim_step_no == sim_step_no_target) {
      stage++;
      sim_step_no_target = 0;
    }

    hexapod.setWalk(Vector3(0.004, -0.002, 0), (5.0 * M_PI / 180.0) / 50.0);
    if (x_ang3.finished() && y_ang3.finished() && z_ang3.finished() && x_pos3.finished() &&
        y_pos3.finished() && z_pos3.finished()) {
      stage++;
    }
  }
  // Everything
  if (stage == 25) {
    hexapod.setWalk(Vector3(0.004, -0.002, 0), (5.0 * M_PI / 180.0) / 50.0);
    tf_base_to_body_new.t_(0) = x_pos3.value();
    tf_base_to_body_new.t_(1) = y_pos3.value();
    tf_base_to_body_new.t_(2) = z_pos3.value();
    tf_base_to_body_new.R_.setRPYExtr(x_ang3.value(), y_ang3.value(), z_ang3.value());
    hexapod.setBody(tf_base_to_body_new);
    if (x_ang3.finished() && y_ang3.finished() && z_ang3.finished() && x_pos3.finished() &&
        y_pos3.finished() && z_pos3.finished()) {
      stage++;
    }
  }

  // TODO add change leg travel ratio
  // TODO add change leg lift height

  if (stage == 26) {
    hexapod.setWalk(Vector3(0.008, -0.008, 0));
  }

  sim_step_no++;
}