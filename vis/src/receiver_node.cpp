#include "hexapod.h"
#include "kinematics_support.h"
#include "transformations.h"
#include "visualisation.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int32.h>

using namespace Transformations;

// Hexapod hexapod = buildDefaultHexapod();
Hexapod hexapod = buildDefaultHexapod2();
// Hexapod hexapod = buildDefaultOctapod();
constexpr float walk_increment = 0.001f;
const Vector3 walk_increment_fb{walk_increment, 0.0f, 0.0f};
const Vector3 walk_increment_lr{0.0f, walk_increment, 0.0f};
Vector3 current_walk{0.0f, 0.0f, 0.0f};
const float turn_increment{0.03f * M_PI / 180.0f};
float current_turn{0.0f};
const float body_rotation_increment{1.0f * M_PI / 180.0};
const float body_translation_increment{0.005f};
const float stance_width_increment = 0.05f;
const float ftgr_increment = 0.1f;
const float leg_raise_increment = 0.01f;

void callbackProcessKeyPress(const std_msgs::Int32::ConstPtr& msg);

int main(int argc, char **argv) {

  ros::init(argc, argv, "demo_walk");
  ros::NodeHandle nh;

  ros::Subscriber command_input = nh.subscribe("hexapod/command_key", 10, callbackProcessKeyPress);

  Vis visualiser(nh, &hexapod);

  size_t sim_step_no = 0;
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    hexapod.setWalk(current_walk, current_turn);
    hexapod.update();  
    visualiser.update();

    sim_step_no++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

void callbackProcessKeyPress(const std_msgs::Int32::ConstPtr& msg) {
  int32_t keyCode = msg->data;
  std::cout << "Key code pressed: " << msg->data << '\n';

  Transform body_change;

  switch (keyCode) {
    case 119:
      current_walk = current_walk + walk_increment_fb;
      break;
    case 115:
      current_walk = current_walk - walk_increment_fb;
      break;
    case 97:
      current_walk = current_walk + walk_increment_lr;
      break;
    case 100:
      current_walk = current_walk - walk_increment_lr;
      break;
    case 113:
      current_turn += turn_increment;
      break;
    case 101:
      current_turn -= turn_increment;
      break;
    case 120:
      current_walk = Vector3(0.0f, 0.0f, 0.0f);
      current_turn = 0.0f;
      break;

    case 116:
      body_change.t_(0) += body_translation_increment;
      hexapod.changeBody(body_change);
      break;
    case 103:
      body_change.t_(0) -= body_translation_increment;
      hexapod.changeBody(body_change);
      break;
    case 102:
      body_change.t_(1) += body_translation_increment;
      hexapod.changeBody(body_change);
      break;
    case 104:
      body_change.t_(1) -= body_translation_increment;
      hexapod.changeBody(body_change);
      break;
    case 114:
      body_change.t_(2) += body_translation_increment;
      hexapod.changeBody(body_change);
      break;
    case 121:
      body_change.t_(2) -= body_translation_increment;
      hexapod.changeBody(body_change);
      break;
    case 98:
      hexapod.setBody(body_change); // TODO implement resetBody()
      break;

    case 105:
      body_change.R_.setRPYExtr(0.0f, body_rotation_increment, 0.0f);
      hexapod.changeBody(body_change);
      break;
    case 107:
      body_change.R_.setRPYExtr(0.0f, -body_rotation_increment, 0.0f);
      hexapod.changeBody(body_change);
      break;
    case 106:
      body_change.R_.setRPYExtr(body_rotation_increment, 0.0f, 0.0f);
      hexapod.changeBody(body_change);
      break;
    case 108:
      body_change.R_.setRPYExtr(-body_rotation_increment, 0.0f, 0.0f);
      hexapod.changeBody(body_change);
      break;
    case 117:
      body_change.R_.setRPYExtr(0.0f, 0.0f, body_rotation_increment);
      hexapod.changeBody(body_change);
      break;
    case 111:
      body_change.R_.setRPYExtr(0.0f, 0.0f, -body_rotation_increment);
      hexapod.changeBody(body_change);
      break;
    case 44:
      hexapod.setBody(body_change); // TODO implement resetBody()
      break;

    case 49:
      hexapod.changeGait(Hexapod::Gait::RIPPLE);
      break;
    case 50:
      hexapod.changeGait(Hexapod::Gait::LEFT_RIGHT_LEFT_RIGHT);
      break;
    case 51:
      hexapod.changeGait(Hexapod::Gait::LHS_THEN_RHS);
      break;
    case 52:
      hexapod.changeGait(Hexapod::Gait::AROUND_THE_CLOCK);
      break;

    case 93:
      hexapod.changeStanceWidth(stance_width_increment);
      break;
    case 91:
      hexapod.changeStanceWidth(-stance_width_increment);
      break;
    case 59:
      hexapod.resetStanceWidth();
      break;

    // Change foot ground travel ratio
    case 35:
      hexapod.changeFootGroundTravelRatio(ftgr_increment);
      break;
    case 39:
      hexapod.changeFootGroundTravelRatio(-ftgr_increment);
      break;
    case 47:
      hexapod.resetFootGroundTravelRatio();
      break;

    // Change leg raise height
    case 61:
      hexapod.changeLegRaiseHeight(leg_raise_increment);
      break;
    case 45:
      hexapod.changeLegRaiseHeight(-leg_raise_increment);
      break;
    case 48:
      hexapod.resetLegRaiseHeight();
      break;

    case 127: // backspace
      hexapod.setMoveMode(Hexapod::MoveMode::STANDARD);
      break;
    case 10: // enter
      hexapod.setMoveMode(Hexapod::MoveMode::HEADLESS);
      break;

    case 96: // backtick
      if (hexapod.getState() == Hexapod::State::UNSUPPORTED) {
        hexapod.setLegsToGround();
      }
      if (hexapod.getState() == Hexapod::State::STANDING) {
        hexapod.riseToWalk();
      }
      break;


    default:
      std::cout << "Unrecognised key\n";
  }

}