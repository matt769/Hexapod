#ifndef HEX_RECEIVER_H
#define HEX_RECEIVER_H

#include "hexapod.h"
#include "transformations.h"

#include <ros/ros.h>
#include <std_msgs/Int32.h>

namespace Tfm = Transformations;

class Receiver {
 public:
  Receiver(const ros::NodeHandle& nh, Hexapod* hexapod);
  void update();

 private:
  ros::NodeHandle nh_;
  Hexapod* const hexapod_;
  ros::Subscriber input_sub_;

  const float walk_increment = 0.001f;
  const Tfm::Vector3 walk_increment_fb{walk_increment, 0.0f, 0.0f};
  const Tfm::Vector3 walk_increment_lr{0.0f, walk_increment, 0.0f};
  const Tfm::Vector3 manual_fb{0.01, 0.0f, 0.0f};
  const Tfm::Vector3 manual_lr{0.0f, 0.01, 0.0f};
  const Tfm::Vector3 manual_ud{0.0f, 0.0f, 0.01};
  const float manual_joint{0.025f};
  Tfm::Vector3 current_walk{0.0f, 0.0f, 0.0f};
  const float turn_increment{0.03f * M_PI / 180.0f};
  float current_turn{0.0f};
  const float body_rotation_increment{1.0f * M_PI / 180.0};
  const float body_translation_increment{0.005f};
  const float stance_width_increment = 0.05f;
  const float ftgr_increment = 0.1f;
  const float leg_raise_increment = 0.01f;

  void callbackProcessKeyPress(const std_msgs::Int32::ConstPtr& msg);
};

#endif