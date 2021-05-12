#ifndef VIS_HEX_H
#define VIS_HEX_H

#include "hexapod.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace hexapod_vis {

class Vis {
 public:
  ros::NodeHandle nh_;
  hexapod::Hexapod *const hexapod_;
  tf2_ros::TransformBroadcaster tf_br_;
  tf2_ros::Buffer tf_buffer_;  // should this be outside the class? ot static?
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  ros::Publisher joints_pub_;
  ros::Publisher foot_traj_marker_pub_;
  size_t num_legs_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_angles_;

  Vis(const ros::NodeHandle& nh, hexapod::Hexapod *hexapod);
  void initialiseTransforms();
  void generateJointNames();
  void updateJoints();
  void updateWorld();
  void updateBody();
  void update();
  void publishFootTrajectories();
};

} // namespace hexapod

#endif