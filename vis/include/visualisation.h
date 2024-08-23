#ifndef VIS_HEX_H
#define VIS_HEX_H

#include <hexapod_core/hexapod.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hexapod_vis {

class Vis : public rclcpp::Node {
 public:
  hexapod::Hexapod *const hexapod_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr foot_traj_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr movement_limits_marker_pub_;
  size_t num_legs_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_angles_;

  Vis(hexapod::Hexapod *hexapod);
  void initialiseTransforms();
  void generateJointNames();
  void updateJoints();
  void updateWorld();
  void updateBody();
  void update();
  void publishFootTrajectories();
  void publishMovementLimits();
};

} // namespace hexapod

#endif