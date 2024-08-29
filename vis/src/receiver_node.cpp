#include "ros_receiver.h"
#include "visualisation.h"
#include "build_from_urdf.h"

#include <hexapod_core/hexapod.h>
#include <hexapod_core/build_hexapod.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>


using namespace hexapod_vis;
using namespace hexapod;
using namespace std::chrono_literals;

// Taken from https://answers.ros.org/question/408470/is-there-a-way-to-load-urdf-from-parameter-in-ros2/
std::string get_urdf_string() {
  auto node = std::make_shared<rclcpp::Node>("temp_node");

  auto parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(node, "/robot_state_publisher");
  while (!parameters_client_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  std::string urdf_string;
  auto parameters = parameters_client_->get_parameters({ "robot_description" });
  for (auto& parameter : parameters)
  {
    if (parameter.get_name() == "robot_description")
    {
      urdf_string = parameter.value_to_string();
      break;
    }
  }

  return urdf_string;
}


int main(int argc, char **argv) {
//  ros::init(argc, argv, "receiver_node");
//  ros::NodeHandle nh;
  rclcpp::init(argc, argv);

//   Hexapod hexapod = buildDefaultHexapod();
//   Hexapod hexapod = buildDefaultHexapod2();
//   Hexapod hexapod = buildDefaultOctapod();
//   Hexapod hexapod = buildFromURDF(get_urdf_string());
//  Hexapod hexapod = buildPhantomX();
  Hexapod hexapod = buildPhantomXForVis();
  auto vis_node = std::make_shared<Vis>(&hexapod);
  auto receiver_node = std::make_shared<RosReceiver>(&hexapod);

  // HOW TO SPIN THESE?

  rclcpp::Rate loop_rate(50);
  // TODO proper shutdown conditions
  while (rclcpp::ok()) {
    hexapod.update();
    vis_node->update();

    rclcpp::spin(receiver_node);
    loop_rate.sleep();
  }

  return 0;
}
