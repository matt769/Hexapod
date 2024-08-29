#include "ros_receiver.h"

#include <hexapod_core/hexapod.h>
#include <hexapod_core/transformations.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace hexapod_vis {
using namespace hexapod;

RosReceiver::RosReceiver(Hexapod *hexapod) : Node("hexapod_receiver_node"), Receiver(hexapod)   {
//  input_sub_ = nh_.subscribe("hexapod/command_key", 10, &RosReceiver::callbackProcessKeyPress, this);
//  input_sub_ = this->create_subscription<std_msgs::msg::Int32>(
//      "hexapod/command_key", 10, RosReceiver::callbackProcessKeyPress, this);
  input_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "hexapod/command_key", 10, std::bind(&RosReceiver::callbackProcessKeyPress, this, std::placeholders::_1));
}

void RosReceiver::callbackProcessKeyPress(const std_msgs::msg::Int32& msg) {
  int32_t keyCode = msg.data;
  std::cout << "Key code pressed: " << msg.data << '\n';
  processCommand((uint8_t)keyCode);
}

} // namespace hexapod

