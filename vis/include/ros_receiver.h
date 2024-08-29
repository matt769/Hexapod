#ifndef HEX_RECEIVER_H
#define HEX_RECEIVER_H

#include <hexapod_core/hexapod.h>
#include <hexapod_core/transformations.h>
#include <hexapod_core/receiver.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace hexapod_vis {

class RosReceiver : public rclcpp::Node, public hexapod::Receiver {
 public:
  RosReceiver(hexapod::Hexapod *hexapod);

 private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr input_sub_;

  void callbackProcessKeyPress(const std_msgs::msg::Int32& msg);
};

} // namespace hexapod_vis

#endif