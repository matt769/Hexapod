#include "ros_receiver.h"

#include "hexapod.h"
#include "transformations.h"

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <std_msgs/Int32.h>

namespace hexapod_vis {
using namespace hexapod;

RosReceiver::RosReceiver(const ros::NodeHandle& nh, Hexapod *hexapod) : Receiver(hexapod), nh_(nh)  {
  input_sub_ = nh_.subscribe("hexapod/command_key", 10, &RosReceiver::callbackProcessKeyPress, this);
}

void RosReceiver::callbackProcessKeyPress(const std_msgs::Int32::ConstPtr& msg) {
  int32_t keyCode = msg->data;
  std::cout << "Key code pressed: " << msg->data << '\n';
  processCommand((uint8_t)keyCode);
}

} // namespace hexapod

