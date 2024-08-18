#ifndef HEX_RECEIVER_H
#define HEX_RECEIVER_H

#include <hexapod_core/hexapod.h>
#include <hexapod_core/transformations.h>
#include <hexapod_core/receiver.h>



#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <std_msgs/Int32.h>

namespace hexapod_vis {

class RosReceiver : public hexapod::Receiver {
 public:
  RosReceiver(const ros::NodeHandle& nh, hexapod::Hexapod *hexapod);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber input_sub_;

  void callbackProcessKeyPress(const std_msgs::Int32::ConstPtr& msg);
};

} // namespace hexapod_vis

#endif