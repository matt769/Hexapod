#include "hexapod.h"
#include "kinematics_support.h"
#include "transformations.h"
#include "visualisation.h"
#include "receiver.h"

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

int main(int argc, char **argv) {

  ros::init(argc, argv, "demo_walk");
  ros::NodeHandle nh;
  
  Hexapod hexapod = buildDefaultHexapod2();
  Vis visualiser(nh, &hexapod);
  Receiver receiver(nh, &hexapod);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    receiver.update();
    hexapod.update();  
    visualiser.update();

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
