#include "receiver.h"
#include "build_from_urdf.h"
#include "hexapod.h"
#include "kinematics_support.h"
#include "transformations.h"
#include "visualisation.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace Transformations;

int main(int argc, char **argv) {
  ros::init(argc, argv, "receiver_node");
  ros::NodeHandle nh;

  // Hexapod hexapod = buildDefaultHexapod();
  // Hexapod hexapod = buildDefaultHexapod2();
  // Hexapod hexapod = buildDefaultOctapod();
  Hexapod hexapod = BuildFromURDF::buildFromURDF();
  // Hexapod hexapod = buildPhantomX();
  Vis visualiser(nh, &hexapod);
  Receiver receiver(nh, &hexapod);

  ros::Rate loop_rate(50);
  while (ros::ok()) {
    receiver.update();
    hexapod.update();
    visualiser.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
