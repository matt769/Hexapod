#ifndef VIS_HEX_H
#define VIS_HEX_H

#include "hexapod.h"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace Vis {

void initialise(const Hexapod& hexapod, tf2_ros::TransformBroadcaster* tf_br,
                ros::Publisher* joints_pub);
void updateVisJoints(const Hexapod& hexapod, ros::Publisher* joints_pub);
void updateVisWorld(const Hexapod& hexapod, tf2_ros::TransformBroadcaster* tf_br,
                    tf2_ros::Buffer* tf_buffer);
void updateVisBody(const Hexapod& hexapod, tf2_ros::TransformBroadcaster* tf_br);

}  // namespace Vis

#endif