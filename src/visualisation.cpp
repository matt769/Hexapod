#include "visualisation.h"

#include "transformations.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace Transformations;

namespace Vis {

void initialise(const Hexapod& hexapod, tf2_ros::TransformBroadcaster* tf_br,
                ros::Publisher* joints_pub) {
  // this tf doesn't exist yet so we need to explicitly create it
  //  (which updateVisWorld doesn't do )
  geometry_msgs::TransformStamped tf_world_to_base;
  tf_world_to_base.header.stamp = ros::Time::now();
  tf_world_to_base.header.frame_id = "world";
  tf_world_to_base.child_frame_id = "base_link";
  tf_world_to_base.transform.translation.x = 0.0;
  tf_world_to_base.transform.translation.y = 0.0;
  tf_world_to_base.transform.translation.z = hexapod.getHeight();
  tf2::Quaternion q2(0.0, 0.0, 0.0, 1.0);
  tf_world_to_base.transform.rotation.x = q2.x();
  tf_world_to_base.transform.rotation.y = q2.y();
  tf_world_to_base.transform.rotation.z = q2.z();
  tf_world_to_base.transform.rotation.w = q2.w();
  tf_br->sendTransform(tf_world_to_base);

  Vis::updateVisBody(hexapod, tf_br);
  Vis::updateVisJoints(hexapod, joints_pub);
}

void updateVisJoints(const Hexapod& hexapod, ros::Publisher* joints_pub) {
  constexpr size_t total_joints = 18;
  std::vector<std::string> joint_names(total_joints);
  std::vector<double> joint_angles(total_joints);
  joint_names[0] = "leg_front_left_joint_1";
  joint_names[1] = "leg_front_left_joint_2";
  joint_names[2] = "leg_front_left_joint_3";
  joint_names[3] = "leg_front_right_joint_1";
  joint_names[4] = "leg_front_right_joint_2";
  joint_names[5] = "leg_front_right_joint_3";
  joint_names[6] = "leg_middle_left_joint_1";
  joint_names[7] = "leg_middle_left_joint_2";
  joint_names[8] = "leg_middle_left_joint_3";
  joint_names[9] = "leg_middle_right_joint_1";
  joint_names[10] = "leg_middle_right_joint_2";
  joint_names[11] = "leg_middle_right_joint_3";
  joint_names[12] = "leg_back_left_joint_1";
  joint_names[13] = "leg_back_left_joint_2";
  joint_names[14] = "leg_back_left_joint_3";
  joint_names[15] = "leg_back_right_joint_1";
  joint_names[16] = "leg_back_right_joint_2";
  joint_names[17] = "leg_back_right_joint_3";

  joint_angles[0] = hexapod.getLeg(Hexapod::FRONT_LEFT).getJointAngles().theta_1;
  joint_angles[1] = hexapod.getLeg(Hexapod::FRONT_LEFT).getJointAngles().theta_2;
  joint_angles[2] = hexapod.getLeg(Hexapod::FRONT_LEFT).getJointAngles().theta_3;
  joint_angles[3] = hexapod.getLeg(Hexapod::FRONT_RIGHT).getJointAngles().theta_1;
  joint_angles[4] = hexapod.getLeg(Hexapod::FRONT_RIGHT).getJointAngles().theta_2;
  joint_angles[5] = hexapod.getLeg(Hexapod::FRONT_RIGHT).getJointAngles().theta_3;
  joint_angles[6] = hexapod.getLeg(Hexapod::MIDDLE_LEFT).getJointAngles().theta_1;
  joint_angles[7] = hexapod.getLeg(Hexapod::MIDDLE_LEFT).getJointAngles().theta_2;
  joint_angles[8] = hexapod.getLeg(Hexapod::MIDDLE_LEFT).getJointAngles().theta_3;
  joint_angles[9] = hexapod.getLeg(Hexapod::MIDDLE_RIGHT).getJointAngles().theta_1;
  joint_angles[10] = hexapod.getLeg(Hexapod::MIDDLE_RIGHT).getJointAngles().theta_2;
  joint_angles[11] = hexapod.getLeg(Hexapod::MIDDLE_RIGHT).getJointAngles().theta_3;
  joint_angles[12] = hexapod.getLeg(Hexapod::BACK_LEFT).getJointAngles().theta_1;
  joint_angles[13] = hexapod.getLeg(Hexapod::BACK_LEFT).getJointAngles().theta_2;
  joint_angles[14] = hexapod.getLeg(Hexapod::BACK_LEFT).getJointAngles().theta_3;
  joint_angles[15] = hexapod.getLeg(Hexapod::BACK_RIGHT).getJointAngles().theta_1;
  joint_angles[16] = hexapod.getLeg(Hexapod::BACK_RIGHT).getJointAngles().theta_2;
  joint_angles[17] = hexapod.getLeg(Hexapod::BACK_RIGHT).getJointAngles().theta_3;

  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  msg.name = joint_names;
  msg.position = joint_angles;
  joints_pub->publish(msg);
}

void updateVisBody(const Hexapod& hexapod, tf2_ros::TransformBroadcaster* tf_br) {
  const Transform tf_btb = hexapod.getBaseToBody();
  // convert to tf matrix3x3
  tf2::Matrix3x3 r;
  r.setValue(tf_btb.R_(0, 0), tf_btb.R_(0, 1), tf_btb.R_(0, 2), tf_btb.R_(1, 0), tf_btb.R_(1, 1),
             tf_btb.R_(1, 2), tf_btb.R_(2, 0), tf_btb.R_(2, 1), tf_btb.R_(2, 2));
  // extract quaternion from it using tf2 implementation
  tf2::Quaternion q;
  r.getRotation(q);

  geometry_msgs::TransformStamped tf_base_to_body;
  tf_base_to_body.header.stamp = ros::Time::now();
  tf_base_to_body.header.frame_id = "base_link";
  tf_base_to_body.child_frame_id = "body_link";
  tf_base_to_body.transform.translation.x = tf_btb.t_(0);
  tf_base_to_body.transform.translation.y = tf_btb.t_(1);
  tf_base_to_body.transform.translation.z = tf_btb.t_(2);
  tf_base_to_body.transform.rotation.x = q.x();
  tf_base_to_body.transform.rotation.y = q.y();
  tf_base_to_body.transform.rotation.z = q.z();
  tf_base_to_body.transform.rotation.w = q.w();
  tf_br->sendTransform(tf_base_to_body);
}

void updateVisWorld(const Hexapod& hexapod, tf2_ros::TransformBroadcaster* tf_br,
                    tf2_ros::Buffer* tf_buffer) {
  // extract movement from hexapod
  const Transform tf_b_nb = hexapod.getBaseMovement();  // for brevity, base to new base
  // convert to tf matrix3x3
  tf2::Matrix3x3 r;
  r.setValue(tf_b_nb.R_(0, 0), tf_b_nb.R_(0, 1), tf_b_nb.R_(0, 2), tf_b_nb.R_(1, 0), tf_b_nb.R_(1, 1),
             tf_b_nb.R_(1, 2), tf_b_nb.R_(2, 0), tf_b_nb.R_(2, 1), tf_b_nb.R_(2, 2));
  // extract quaternion from it using tf2 implementation
  tf2::Quaternion q_b_nb;
  r.getRotation(q_b_nb);
  // construct full tf2 transform
  tf2::Transform tf_base_to_new_base;
  tf_base_to_new_base.setRotation(q_b_nb);
  tf2::Vector3 t_b_nb(tf_b_nb.t_(0), tf_b_nb.t_(1), tf_b_nb.t_(2));
  tf_base_to_new_base.setOrigin(t_b_nb);

  // current world to base
  geometry_msgs::TransformStamped tf_world_to_base_msg =
      tf_buffer->lookupTransform("world", "base_link", ros::Time(0));
  tf2::Stamped<tf2::Transform> tf_world_to_base;
  tf2::fromMsg(tf_world_to_base_msg, tf_world_to_base);

  // new world to base
  tf2::Transform tf_world_to_new_base = tf_world_to_base * tf_base_to_new_base;
  tf_world_to_base_msg.transform.rotation.x = tf_world_to_new_base.getRotation().x();
  tf_world_to_base_msg.transform.rotation.y = tf_world_to_new_base.getRotation().y();
  tf_world_to_base_msg.transform.rotation.z = tf_world_to_new_base.getRotation().z();
  tf_world_to_base_msg.transform.rotation.w = tf_world_to_new_base.getRotation().w();
  tf_world_to_base_msg.transform.translation.x = tf_world_to_new_base.getOrigin().x();
  tf_world_to_base_msg.transform.translation.y = tf_world_to_new_base.getOrigin().y();
  tf_world_to_base_msg.transform.translation.z = tf_world_to_new_base.getOrigin().z();
  tf_world_to_base_msg.header.frame_id = "world";
  tf_world_to_base_msg.child_frame_id = "base_link";
  tf_world_to_base_msg.header.stamp = ros::Time::now();

  tf_br->sendTransform(tf_world_to_base_msg);
}

}  // namespace Vis