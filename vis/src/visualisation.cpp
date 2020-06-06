#include "visualisation.h"

#include "transformations.h"
#include "hexapod.h"
#include "leg.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>

using namespace Transformations;


Vis::Vis(const ros::NodeHandle& nh, Hexapod* hexapod) 
  : nh_(nh), hexapod_(hexapod), num_legs_(hexapod->num_legs_)

{
  const size_t number_of_joints = num_legs_ * 3;
  joint_names_.resize(number_of_joints);
  joint_angles_.resize(number_of_joints);
  generateJointNames();

  joints_pub_ = nh_.advertise<sensor_msgs::JointState >("joint_states", 1);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
  initialiseTransforms();
  ros::Duration(1).sleep(); // make sure tf is ready when we query it shortly
}

void Vis::initialiseTransforms() {
  // this tf doesn't exist yet so we need to explicitly create it
  //  (which updateVisWorld doesn't do )
  geometry_msgs::TransformStamped tf_world_to_base;
  tf_world_to_base.header.stamp = ros::Time::now();
  tf_world_to_base.header.frame_id = "world";
  tf_world_to_base.child_frame_id = "base_link";
  tf_world_to_base.transform.translation.x = 0.0;
  tf_world_to_base.transform.translation.y = 0.0;
  tf_world_to_base.transform.translation.z = hexapod_->getHeight();
  tf2::Quaternion q2(0.0, 0.0, 0.0, 1.0);
  tf_world_to_base.transform.rotation.x = q2.x();
  tf_world_to_base.transform.rotation.y = q2.y();
  tf_world_to_base.transform.rotation.z = q2.z();
  tf_world_to_base.transform.rotation.w = q2.w();
  tf_br_.sendTransform(tf_world_to_base);

  updateBody();
  updateJoints();
}

/**
 * @brief 
 * @details
 * Assumes that leg indicies are allocated from front to back, left to right
 * i.e. front left = 0, front right = 1, next row back left = 2, right = 3 etc
 * so for legs on the left, index % 2 == 0
 * The front row is named 'front', back row is named 'back'
 * And all others are 'rowN' where N starts at 1 (next to the front) and increases towards the back
 * The naming MUST match the naming used in the associated URDF
 */
void Vis::generateJointNames() {
  for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    std::string position, side;

    const size_t row_idx = leg_idx / 2; // integer division
    std::string row_str = std::to_string(row_idx);
    // bit of faffing around to left pad the row number
    const size_t pad_size = 2;
    row_str.insert(row_str.begin(), pad_size-row_str.size(),'0');
    position = "row" + row_str;

    if(leg_idx % 2 == 0) {
      side = "left";
    }
    else {
      side = "right";
    }
    std::string leg_name = "leg_" + position + "_" + side + "_";

    joint_names_.at(3*leg_idx + 0) = leg_name + "joint_1";
    joint_names_.at(3*leg_idx + 1) = leg_name + "joint_2";
    joint_names_.at(3*leg_idx + 2) = leg_name + "joint_3";
  }
}


void Vis::updateJoints() {
  
  for (size_t leg_idx = 0; leg_idx < num_legs_; leg_idx++) {
    const Leg::JointAngles leg_joint_angles = hexapod_->getLeg(leg_idx).getJointAngles();
    joint_angles_.at(3*leg_idx + 0) = leg_joint_angles.theta_1;
    joint_angles_.at(3*leg_idx + 1) = leg_joint_angles.theta_2;
    joint_angles_.at(3*leg_idx + 2) = leg_joint_angles.theta_3;
  }

  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  msg.name = joint_names_;
  msg.position = joint_angles_;
  joints_pub_.publish(msg);
}

void Vis::updateBody() {
  const Transform tf_btb = hexapod_->getBaseToBody();
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
  tf_br_.sendTransform(tf_base_to_body);
}

void Vis::updateWorld() {
  // extract movement from hexapod_
  const Transform tf_b_nb = hexapod_->getBaseMovement();  // for brevity, base to new base
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
      tf_buffer_.lookupTransform("world", "base_link", ros::Time(0));
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

  tf_br_.sendTransform(tf_world_to_base_msg);
}


void Vis::update() {
  updateJoints();
  updateWorld();
  updateBody();
}