#include "build_from_urdf.h"

#include "hexapod.h"
#include "leg.h"
#include "transformations.h"

#include <urdf/model.h>

// I could have just got these transforms from tf, right?...

using namespace Transformations;

namespace BuildFromURDF {

Hexapod buildFromURDF() {
  urdf::Model urdf_model;
  urdf_model.initParam("robot_description");

  urdf::LinkConstSharedPtr body_link = urdf_model.getLink("body_link");
  std::cout << body_link->name << '\n';

  // TODO add check

  std::vector<urdf::LinkSharedPtr> leg_base_links = body_link->child_links;
  size_t num_legs = leg_base_links.size();

  std::map<std::string, std::pair<Transform, Leg::Dims>> legs_ordered;

  for (const auto& leg_base : leg_base_links) {
    // get body to base transformation
    urdf::Pose pose = leg_base->parent_joint->parent_to_joint_origin_transform;
    Transform body_to_leg_tf;
    body_to_leg_tf.R_ = QuaternionToRotationMatrix(
        Quaternion(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z));
    body_to_leg_tf.t_ = Vector3(pose.position.x, pose.position.y, pose.position.z);

    // get leg dimensions
    urdf::LinkSharedPtr leg1 = (leg_base->child_links).at(0);
    urdf::LinkSharedPtr leg2 = (leg1->child_links).at(0);
    urdf::LinkSharedPtr foot = (leg2->child_links).at(0);
    Leg::Dims dims;
    dims.a = leg1->parent_joint->parent_to_joint_origin_transform.position.x;
    dims.b = leg2->parent_joint->parent_to_joint_origin_transform.position.x;
    dims.c = foot->parent_joint->parent_to_joint_origin_transform.position.x;

    legs_ordered.emplace(std::string(leg_base->name), std::make_pair(body_to_leg_tf, dims));
  }

  // starting angles are fixed here - perhaps not ideal
  Joint joints[3];
  joints[0] = Joint(-90.0f * M_PI / 180.0, 90.0f * M_PI / 180.0, 0.0);
  joints[1] = Joint(-150.0f * M_PI / 180.0, 150.0f * M_PI / 180.0, M_PI / 2.0);
  joints[2] = Joint(-150.0f * M_PI / 10.0, 150.0f * M_PI / 180.0, M_PI / 4.0);

  // put it into arrays required for Hexapod constructor
  Transform* leg_tfs_for_build = new Transform[num_legs];
  Leg* legs = new Leg[num_legs];
  std::vector<std::string> joint_names;
  size_t idx = 0;
  for (const auto& leg_info : legs_ordered) {
    leg_tfs_for_build[idx] = leg_info.second.first;
    legs[idx] = Leg(leg_info.second.second, joints);
    joint_names.push_back(leg_info.first);
    idx++;
  }

  // bit hacky but assume hexapod dimensions based on legs (although the only one that actually
  // matters is depth)
  Transform front_left = leg_tfs_for_build[0];
  Transform front_right = leg_tfs_for_build[1];
  Transform back_right = leg_tfs_for_build[num_legs - 1];
  float length = (front_right.t_ - back_right.t_).norm();
  float width = (front_left.t_ - front_right.t_).norm();
  float depth = width / 5.0f;
  Hexapod::Dims hex_dims = {length, width, depth};
  std::cout << "Building robot with " << num_legs << " legs." << '\n';
  std::cout << "Body dimensions " << length << " x " << width << " x " << depth << '\n';
  std::cout << "Leg link lengths:" << '\n';
  for (size_t idx = 0; idx < num_legs; idx++) {
    std::cout << "Leg " << idx << '\n';
    std::cout << "Dimensions: " << legs[idx].dims_.a << ", " << legs[idx].dims_.b << ", "
              << legs[idx].dims_.c << '\n';
    std::cout << "Located "
              << ":" << leg_tfs_for_build[idx].t_.x() << ", " << leg_tfs_for_build[idx].t_.y()
              << ", " << leg_tfs_for_build[idx].t_.z() << '\n';
    std::cout << "Based on urdf joint: " << joint_names.at(idx) << '\n';
  }

  return Hexapod(num_legs, hex_dims, std::move(leg_tfs_for_build), std::move(legs));
}

}  // namespace