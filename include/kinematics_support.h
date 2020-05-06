#ifndef KINEMATICS_SUPPORT_H
#define KINEMATICS_SUPPORT_H

#include "transformations.h"

namespace KinematicsSupport {
static constexpr float eps = 0.0001;
float wrapAngle(float angle);
bool comparePositions(const Transformations::Vector3& a, const Transformations::Vector3& b);
bool compareFloat(float a, float b);
bool compareFloat(float a, float b, float tolerance);
bool clamp(float& val, float lower, float upper);
}

#endif