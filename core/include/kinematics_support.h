#ifndef KINEMATICS_SUPPORT_H
#define KINEMATICS_SUPPORT_H

#include "transformations.h"

namespace Tfm = Transformations;

namespace KinematicsSupport {
static constexpr float eps = 0.0001;
float wrapAngle(float angle);
bool comparePositions(const Tfm::Vector3& a, const Tfm::Vector3& b);
bool compareFloat(float a, float b);
bool compareFloat(float a, float b, float tolerance);
bool approxEqual(Tfm::Vector3 a, Tfm::Vector3 b, float tolerance);
bool clamp(float& val, float lower, float upper);
}

#endif