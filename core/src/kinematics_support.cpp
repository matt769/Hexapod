#include "kinematics_support.h"

#include "transformations.h"

#include <cmath>

using namespace Transformations;

namespace KinematicsSupport {

float wrapAngle(float angle) {
  while (angle > M_PI) {
    angle -= M_PI;
  }
  while (angle < -M_PI) {
    angle += M_PI;
  }
  return angle;
}

bool comparePositions(const Vector3& a, const Vector3& b) {
  return compareFloat(a.x(), b.x()) && compareFloat(a.y(), b.y()) && compareFloat(a.z(), b.z());
}

bool compareFloat(float a, float b) { return compareFloat(a, b, eps); }

bool compareFloat(float a, float b, float tolerance) { return fabs(a - b) < tolerance; }

bool approxEqual(Vector3 a, Vector3 b, float tolerance) {
  return fabs((a - b).norm()) < tolerance;
}

bool clamp(float& val, const float lower, const float upper) {
  if (val < (lower - eps) || val > (upper + eps)) {
    return false;
  } else {
    val = fmin(fmax(val, lower), upper);
    return true;
  }
}
}