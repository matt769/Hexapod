#ifndef KINEMATICS_SUPPORT_H
#define KINEMATICS_SUPPORT_H

#include "transformations.h"

#ifdef __AVR__
#include <Arduino.h>
#endif

namespace hexapod {
namespace util {

static constexpr float eps = 0.0001;
float wrapAngle(float angle);
bool comparePositions(const Vector3& a, const Vector3& b);
bool compareFloat(float a, float b);
bool compareFloat(float a, float b, float tolerance);
bool approxEqual(Vector3 a, Vector3 b, float tolerance);
bool clamp(float& val, float lower, float upper);

} // namespace util
} // namespace hexapod

#endif