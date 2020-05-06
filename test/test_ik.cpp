#include <stdlib.h>
#include "hexapod.h"
#include "kinematics_support.h"
#include "ros/ros.h"
#include "transformations.h"

// This file is pretty outdated

using namespace KinematicsSupport;
using namespace Transformations;

bool testIK(float a, float b, float c, bool verbose);
bool testIK(float a, float b, float c);

int main() {
  Leg test_leg;

  // specific cases
  testIK(0, 0, 0);
  testIK(0.5, 0, 0);
  testIK(0, 0.5, 0);
  testIK(0.5, 0.5, 0.5);
  testIK(0.1, 0.2, 0.3);
  testIK(0.645772, -1.09956, -1.20428);
  testIK(0.139626, 1.32645, 1.01229);
  testIK(0.453786, -1.18682, -1.23918);
  testIK(0.645772, -1.09956, -1.20428);
  testIK(0.139626, 1.32645, 1.01229);
  testIK(0, 1.32645, 1.01229);
  testIK(0, 1.0821, 0.872665);
  testIK(0, -1.16937, -1.29154);
  testIK(0, -1.3439, -1.29154);
  testIK(0, -1.16937, -1.29154);
  testIK(1.06465, 0.733038, 0.471239);
  testIK(-1.5708, 2.32129, 0);
  testIK(1.18682, -2.1293, -2.35619);
  testIK(-0.645772, 2.35619, 1.8675);
  testIK(-1.06465, -1.67552, -2.37365);
  testIK(-0.488692, -2.14675, -2.25147);
  testIK(0.628319, 2.33874, 1.72788);
  testIK(-0.296706, 2.25147, 1.8675);

  // a load of random cases
  srand(0);

  size_t fail_cnt = 0;
  size_t test_num = 1000;
  int angle_range = 280;
  for (size_t i = 0; i < test_num; i++) {
    // joint1 range of 170 degrees, centred around 0
    float a = (float)((rand() % 170) - 85) * M_PI / 180.0f;
    float b = (float)((rand() % angle_range) - angle_range / 2) * M_PI / 180.0f;
    float c = (float)((rand() % angle_range) - angle_range / 2) * M_PI / 180.0f;
    if (!testIK(a, b, c, false)) {
      std::cout << "FAIL TEST " << test_num << ":" << a << ", " << b << ", " << c << '\n';
      fail_cnt++;
    };
  }

  std::cout << "Of " << test_num << " random tests, " << fail_cnt << " failed.\n";
  return 0;
}

bool testIK(float a, float b, float c, bool verbose) {
  static int test_run = 0;
  test_run++;
  if (verbose) std::cout << "******** Test " << test_run << " ***********\n";

  Leg test_leg;
  Leg::JointAngles test_angles{a, b, c};
  if (verbose)
    std::cout << "Test angle: " << test_angles.theta_1 << ", " << test_angles.theta_2 << ", "
              << test_angles.theta_3 << '\n';
  Vector3 pos, pos_result_1, pos_result_2;
  test_leg.calculateFootPosition(test_angles, pos);
  if (verbose) std::cout << "Position: " << pos.x() << ", " << pos.y() << ", " << pos.z() << '\n';

  Leg::JointAngles result_angles[2];
  size_t num_results = test_leg.calculateJointAngles(pos, result_angles);

  if (num_results == 0) {
    if (verbose) std::cout << "No results.\n";
    return false;
  } else {
    if (verbose) std::cout << "Number of results: " << num_results << '\n';
  }

  std::string result;
  bool correct_result = false;

  if (compareFloat(a, result_angles[0].theta_1) && compareFloat(b, result_angles[0].theta_2) &&
      compareFloat(c, result_angles[0].theta_3)) {
    correct_result = true;
  }

  if (compareFloat(a, result_angles[1].theta_1) && compareFloat(b, result_angles[1].theta_2) &&
      compareFloat(c, result_angles[1].theta_3)) {
    correct_result = true;
  }

  if (correct_result) {
    if (verbose) std::cout << "PASS. Correct angles found in one of the options.\n";
    return true;
  } else {
    if (verbose)
      std::cout << "Angles 1: " << result_angles[0].theta_1 << ", " << result_angles[0].theta_2
                << ", " << result_angles[0].theta_3 << '\n';
    test_leg.calculateFootPosition(result_angles[0], pos_result_1);
    if (verbose)
      std::cout << "Position: " << pos_result_1.x() << ", " << pos_result_1.y() << ", "
                << pos_result_1.z() << '\n';
    if (verbose)
      std::cout << "Angles 2: " << result_angles[1].theta_1 << ", " << result_angles[1].theta_2
                << ", " << result_angles[1].theta_3 << '\n';
    test_leg.calculateFootPosition(result_angles[1], pos_result_2);
    if (verbose)
      std::cout << "Position: " << pos_result_2.x() << ", " << pos_result_2.y() << ", "
                << pos_result_2.z() << '\n';
    return false;
  }
}

bool testIK(float a, float b, float c) { return testIK(a, b, c, false); }