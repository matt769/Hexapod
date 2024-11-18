#include "../include/hexapod_core/joint.h"

#include <catch2/catch.hpp>

//#include <iostream>

using namespace hexapod;

// We'll use degrees everywhere because the joint class doesn't actually have any concept of units

// Physical joint with limits -50, 100
// Physical joint angle 0 matches model 0 (no offset),
//  and the rotation is expected the same way (no flip axis)
TEST_CASE( "Joint simple") {
  const float starting_physical_angle = 0;
  const float offset = 0.0;
  const float physical_lower_limit = -50;
  const float physical_upper_limit = 100;
  const bool flip = false;
  const float model_lower_limit = -50;
  const float model_upper_limit = 100;
  // Confirm I'm not doing anything weird with setup
  REQUIRE(physical_upper_limit - physical_lower_limit == model_upper_limit - model_lower_limit);

  Joint j(physical_lower_limit,physical_upper_limit, starting_physical_angle, offset, flip);

  // The physical angle we generated the joint with is unchanged
  REQUIRE(j.toPhysicalAngle() == starting_physical_angle);
  // And it has the correct relationship with the model angle
  REQUIRE(j.angle_ == starting_physical_angle - offset);

  // The physical-model relationship is fixed regardless of input
  for (float a = -180.0; a <= 180.0; a += 60.0) {
    REQUIRE(j.fromPhysicalAngle(a) == a);
    REQUIRE(j.toPhysicalAngle(a) == a);
  }
  // Models angles outside the model limits are clamped to the limits
  REQUIRE(j.clampToLimts(model_lower_limit - 1.0) == model_lower_limit);
  REQUIRE(j.clampToLimts(model_upper_limit + 1.0) == model_upper_limit);
  // Model angles equal to the model limits are clamped to the limits / unchanged
  REQUIRE(j.clampToLimts(model_lower_limit) == model_lower_limit);
  REQUIRE(j.clampToLimts(model_upper_limit) == model_upper_limit);
  // Model angles between the limits are unchanged
  for (float a = model_lower_limit; a < model_upper_limit; a += 1.0) {
    REQUIRE(j.clampToLimts(a) == a);
  }

  // Changing the angle by specifying the physical angle results in the correct model angle
  for (float new_physical_angle = -180.0; new_physical_angle <= 180.0; new_physical_angle += 60.0) {
    j.setFromPhysicalAngle(new_physical_angle);
    REQUIRE(j.angle_ == new_physical_angle);
    REQUIRE(j.toPhysicalAngle() == new_physical_angle);
  }
}


// Example
// Physical joint with limits -50, 100
// Physical joint is at angle 20 when model is at 0
//  i.e. phys = model + 20    or model = phys - 20
//  and the rotation is expected the same way (no flip axis)
// So the model limits are -70, 80
TEST_CASE( "Joint offset") {
  const float starting_physical_angle = 0;
  const float offset = 20.0;
  const float physical_lower_limit = -50;
  const float physical_upper_limit = 100;
  const bool flip = false;
  const float model_lower_limit = -70;
  const float model_upper_limit = 80;
  // Confirm I'm not doing anything weird with setup
  REQUIRE(physical_upper_limit - physical_lower_limit == model_upper_limit - model_lower_limit);

  Joint j(physical_lower_limit,physical_upper_limit, starting_physical_angle, offset, flip);

  // The physical angle we generated the joint with is unchanged
  REQUIRE(j.toPhysicalAngle() == starting_physical_angle);
  // And it has the correct relationship with the model angle
  REQUIRE(j.angle_ == starting_physical_angle - offset);

  // The physical-model relationship is fixed regardless of input
  for (float a = -180.0; a <= 180.0; a += 60.0) {
    REQUIRE(j.fromPhysicalAngle(a) == a - offset);
    REQUIRE(j.toPhysicalAngle(a) == a + offset);
  }
  // Models angles outside the model limits are clamped to the limits
  REQUIRE(j.clampToLimts(model_lower_limit - 1.0) == model_lower_limit);
  REQUIRE(j.clampToLimts(model_upper_limit + 1.0) == model_upper_limit);
  // Model angles equal to the model limits are clamped to the limits / unchanged
  REQUIRE(j.clampToLimts(model_lower_limit) == model_lower_limit);
  REQUIRE(j.clampToLimts(model_upper_limit) == model_upper_limit);
  // Model angles between the limits are unchanged
  for (float a = model_lower_limit; a < model_upper_limit; a += 1.0) {
    REQUIRE(j.clampToLimts(a) == a);
  }

  // Changing the angle by specifying the physical angle results in the correct model angle
  for (float new_physical_angle = -180.0; new_physical_angle <= 180.0; new_physical_angle += 60.0) {
    j.setFromPhysicalAngle(new_physical_angle);
    REQUIRE(j.angle_ == new_physical_angle - offset);
    REQUIRE(j.toPhysicalAngle() == new_physical_angle);
  }
}

TEST_CASE( "Joint flip") {
  const float starting_physical_angle = 0;
  const float offset = 0.0;
  const float physical_lower_limit = -50;
  const float physical_upper_limit = 100;
  const bool flip = true;
  const float model_lower_limit = -100;
  const float model_upper_limit = 50;
  // Confirm I'm not doing anything weird with setup
  REQUIRE(physical_upper_limit - physical_lower_limit == model_upper_limit - model_lower_limit);

  Joint j(physical_lower_limit,physical_upper_limit, starting_physical_angle, offset, flip);

  // The physical angle we generated the joint with is unchanged
  REQUIRE(j.toPhysicalAngle() == starting_physical_angle);
  // And it has the correct relationship with the model angle
  REQUIRE(j.angle_ == starting_physical_angle - offset);

  // The physical-model relationship is fixed regardless of input
  for (float a = -180.0; a <= 180.0; a += 60.0) {
    REQUIRE(j.fromPhysicalAngle(a) == -a);
    REQUIRE(j.toPhysicalAngle(a) == -a);
  }
  // Models angles outside the model limits are clamped to the limits
  REQUIRE(j.clampToLimts(model_lower_limit - 1.0) == model_lower_limit);
  REQUIRE(j.clampToLimts(model_upper_limit + 1.0) == model_upper_limit);
  // Model angles equal to the model limits are clamped to the limits / unchanged
  REQUIRE(j.clampToLimts(model_lower_limit) == model_lower_limit);
  REQUIRE(j.clampToLimts(model_upper_limit) == model_upper_limit);
  // Model angles between the limits are unchanged
  for (float a = model_lower_limit; a < model_upper_limit; a += 1.0) {
    REQUIRE(j.clampToLimts(a) == a);
  }

  // Changing the angle by specifying the physical angle results in the correct model angle
  for (float new_physical_angle = -180.0; new_physical_angle <= 180.0; new_physical_angle += 60.0) {
    j.setFromPhysicalAngle(new_physical_angle);
    REQUIRE(j.angle_ == -new_physical_angle);
    REQUIRE(j.toPhysicalAngle() == new_physical_angle);
  }
}

TEST_CASE( "Joint offset flip") {
  const float starting_physical_angle = 0;
  const float offset = 20.0;
  const float physical_lower_limit = -50;
  const float physical_upper_limit = 100;
  const bool flip = true;
  const float model_lower_limit = -80;
  const float model_upper_limit = 70;
  // Confirm I'm not doing anything weird with setup
  REQUIRE(physical_upper_limit - physical_lower_limit == model_upper_limit - model_lower_limit);

  Joint j(physical_lower_limit,physical_upper_limit, starting_physical_angle, offset, flip);

  // The physical angle we generated the joint with is unchanged
  REQUIRE(j.toPhysicalAngle() == starting_physical_angle);
  // And it has the correct relationship with the model angle
  REQUIRE(j.angle_ == -(starting_physical_angle - offset));

  // The physical-model relationship is fixed regardless of input
  for (float a = -180.0; a <= 180.0; a += 60.0) {
    REQUIRE(j.fromPhysicalAngle(a) == -(a - offset));
    REQUIRE(j.toPhysicalAngle(a) == (-a) + offset);
  }
  // Models angles outside the model limits are clamped to the limits
  REQUIRE(j.clampToLimts(model_lower_limit - 1.0) == model_lower_limit);
  REQUIRE(j.clampToLimts(model_upper_limit + 1.0) == model_upper_limit);
  // Model angles equal to the model limits are clamped to the limits / unchanged
  REQUIRE(j.clampToLimts(model_lower_limit) == model_lower_limit);
  REQUIRE(j.clampToLimts(model_upper_limit) == model_upper_limit);
  // Model angles between the limits are unchanged
  for (float a = model_lower_limit; a < model_upper_limit; a += 1.0) {
    REQUIRE(j.clampToLimts(a) == a);
  }

  // Changing the angle by specifying the physical angle results in the correct model angle
  for (float new_physical_angle = -180.0; new_physical_angle <= 180.0; new_physical_angle += 60.0) {
    j.setFromPhysicalAngle(new_physical_angle);
    REQUIRE(j.angle_ == -(new_physical_angle - offset));
    REQUIRE(j.toPhysicalAngle() == new_physical_angle);
  }
}

