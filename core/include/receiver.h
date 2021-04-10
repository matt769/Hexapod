#ifndef HEX_RECEIVER_H
#define HEX_RECEIVER_H

#include "hexapod.h"
#include "transformations.h"

namespace Tfm = Transformations;

/**
 * @brief Translates control input into Hexapod commands
 * @details Takes in single character input i.e. from a keyboard
 *  and calls Hexapod functions to control the robot.
 * Contains a small amount of state to facilitate keeping the robot moving (the hexapod currently
 * requires input every period or it won't move at all, with a few exceptions)
 */
class Receiver {
 public:
  Receiver();
  explicit Receiver(Hexapod* hexapod);
  void setRobot(Hexapod* hexapod);
  void update();
  void processCommand(uint8_t cmd);

 private:
  Hexapod* hexapod_ = nullptr;
  const float walk_increment = 0.0001f;
  const Tfm::Vector3 walk_increment_fb{walk_increment, 0.0f, 0.0f};
  const Tfm::Vector3 walk_increment_lr{0.0f, walk_increment, 0.0f};
  const Tfm::Vector3 manual_fb{0.001, 0.0f, 0.0f};
  const Tfm::Vector3 manual_lr{0.0f, 0.001, 0.0f};
  const Tfm::Vector3 manual_ud{0.0f, 0.0f, 0.001};
  const float manual_joint{0.025f};
  Tfm::Vector3 current_walk{0.0f, 0.0f, 0.0f};
  const float turn_increment{0.03f * M_PI / 180.0f};
  float current_turn{0.0f};
  const float body_rotation_increment{1.0f * M_PI / 180.0};
  const float body_translation_increment{0.0005f};
  const float stance_width_increment = 0.005f;
  const float ftgr_increment = 0.01f;
  const float leg_raise_increment = 0.001f;
};

#endif