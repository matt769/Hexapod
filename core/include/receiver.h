#ifndef HEXAPOD_RECEIVER_H
#define HEXAPOD_RECEIVER_H

#include "hexapod.h"
#include "transformations.h"

namespace hexapod {

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
  explicit Receiver(Hexapod *hexapod);
  void setRobot(Hexapod *hexapod);
  void update();
  void processCommand(uint8_t cmd);

 private:
  Hexapod *hexapod_ = nullptr;
  Vector3 walk_increment_fb;
  Vector3 walk_increment_lr;
  Vector3 manual_fb;
  Vector3 manual_lr;
  Vector3 manual_ud;
};

} // namespace hexapod

#endif // HEXAPOD_RECEIVER_H