#include <hexapod.h>
#include <ps4.h>
#include <transformations.h>

using namespace hexapod;  // Wouldn't normally do this in header, but this is only a header because
                          // using arduino

class PS4Receiver {
 public:
  explicit PS4Receiver(Hexapod* hexapod) { setRobot(hexapod); }

  PS4Receiver() : PS4Receiver(nullptr) {}

  void setRobot(Hexapod* hexapod) {
    hexapod_ = hexapod;
    walk_increment = hexapod_->walk_translation_increment_;
    walk_turn_increment = hexapod_->walk_turn_increment_;
    walk_increment_fb = Vector3{walk_increment, 0.0f, 0.0f};
    walk_increment_lr = Vector3{0.0f, walk_increment, 0.0f};
    manual_fb = Vector3{0.001, 0.0f, 0.0f};
    manual_lr = Vector3{0.0f, 0.001, 0.0f};
    manual_ud = Vector3{0.0f, 0.0f, 0.001};
  }

  /**
   * @brief Requires a specific sequence of inputs before it will return true
   * @details Left stick should start centered, then be moved all the way down
   *  then all the way up, then back to the centre.
   */
  bool arm(const ps4::PS4_data& data) {
    bool static first = false;
    bool static second = false;
    bool static third = false;
    bool static forth = false;

    if (!first && data.l_joystick_y < kJoystickMid + kJoystickDeadzone &&
        data.l_joystick_y > kJoystickMid - kJoystickDeadzone) {
      first = true;
      return false;
    }
    if (first && !second && data.l_joystick_y > 200) {
      second = true;
      return false;
    }
    if (second && !third && data.l_joystick_y < 50) {
      third = true;
      return false;
    }
    if (third && !forth && data.l_joystick_y < kJoystickMid + kJoystickDeadzone &&
        data.l_joystick_y > kJoystickMid - kJoystickDeadzone) {
      forth = true;
      return true;
    }
    return first && second && third && forth;
  }

  void processCommand(const ps4::PS4_data& ps4_data) {
    if (!hexapod_) return;

    if (ps4_data.button_l2 && ps4_data.button_r2 && ps4_data.button_x) {
      hexapod_->setWalk(Vector3(0, 0, 0));  // TODO should be done in hexapod when setting manual
                                            // control? or is this not required at all?
      hexapod_->setFullManualControl(true);
      hexapod_->setManualLegControl(0);
      Serial.println(F("Manual leg control"));
      // TODO currently we don't provide a way to go back to normal control
    } else if (ps4_data.button_l2 && ps4_data.button_r2 && ps4_data.button_circle) {
      hexapod_->setWalk(Vector3(0, 0, 0));  // TODO should be done in hexapod when setting manual
                                            // control? or is this not required at all?
      hexapod_->setFullManualControl(true);
      hexapod_->setManualLegControl(0);
      hexapod_->setManualJointControl(0);
      Serial.println(F("Manual joint control"));
      // TODO currently we don't provide a way to go back to normal control
    }

    if (hexapod_->getState() == Hexapod::State::FULL_MANUAL) {
      // TODO Perhaps the MANUAL control states are a bit odd? FULL_MANUAL doesn't really mean much
      // on it's own - will always need to check ManualControlType
      Hexapod::ManualControlType mct = hexapod_->getManualControlType();
      uint8_t current_leg_idx = hexapod_->getManualControlLegIdx();
      uint8_t current_joint_idx = hexapod_->getManualControlJointIdx();

      // TODO only supporting the single leg mode to start
      if ((mct == Hexapod::ManualControlType::SINGLE_LEG || mct == Hexapod::ManualControlType::SINGLE_JOINT) &&
          (ps4_data.button_up || ps4_data.button_down)) {
        // Handle changing legs
        if (ps4_data.button_up && current_leg_idx < hexapod_->num_legs_ - 1) {
          current_leg_idx += 1;
        } else if (ps4_data.button_down && current_leg_idx > 0) {
          current_leg_idx -= 1;
        }
        hexapod_->setManualLegControl(current_leg_idx);
        // Really need to improve hexapod interface - changing the leg will set mode back to
        // SINGLE_LEG, so if in need to go back to  joint mode
        //  if that's what we were meant to be in
        if (mct == Hexapod::ManualControlType::SINGLE_JOINT) {
          hexapod_->setManualJointControl(current_joint_idx);
        }
        Serial.print(F("Manual control. Leg "));
        Serial.println(hexapod_->getManualControlLegIdx());
      }

      if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
        const float manual_foot_movement_increment = 0.01;
        Vector3 foot_movement = Vector3();
        if (ps4_data.l_joystick_x < kJoystickMid - kJoystickDeadzone) {
          foot_movement.x() = -manual_foot_movement_increment;
        } else if (ps4_data.l_joystick_x > kJoystickMid + kJoystickDeadzone) {
          foot_movement.x() = manual_foot_movement_increment;
        }

        if (ps4_data.l_joystick_y < kJoystickMid - kJoystickDeadzone) {
          foot_movement.y() = -manual_foot_movement_increment;
        } else if (ps4_data.l_joystick_y > kJoystickMid + kJoystickDeadzone) {
          foot_movement.y() = manual_foot_movement_increment;
        }

        if (ps4_data.r_joystick_y < kJoystickMid - kJoystickDeadzone) {
          foot_movement.z() = -manual_foot_movement_increment;
        } else if (ps4_data.r_joystick_y > kJoystickMid + kJoystickDeadzone) {
          foot_movement.z() = manual_foot_movement_increment;
        }

        hexapod_->manualMoveFoot(foot_movement);
      }

      if (mct == Hexapod::ManualControlType::SINGLE_JOINT) {
        // Hexpod doesn't support setting all joints manually in a single timestemp unfortunately
        // So have to choose one to set
        if (ps4_data.button_left || ps4_data.button_right) {
          if (ps4_data.button_right && current_joint_idx < Leg::NUM_JOINTS - 1) {
            current_joint_idx += 1;
          } else if (ps4_data.button_left && current_joint_idx > 0) {
            current_joint_idx -= 1;
          }
          hexapod_->setManualJointControl(current_joint_idx);
          Serial.print(F("Manual control. Joint "));
          Serial.println(hexapod_->getManualControlJointIdx());
        }

        const float manual_joint_angle_increment = 0.02;
        float joint_change = 0.0;

        if (ps4_data.r_joystick_y < kJoystickMid - kJoystickDeadzone) {
          joint_change = -manual_joint_angle_increment;
        } else if (ps4_data.r_joystick_y > kJoystickMid + kJoystickDeadzone) {
          joint_change = manual_joint_angle_increment;
        }

        hexapod_->manualChangeJoint(joint_change);
      }

      // TODO Change so that we can hold different buttons to adjust different joints i.e. don't
      // need to explicitly toggle
      // TODO test this works so far in practice

      return;
    }

    // Rotational part will be set. Translational part will be incremented.
    Transform body_change = hexapod_->getBaseToBody();
    //    ps4::printData(&Serial, ps4_data);

    // START UP / SHUTDOWN
    if (ps4_data.button_ps) {
      if (hexapod_->getState() == Hexapod::State::UNSUPPORTED) {
        hexapod_->setAllLegTargetsToGround(50);
        Serial.println(F("setAllLegTargetsToGround"));
      } else if (hexapod_->getState() == Hexapod::State::STANDING) {
        hexapod_->riseToWalk();
        Serial.println(F("riseToWalk"));
      } else if (hexapod_->getState() == Hexapod::State::WALKING) {
        hexapod_->lowerToGround();
        Serial.println(F("lowerToGround"));
      }
      return;
    }

    if (hexapod_->getState() != Hexapod::State::WALKING) {
      return;
    }

    // MOVEMENT PARAMETERS
    if (ps4_data.button_x) {
      hexapod_->changeLegRaiseTime(-hexapod_->leg_raise_time_increment_);  // faster
    }
    if (ps4_data.button_square) {
      hexapod_->changeLegRaiseTime(hexapod_->leg_raise_time_increment_);
    }
    if (ps4_data.button_circle) {
      hexapod_->changeLegRaiseHeight(hexapod_->leg_raise_increment_);
    }
    if (ps4_data.button_triangle) {
      hexapod_->changeLegRaiseHeight(-hexapod_->leg_raise_increment_);
    }

    if (!ps4_data.button_l1) {
      if (ps4_data.button_up) {
        hexapod_->changeFootGroundTravelRatio(hexapod_->ftgr_increment_);
      } else if (ps4_data.button_down) {
        hexapod_->changeFootGroundTravelRatio(-hexapod_->ftgr_increment_);
      }
      if (ps4_data.button_left) {
        hexapod_->changeStanceWidth(-hexapod_->stance_width_increment_);
      } else if (ps4_data.button_right) {
        hexapod_->changeStanceWidth(hexapod_->stance_width_increment_);
      }
    }

    // MOVEMENT MODE
    // This will need to be debounced
    //    if(ps4_data.button_share) {
    //      Serial.println("sq");
    //      hexapod_->setMoveMode(Hexapod::MoveMode::HEADLESS);
    //    }

    // GAIT
    if (ps4_data.button_l3) {
      if (ps4_data.button_x) {
        hexapod_->changeGait(Hexapod::Gait::RIPPLE);
      } else if (ps4_data.button_square) {
        hexapod_->changeGait(Hexapod::Gait::TRIPOD);
      } else if (ps4_data.button_circle) {
        // unused atm
      } else if (ps4_data.button_triangle) {
        // unused atm
      }
    }

    float speed_trans_x;
    // Towards 0 is left
    if (ps4_data.l_joystick_x < kJoystickMid - kJoystickDeadzone) {
      speed_trans_x = (float)(kJoystickMid - kJoystickDeadzone - ps4_data.l_joystick_x);
      speed_trans_x /= (float)(255 - kJoystickMid + kJoystickDeadzone);
    }
    // Towards 255 is right
    else if (ps4_data.l_joystick_x > kJoystickMid + kJoystickDeadzone) {
      speed_trans_x = (float)(ps4_data.l_joystick_x - kJoystickMid - kJoystickDeadzone);
      speed_trans_x /= (float)(255 - kJoystickMid + kJoystickDeadzone);
      speed_trans_x = -speed_trans_x;
    } else {
      speed_trans_x = 0.0f;
    }

    float speed_trans_y;
    // Towards 0 is forward
    if (ps4_data.l_joystick_y < kJoystickMid - kJoystickDeadzone) {
      speed_trans_y = (float)(kJoystickMid - kJoystickDeadzone - ps4_data.l_joystick_y);
      speed_trans_y /= (float)(255 - kJoystickMid + kJoystickDeadzone);
    }
    // Towards 255 is backward
    else if (ps4_data.l_joystick_y > kJoystickMid + kJoystickDeadzone) {
      speed_trans_y = (float)(ps4_data.l_joystick_y - kJoystickMid - kJoystickDeadzone);
      speed_trans_y /= (float)(255 - kJoystickMid + kJoystickDeadzone);
      speed_trans_y = -speed_trans_y;
    } else {
      speed_trans_y = 0.0f;
    }

    float turn_speed;
    // Towards 0 is CCW (+turn rate)
    if (ps4_data.r_joystick_x < kJoystickMid - kJoystickDeadzone) {
      turn_speed = (float)(kJoystickMid - kJoystickDeadzone - ps4_data.r_joystick_x);
      turn_speed /= (float)(255 - kJoystickMid + kJoystickDeadzone);
    }
    // Towards 255 is CW (-turn rate)
    else if (ps4_data.r_joystick_x > kJoystickMid + kJoystickDeadzone) {
      turn_speed = (float)(ps4_data.r_joystick_x - kJoystickMid - kJoystickDeadzone);
      turn_speed /= (float)(255 - kJoystickMid + kJoystickDeadzone);
      turn_speed = -turn_speed;
    } else {
      turn_speed = 0.0f;
    }

    float scaled_walk_increment = walk_increment * kMaxTransSpeed;      // can do once at start
    float scaled_turn_increment = walk_turn_increment * kMaxTurnSpeed;  // can do once at start
    // Note: in hexapod, forward is along x axis, left/right along y
    hexapod_->setWalk(Vector3{speed_trans_y * scaled_walk_increment, speed_trans_x * scaled_walk_increment, 0.0f},
                      turn_speed * scaled_turn_increment);

    // Only do these changes if holding L1
    if (ps4_data.button_l1) {
      float roll;
      // Towards 0 is +ve
      if (ps4_data.accel_x < kJoystickMid - kJoystickDeadzone) {
        roll = (float)(kJoystickMid - kJoystickDeadzone - ps4_data.accel_x);
        roll /= (float)(255 - kJoystickMid + kJoystickDeadzone);
      }
      // Towards 255 is -ve
      else if (ps4_data.accel_x > kJoystickMid + kJoystickDeadzone) {
        roll = (float)(ps4_data.accel_x - kJoystickMid - kJoystickDeadzone);
        roll /= (float)(255 - kJoystickMid + kJoystickDeadzone);
        roll = -roll;
      } else {
        roll = 0.0f;
      }

      float pitch;
      // Towards 0 is +ve
      if (ps4_data.accel_y < kJoystickMid - kJoystickDeadzone) {
        pitch = (float)(kJoystickMid - kJoystickDeadzone - ps4_data.accel_y);
        pitch /= (float)(255 - kJoystickMid + kJoystickDeadzone);
      }
      // Towards 255 is -ve
      else if (ps4_data.accel_y > kJoystickMid + kJoystickDeadzone) {
        pitch = (float)(ps4_data.accel_y - kJoystickMid - kJoystickDeadzone);
        pitch /= (float)(255 - kJoystickMid + kJoystickDeadzone);
        pitch = -pitch;
      } else {
        pitch = 0.0f;
      }

      body_change.R_.setRPYExtr(roll * kBodyRotationScale, pitch * kBodyRotationScale, 0.0f);

      if (ps4_data.button_up) {
        body_change.t_(0) += hexapod_->body_translation_increment_;
      } else if (ps4_data.button_down) {
        body_change.t_(0) -= hexapod_->body_translation_increment_;
      }
      if (ps4_data.button_left) {
        body_change.t_(1) += hexapod_->body_translation_increment_;
      } else if (ps4_data.button_right) {
        body_change.t_(1) -= hexapod_->body_translation_increment_;
      }
      if (ps4_data.button_r2) {
        body_change.t_(2) += hexapod_->body_translation_increment_;
      } else if (ps4_data.button_l2) {
        body_change.t_(2) -= hexapod_->body_translation_increment_;
      }

      hexapod_->setBody(body_change);
    }

    if (ps4_data.button_r1) {
      hexapod_->setBody(Transform());
    }
  }

 private:
  Hexapod* hexapod_ = nullptr;
  float walk_increment;
  float walk_turn_increment;
  Vector3 walk_increment_fb;
  Vector3 walk_increment_lr;
  Vector3 manual_fb;
  Vector3 manual_lr;
  Vector3 manual_ud;
  static constexpr uint8_t kJoystickDeadzone = 20;  // either side of midpoint
  static constexpr uint8_t kJoystickMid = 127;
  static constexpr float kMaxTransSpeed = 30.0;  // times walk increment
  static constexpr float kMaxTurnSpeed = 60.0;
  static constexpr float kBodyRotationScale = 0.5;
};
