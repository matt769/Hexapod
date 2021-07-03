#include <transformations.h>
#include <hexapod.h>
#include <ps4.h>

using namespace hexapod; // Wouldn't normally do this in header, but this is only a header because using arduino

class PS4Receiver {
 public:
  explicit PS4Receiver(Hexapod* hexapod)
  {
    setRobot(hexapod);
  }

  PS4Receiver() : PS4Receiver(nullptr) {}

  void setRobot(Hexapod *hexapod) {
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

    if (!first && data.l_joystick_y < kJoystickMid + kJoystickDeadzone && data.l_joystick_y > kJoystickMid - kJoystickDeadzone) {
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
    if (third && !forth && data.l_joystick_y < kJoystickMid + kJoystickDeadzone && data.l_joystick_y > kJoystickMid - kJoystickDeadzone) {
      forth = true;
      return true;
    }
    return first && second && third && forth;    
  }

  void processCommand(const ps4::PS4_data& ps4_data) {
    if (!hexapod_) return;

    if(hexapod_->getState() == Hexapod::State::FULL_MANUAL) {
      // TODO
      // Alternate controls
      // Manual mode entry and exit

      return;
    }


    // Rotational part will be set. Translational part will be incremented.
    Transform body_change = hexapod_->getBaseToBody();
//    ps4::printData(&Serial, ps4_data);

    // START UP / SHUTDOWN
    if(ps4_data.button_ps) {
      if (hexapod_->getState() == Hexapod::State::UNSUPPORTED) {
        hexapod_->setAllLegTargetsToGround(50);
        Serial.println(F("setAllLegTargetsToGround"));
      }
      else if (hexapod_->getState() == Hexapod::State::STANDING) {
        hexapod_->riseToWalk();
        Serial.println(F("riseToWalk"));
      }
      else if (hexapod_->getState() == Hexapod::State::WALKING) {
        hexapod_->lowerToGround();
        Serial.println(F("lowerToGround"));
      }
      return;
    }

    // MOVEMENT PARAMETERS
    if(ps4_data.button_x) {
      hexapod_->changeLegRaiseTime(-hexapod_->leg_raise_time_increment_); // faster
    }
    if(ps4_data.button_square) {
      hexapod_->changeLegRaiseTime(hexapod_->leg_raise_time_increment_);
    }
    if(ps4_data.button_circle) {
      hexapod_->changeLegRaiseHeight(hexapod_->leg_raise_increment_);
    }
    if(ps4_data.button_triangle) {
      hexapod_->changeLegRaiseHeight(-hexapod_->leg_raise_increment_);
    }

    if (!ps4_data.button_l1) {
      if (ps4_data.button_up) {
        hexapod_->changeFootGroundTravelRatio(hexapod_->ftgr_increment_);
      }
      else if (ps4_data.button_down) {
        hexapod_->changeFootGroundTravelRatio(-hexapod_->ftgr_increment_);
      }
      if (ps4_data.button_left) {
        hexapod_->changeStanceWidth(-hexapod_->stance_width_increment_);
      }
      else if (ps4_data.button_right) {
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
      }
      else if (ps4_data.button_square) {
        hexapod_->changeGait(Hexapod::Gait::TRIPOD);
      }
      else if (ps4_data.button_circle) {
        // unused atm
      }
      else if (ps4_data.button_triangle) {
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
    }
    else {
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
    }
    else {
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
    }
    else {
      turn_speed = 0.0f;
    }
  
    float scaled_walk_increment = walk_increment * kMaxTransSpeed; // can do once at start
    float scaled_turn_increment = walk_turn_increment * kMaxTurnSpeed; // can do once at start
    // Note: in hexapod, forward is along x axis, left/right along y
    hexapod_->setWalk(Vector3{speed_trans_y * scaled_walk_increment, speed_trans_x * scaled_walk_increment, 0.0f}, turn_speed * scaled_turn_increment);

    
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
      }
      else {
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
      }
      else {
        pitch = 0.0f;
      }

      body_change.R_.setRPYExtr(roll * kBodyRotationScale, pitch * kBodyRotationScale, 0.0f);

      if (ps4_data.button_up) {
        body_change.t_(0) += hexapod_->body_translation_increment_;
      }
      else if (ps4_data.button_down) {
        body_change.t_(0) -= hexapod_->body_translation_increment_;
      }
      if (ps4_data.button_left) {
        body_change.t_(1) += hexapod_->body_translation_increment_;
      }
      else if (ps4_data.button_right) {
        body_change.t_(1) -= hexapod_->body_translation_increment_;
      }
      if (ps4_data.button_r2) {
        body_change.t_(2) += hexapod_->body_translation_increment_;
      }
      else if (ps4_data.button_l2) {
        body_change.t_(2) -= hexapod_->body_translation_increment_;
      }

      hexapod_->setBody(body_change);
    }

    if (ps4_data.button_r1) {
      hexapod_->setBody(Transform());
    }
   
  }


 private:
  Hexapod *hexapod_ = nullptr;
  float walk_increment;
  float walk_turn_increment;
  Vector3 walk_increment_fb;
  Vector3 walk_increment_lr;
  Vector3 manual_fb;
  Vector3 manual_lr;
  Vector3 manual_ud;
  static constexpr uint8_t kJoystickDeadzone = 20; // either side of midpoint
  static constexpr uint8_t kJoystickMid = 127;
  static constexpr float kMaxTransSpeed = 30.0; // times walk increment
  static constexpr float kMaxTurnSpeed = 60.0;
  static constexpr float kBodyRotationScale = 0.5;
};
