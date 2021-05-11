#include "ros_receiver.h"

#include "hexapod.h"
#include "transformations.h"

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <std_msgs/Int32.h>

namespace hexapod_vis {
using namespace hexapod;

Receiver::Receiver(const ros::NodeHandle& nh, Hexapod *hexapod) : nh_(nh), hexapod_(hexapod) {
  input_sub_ = nh_.subscribe("hexapod/command_key", 10, &Receiver::callbackProcessKeyPress, this);
}

void Receiver::update() {
  if (hexapod_->getState() != Hexapod::State::FULL_MANUAL) {
    hexapod_->setWalk(current_walk, current_turn);
  }
  // everything else is set within the callback itself
}

void Receiver::callbackProcessKeyPress(const std_msgs::Int32::ConstPtr& msg) {
  int32_t keyCode = msg->data;
  std::cout << "Key code pressed: " << msg->data << '\n';

  // If we're in manual mode, alternative commands
  if (hexapod_->getState() == Hexapod::State::FULL_MANUAL) {
    Hexapod::ManualControlType mct = hexapod_->getManualControlType();
    switch (keyCode) {
      case 108: // L
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->setManualLegControl(hexapod_->getManualControlLegIdx() + 1); // will be wrapped if too high
        } else {
          hexapod_->setManualLegControl(0);
        }
        break;
      case 106: // J
        if (mct == Hexapod::ManualControlType::SINGLE_JOINT) {
          hexapod_->setManualJointControl(hexapod_->getManualControlJointIdx() + 1);
        } else {
          hexapod_->setManualJointControl(0);
        }
        break;
      case 119: // W
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(manual_fb);
        } else if (mct == Hexapod::ManualControlType::SINGLE_JOINT) {
          hexapod_->manualChangeJoint(manual_joint);
        }
        break;
      case 97: // A
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(manual_lr);
        }
        break;
      case 115: // S
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(-manual_fb);
        } else if (mct == Hexapod::ManualControlType::SINGLE_JOINT) {
          hexapod_->manualChangeJoint(-manual_joint);
        }
        break;
      case 100: // D
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(-manual_lr);
        }
        break;
      case 113:
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(manual_ud);
        }
        break;
      case 101:
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(-manual_ud);
        }
        break;
    }
    return;
  }


  // 'Standard' control

  Transform body_change;

  switch (keyCode) {
    case 119:current_walk = current_walk + walk_increment_fb;
      break;
    case 115:current_walk = current_walk - walk_increment_fb;
      break;
    case 97:current_walk = current_walk + walk_increment_lr;
      break;
    case 100:current_walk = current_walk - walk_increment_lr;
      break;
    case 113:current_turn += turn_increment;
      break;
    case 101:current_turn -= turn_increment;
      break;
    case 120:current_walk = Vector3(0.0f, 0.0f, 0.0f);
      current_turn = 0.0f;
      break;

    case 116:body_change.t_(0) += body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 103:body_change.t_(0) -= body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 102:body_change.t_(1) += body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 104:body_change.t_(1) -= body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 114:body_change.t_(2) += body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 121:body_change.t_(2) -= body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 98:hexapod_->setBody(body_change);  // TODO implement resetBody()
      break;

    case 105:body_change.R_.setRPYExtr(0.0f, body_rotation_increment, 0.0f);
      hexapod_->changeBody(body_change);
      break;
    case 107:body_change.R_.setRPYExtr(0.0f, -body_rotation_increment, 0.0f);
      hexapod_->changeBody(body_change);
      break;
    case 106:body_change.R_.setRPYExtr(body_rotation_increment, 0.0f, 0.0f);
      hexapod_->changeBody(body_change);
      break;
    case 108:body_change.R_.setRPYExtr(-body_rotation_increment, 0.0f, 0.0f);
      hexapod_->changeBody(body_change);
      break;
    case 117:body_change.R_.setRPYExtr(0.0f, 0.0f, body_rotation_increment);
      hexapod_->changeBody(body_change);
      break;
    case 111:body_change.R_.setRPYExtr(0.0f, 0.0f, -body_rotation_increment);
      hexapod_->changeBody(body_change);
      break;
    case 44:hexapod_->setBody(body_change);  // TODO implement resetBody()
      break;

    case 49:hexapod_->changeGait(Hexapod::Gait::RIPPLE);
      break;
    case 50:hexapod_->changeGait(Hexapod::Gait::LEFT_RIGHT_LEFT_RIGHT);
      break;
    case 51:hexapod_->changeGait(Hexapod::Gait::LHS_THEN_RHS);
      break;
    case 52:hexapod_->changeGait(Hexapod::Gait::AROUND_THE_CLOCK);
      break;
    case 53:
      if (hexapod_->num_legs_ == 6) {
        hexapod_->changeGait(Hexapod::Gait::TRIPOD);
      }
      break;

    case 93:hexapod_->changeStanceWidth(stance_width_increment);
      break;
    case 91:hexapod_->changeStanceWidth(-stance_width_increment);
      break;
    case 59:hexapod_->resetStanceWidth();
      break;

      // Change foot ground travel ratio
    case 35:hexapod_->changeFootGroundTravelRatio(ftgr_increment);
      break;
    case 39:hexapod_->changeFootGroundTravelRatio(-ftgr_increment);
      break;
    case 47:hexapod_->resetFootGroundTravelRatio();
      break;

      // Change leg raise height
    case 61:hexapod_->changeLegRaiseHeight(leg_raise_increment);
      break;
    case 45:hexapod_->changeLegRaiseHeight(-leg_raise_increment);
      break;
    case 48:hexapod_->resetLegRaiseHeight();
      break;

    case 127:  // backspace
      hexapod_->setMoveMode(Hexapod::MoveMode::STANDARD);
      break;
    case 10:  // enter
      hexapod_->setMoveMode(Hexapod::MoveMode::HEADLESS);
      break;

    case 96:  // backtick
      if (hexapod_->getState() == Hexapod::State::UNSUPPORTED) {
        hexapod_->setAllLegTargetsToGround(50);
      }
      if (hexapod_->getState() == Hexapod::State::STANDING) {
        hexapod_->riseToWalk();
      }
      break;
      // Manual control
    case 32:  // Space bar
      if (hexapod_->getState() != Hexapod::State::FULL_MANUAL) {
        hexapod_->setFullManualControl(true); // TODO allow turning off in future
      }
      break;

    default:std::cout << "Unrecognised key\n";
  }
}

} // namespace hexapod

