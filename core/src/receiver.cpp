#include "receiver.h"

#ifdef __AVR__
#include <Arduino.h>
#else
#include <iostream>
#endif

#include "hexapod.h"
#include "transformations.h"

using namespace Transformations;

Receiver::Receiver() : Receiver(nullptr) {}
Receiver::Receiver(Hexapod* hexapod) : hexapod_(hexapod) {}

void Receiver::setRobot(Hexapod* hexapod) {
  hexapod_ = hexapod;
}

void Receiver::update() {
  if (!hexapod_) return;
  if (hexapod_->getState() != Hexapod::State::FULL_MANUAL) {
    hexapod_->setWalk(current_walk, current_turn);
  }
  // everything else is set within processCommand
}


void Receiver::processCommand(const uint8_t cmd) {
  if (!hexapod_) return;

  // If we're in manual mode, alternative commands
  if (hexapod_->getState() == Hexapod::State::FULL_MANUAL) {
    Hexapod::ManualControlType mct = hexapod_->getManualControlType();
    switch (cmd) {
      case 108: // l
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->setManualLegControl(hexapod_->getManualControlLegIdx() + 1); // will be wrapped if too high
        }
        else {
          hexapod_->setManualLegControl(0);
        }
        break;
      case 106: // j
        if (mct == Hexapod::ManualControlType::SINGLE_JOINT) {
          hexapod_->setManualJointControl(hexapod_->getManualControlJointIdx() + 1);
        }
        else {
          hexapod_->setManualJointControl(0);
        }
        break;
      case 119: // w
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(manual_fb);
        }
        else if (mct == Hexapod::ManualControlType::SINGLE_JOINT) {
          hexapod_->manualChangeJoint(manual_joint);
        }
        break;
      case 97: // a
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(manual_lr);
        }
        break;
      case 115: // s
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(-manual_fb);
        }
        else if (mct == Hexapod::ManualControlType::SINGLE_JOINT) {
          hexapod_->manualChangeJoint(-manual_joint);
        }
        break;
      case 100: // d
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(-manual_lr);
        }
        break;
      case 113: // q
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(manual_ud);
        }
        break;
      case 101: // e
        if (mct == Hexapod::ManualControlType::SINGLE_LEG) {
          hexapod_->manualMoveFoot(-manual_ud);
        }
        break;
    }
    return;
  }


  // 'Standard' control
  Transform body_change;

  switch (cmd) {
    case 119: // w  Increase forward velocity
      current_walk = current_walk + walk_increment_fb;
      break;
    case 115: // s Decrease forward velocity
      current_walk = current_walk - walk_increment_fb;
      break;
    case 97: // a Increase leftward velocity
      current_walk = current_walk + walk_increment_lr;
      break;
    case 100: // d Decrease leftward velocity
      current_walk = current_walk - walk_increment_lr;
      break;
    case 113: // q Increase CCW angular velocity
      current_turn += turn_increment;
      break;
    case 101: // e Decrease CCW angular velocity
      current_turn -= turn_increment;
      break;
    case 120: // x  Stop
      current_walk = Vector3(0.0f, 0.0f, 0.0f);
      current_turn = 0.0f;
      break;

    case 116: // t  Move body forward
      body_change.t_(0) += body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 103: // g  Move body backward
      body_change.t_(0) -= body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 102: // f  Move body left
      body_change.t_(1) += body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 104: // h  Move body right
      body_change.t_(1) -= body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 114: // r  Move body up
      body_change.t_(2) += body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 121: // y  Move body down
      body_change.t_(2) -= body_translation_increment;
      hexapod_->changeBody(body_change);
      break;
    case 98: // b Reset body translation
      hexapod_->setBody(body_change);  // TODO implement resetBody()
      break;

    case 105:
      // TODO precalculate these rotations
      body_change.R_.setRPYExtr(0.0f, body_rotation_increment, 0.0f);
      hexapod_->changeBody(body_change);
      break;
    case 107:
      body_change.R_.setRPYExtr(0.0f, -body_rotation_increment, 0.0f);
      hexapod_->changeBody(body_change);
      break;
    case 106:
      body_change.R_.setRPYExtr(body_rotation_increment, 0.0f, 0.0f);
      hexapod_->changeBody(body_change);
      break;
    case 108:
      body_change.R_.setRPYExtr(-body_rotation_increment, 0.0f, 0.0f);
      hexapod_->changeBody(body_change);
      break;
    case 117:
      body_change.R_.setRPYExtr(0.0f, 0.0f, body_rotation_increment);
      hexapod_->changeBody(body_change);
      break;
    case 111:
      body_change.R_.setRPYExtr(0.0f, 0.0f, -body_rotation_increment);
      hexapod_->changeBody(body_change);
      break;
    case 44:
      hexapod_->setBody(body_change);  // TODO implement resetBody()
      break;

    case 49:
      hexapod_->changeGait(Hexapod::Gait::RIPPLE);
      break;
    case 50:
      hexapod_->changeGait(Hexapod::Gait::LEFT_RIGHT_LEFT_RIGHT);
      break;
    case 51:
      hexapod_->changeGait(Hexapod::Gait::LHS_THEN_RHS);
      break;
    case 52:
      hexapod_->changeGait(Hexapod::Gait::AROUND_THE_CLOCK);
      break;

    case 93:
      hexapod_->changeStanceWidth(stance_width_increment);
      break;
    case 91:
      hexapod_->changeStanceWidth(-stance_width_increment);
      break;
    case 59:
      hexapod_->resetStanceWidth();
      break;

    // Change foot ground travel ratio
    case 35:
      hexapod_->changeFootGroundTravelRatio(ftgr_increment);
      break;
    case 39:
      hexapod_->changeFootGroundTravelRatio(-ftgr_increment);
      break;
    case 47:
      hexapod_->resetFootGroundTravelRatio();
      break;

    // Change leg raise height
    case 61:
      hexapod_->changeLegRaiseHeight(leg_raise_increment);
      break;
    case 45:
      hexapod_->changeLegRaiseHeight(-leg_raise_increment);
      break;
    case 48:
      hexapod_->resetLegRaiseHeight();
      break;

    // TODO change to toggle
    case 127:  // backspace
      hexapod_->setMoveMode(Hexapod::MoveMode::STANDARD);
      break;
    case 10:  // enter
      hexapod_->setMoveMode(Hexapod::MoveMode::HEADLESS);
      break;

    case 96:  // backtick
      if (hexapod_->getState() == Hexapod::State::UNSUPPORTED) {
        hexapod_->setLegsToGround();
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

    default:
#ifdef __AVR__
  Serial.print("Unrecognised key: ");
  Serial.println(cmd);
#else
      std::cout << "Unrecognised key: " << cmd << '\n';
#endif
  }
}