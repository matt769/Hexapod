// NB - the hexapod must start with all joints within the joint limits set here
//  (covers most of the acheivable range, but not the very last few degrees)

// https://github.com/matt769/DynamixelAX12
#include <ax12.h>
// https://github.com/matt769/Hexapod
#include <hexapod.h>
#include <build_hexapod.h>
#include <transformations.h>
#include <receiver.h>

namespace ax12 = dynamixel_ax12;
using namespace hexapod;

uint32_t last_update;
constexpr uint32_t update_period = 50; // ms e.g. 50ms = 20Hz
constexpr uint16_t update_frequency = 1000 / update_period;

constexpr uint8_t kNumLegs = 6;
Hexapod hex = buildPhantomX();


Receiver receiver;
constexpr uint8_t kNumServos = 18;
const float kServoUnitToDeg = 0.293; // TODO Move (some of) these into ax12 library
const float kServoUnitToRad = kServoUnitToDeg * PI/180;
const float kRadToServoUnit = 1.0 / kServoUnitToRad;
const int16_t kServoMiddle = 512;

// for sync write transmission (of position)
constexpr uint8_t tx_buffer_size = kNumServos * (2+1) + 5;
uint8_t sync_write_tx_buffer[tx_buffer_size];

const int16_t offset[kNumServos] = {0, 0, -48, 48, -109, 109, 0, 0, -48, 48, -109, 109, 0, 0, -48, 48, -109, 109};
const int16_t rest[kNumServos] = {kServoMiddle, kServoMiddle, 188, 836, 212, 812, kServoMiddle, kServoMiddle, 188, 836, 212, 812, kServoMiddle, kServoMiddle, 188, 836, 212, 812};
int16_t current_position[kNumServos];
int16_t goal_position[kNumServos] = {kServoMiddle, kServoMiddle, 188, 836, 212, 812, kServoMiddle, kServoMiddle, 188, 836, 212, 812, kServoMiddle, kServoMiddle, 188, 836, 212, 812};

// This contains the servo IDs in the order that they will be extracted from the hexapod
// The hexapod convention is to go leg by leg from lef to right and front to back i.e. front left, front right, next row back left etc..
//  and within each leg moving from the base (where it is attached to the body) out towards the foot i.e. base, hip, knee
//  so overall... front left leg base, hip, knee, front right leg base, hip, knee, 
uint8_t joint_servo_mapping[kNumServos] = {1, 3, 5, 2, 4, 6, 13, 15, 17, 14, 16, 18, 7, 9, 11, 8, 10, 12};

// converts an index into model order array, to an index into the servo ID order arrays e.g. current_position, goal_position
uint8_t modelJointIdxToServoIndex(const uint8_t model_joint_idx) {
  return joint_servo_mapping[model_joint_idx]-1;
}

uint8_t servoIndexToModelJointIdx(const uint8_t servo_idx) {
  return joint_servo_mapping[servo_idx];
}

uint8_t servoIndexToServoId(const uint8_t servo_idx) {
  return servo_idx + 1;
}

uint8_t servoIdToServoIndex(const uint8_t servo_id) {
  return servo_id - 1;
}

uint16_t angleToServoUnits(const float angle) {
  return static_cast<int16_t>(angle * kRadToServoUnit) + kServoMiddle;
}

float servoUnitsToAngle(const int16_t servo_position) {
  return static_cast<float>(servo_position - kServoMiddle) * kServoUnitToRad;
}

void printBuffer(const uint8_t* buffer, uint8_t length) {
  for (int i=0;i<length;i++) {
    Serial.print(buffer[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
}

void printBuffer(const int16_t* buffer, uint8_t length) {
  for (int i=0;i<length;i++) {
    Serial.print(buffer[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
}

int16_t getRegisterWithRetries(const uint8_t id, const uint8_t regstart, const uint8_t data_length, uint8_t retries) {
  for (uint8_t i = 0; i < retries; i++) {
    int16_t val = ax12::getRegister(id, regstart, data_length);
    if (val >= 0) {
      return val;
    }
    delay(20);
  }
  return -1;
}

bool setRegisterWithRetries(const uint8_t id, const uint8_t regstart, const uint16_t data, uint8_t retries) {
  for (uint8_t i = 0; i < retries; i++) {
    ax12::setRegister(id, regstart, data, true);
    
    if (ax12::getLastError() == 0) {
      return true;
    }
    delay(20);
  }
  return false;
}

void moveToPosition(const uint8_t id, const int16_t goal_position, const uint16_t steps) {
  int16_t current_position = getRegisterWithRetries(id, ax12::RegisterPosition::AX_PRESENT_POSITION_L, 2, 5);
  if (current_position < 0 || current_position > 1023) {
//    Serial.println(F("[moveToPosition] Could not read current servo position."));
    return;
  }
  int16_t diff = goal_position - current_position;
  float inc = (float)diff / (float)steps;
  for (uint16_t step = 0; step < steps; step++) {
    float new_position = (float)current_position + (float)step * inc;
    setRegister(id, ax12::RegisterPosition::AX_GOAL_POSITION_L, (uint16_t)new_position, true);
    delay(50);
  }
}

// 1 step per 20ms
void moveAll(const int16_t* goal_position, const uint16_t steps) {
  getCurrentPhysicalPosition();

  float inc[kNumServos];
  for (uint8_t idx = 0; idx < kNumServos; idx++) {
    const int16_t diff = goal_position[idx] - current_position[idx];
    inc[idx] = (float)diff / (float)steps;
  }

  for (uint16_t step = 0; step < steps; step++) {
    
    ax12::setupSyncWrite(kNumServos, ax12::RegisterPosition::AX_GOAL_POSITION_L, 2, sync_write_tx_buffer);
    for (uint8_t idx = 0; idx < kNumServos; idx++) {
      const float new_position = (float)current_position[idx] + (float)step * inc[idx];
      const uint8_t servo_id = idx+1;
      ax12::addToSyncWrite(servo_id, (uint16_t)new_position);
    }
    ax12::executeSyncWrite(); // Ignoring return value for now
    
    delay(50);
  }

  getCurrentPhysicalPosition();
}


// so if joint is already close, it won't take ages
// Super basic, could be much improved, just use for testing
// Speed is servo units per 20ms increment
void moveToPositionFixedSpeed(const uint8_t id, const int16_t goal_position, const uint16_t speed) {
  const int16_t current_position = getRegisterWithRetries(id, ax12::RegisterPosition::AX_PRESENT_POSITION_L, 2, 5);
  if (current_position < 0 || current_position > 1023) {
    return;
  }

  const float diff = (float)(goal_position - current_position);
  float inc = (float)speed;
  if (diff < 0.0) {
    inc = -inc;
  }
  float total_moved = 0.0;
  float new_position = (float)current_position;
  while (total_moved < fabs(diff)) {
    new_position += inc;
    total_moved += fabs(inc);
    // TODO handle overshoot
    setRegister(id, ax12::RegisterPosition::AX_GOAL_POSITION_L, (uint16_t)new_position, true);
    delay(50);
  }
}

void getCurrentPhysicalPosition() {
    for (int servo_id = 1; servo_id <= 18; servo_id++) {
      current_position[servo_id-1] = getRegisterWithRetries(servo_id, ax12::RegisterPosition::AX_PRESENT_POSITION_L, 2, 5);
      if (current_position[servo_id-1] < 0|| current_position[servo_id-1] > 1023) {
        Serial.println(F("Error reading servo position."));
        while(true);
      }
  }
}

// Set the hexapod model joint angles with the current (i.e. measured) servo angles
void setHexapodModelJointsToCurrentServoPositions() {
  for (uint8_t leg_idx = 0; leg_idx < hex.num_legs_; leg_idx++) {
    Leg::JointAngles leg_current_angles;
    const uint8_t joint1_servo_id = joint_servo_mapping[3*leg_idx + 0];
    const uint8_t joint2_servo_id = joint_servo_mapping[3*leg_idx + 1];
    const uint8_t joint3_servo_id = joint_servo_mapping[3*leg_idx + 2];

    leg_current_angles.theta_1 = servoUnitsToAngle(current_position[joint1_servo_id-1]);
    leg_current_angles.theta_2 = servoUnitsToAngle(current_position[joint2_servo_id-1]);
    leg_current_angles.theta_3 = servoUnitsToAngle(current_position[joint3_servo_id-1]);
    
    bool res = hex.setLegJointsPhysical(leg_idx, leg_current_angles);
    Serial.println(res);
  }
}


// Updates goal_position array with the values from the hexapod model
void setServoGoalsToCurrentModelJoints() {
  float joint_array[kNumServos]; // 
  uint8_t ja_idx = 0;
  for (uint8_t leg_idx = 0; leg_idx < hex.num_legs_; leg_idx++) {
    Leg::JointAngles lja = hex.getLeg(leg_idx).getJointAnglesPhysical();
    joint_array[ja_idx++] = lja.theta_1;
    joint_array[ja_idx++] = lja.theta_2;
    joint_array[ja_idx++] = lja.theta_3;
  }
  for (uint8_t hidx = 0; hidx < kNumServos; hidx++) {
    goal_position[(joint_servo_mapping[hidx]-1)] = angleToServoUnits(joint_array[hidx]);
  }
}

// Will command the servos to move directly to whatever the goal_positions are set to.
bool applyServoGoals() {

  ax12::setupSyncWrite(kNumServos, ax12::RegisterPosition::AX_GOAL_POSITION_L, 2, sync_write_tx_buffer);
  for (uint8_t idx = 0; idx < kNumServos; idx++) {
    const uint8_t servo_id = idx+1;
    ax12::addToSyncWrite(servo_id, (uint16_t)goal_position[idx]);
  }
  return ax12::executeSyncWrite();
}

void applyServoGoalsOverTime(const uint16_t num_steps, uint32_t step_period) {
  getCurrentPhysicalPosition();

  float inc[kNumServos];
  for (uint8_t idx = 0; idx < kNumServos; idx++) {
    const int16_t diff = goal_position[idx] - current_position[idx];
    inc[idx] = (float)diff / (float)num_steps;
  }

  const uint8_t data_length = 2;
  const uint8_t buffer_size = kNumServos * (data_length+1) + 5;
  uint8_t txbuffer[buffer_size];
  for (uint16_t step = 0; step < num_steps; step++) {
    
    ax12::setupSyncWrite(kNumServos, ax12::RegisterPosition::AX_GOAL_POSITION_L, 2, txbuffer);
    for (uint8_t idx = 0; idx < kNumServos; idx++) {
      const float new_position = (float)current_position[idx] + (float)step * inc[idx];
      const uint8_t servo_id = idx+1;
      ax12::addToSyncWrite(servo_id, (uint16_t)new_position);
    }
    ax12::executeSyncWrite(); // Ignore return value for now
    
    delay(step_period);
  }
}


void setup() {
  bool res = false;

  hex.setUpdateFrequency(update_frequency);
  
  dynamixel_ax12::init(1000000);

  Serial.begin(115200);
  Serial.println(F("Initialising..."));

  
  ax12::enableTorque();
  Serial.print(F("Starting joint positions from servos"));
  getCurrentPhysicalPosition();
  printBuffer(current_position, kNumServos);
  
  Serial.print(F("Joint goals from hexapod model after setting joints to current servo positions"));
  // NOTE!! If the hexapod has it legs outside the model's allowed ranges, this will not work properly
  // TODO use a manual movement to set the legs to something we know is allowed
  //  and only then initialise the model angles
  setHexapodModelJointsToCurrentServoPositions();
  // the model should now be in sync with the physical robot
  
  Serial.print(F("Moving to preset starting position..."));
  const Leg::JointAngles start = {0.0f, 1.41f, 1.50f};
  const uint16_t move_duration = 50;
  bool res1 = true;
  for (uint8_t leg_idx = 0; leg_idx < hex.num_legs_; leg_idx++) {
    res1 &= hex.setLegTarget(leg_idx, start, move_duration);
  }
  if (!res1) while(1);
  Serial.println(res);
  delay(1000);
  
  for(uint16_t cnt = 0; cnt < move_duration; ++cnt) {
    hex.update();
    setServoGoalsToCurrentModelJoints();
    applyServoGoals();
    delay(update_period); // inaccurate timing used like this but fine for basic setup
  }
  
  receiver.setRobot(&hex);
  Serial.println(F("Listening for instructions..."));
  
  last_update = millis();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    Serial.println(cmd);
    receiver.processCommand(cmd);
  }

  if (millis() - last_update > update_period) {
    hex.update();
    setServoGoalsToCurrentModelJoints();
    applyServoGoals();
    last_update += update_period;
  }
}
