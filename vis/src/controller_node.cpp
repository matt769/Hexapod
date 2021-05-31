#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <signal.h>
#include <stdio.h>
#include <termios.h>

class Controller {
 public:
  Controller();
  ~Controller();
  void getInput();

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  int kfd = 0;
  struct termios cooked, raw;
};

void quit(int /* sig */) {
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller");
  Controller hex_controller;

  signal(SIGINT, quit);

  hex_controller.getInput();

  return (0);
}

Controller::Controller() { pub_ = nh_.advertise<std_msgs::Int32>("hexapod/command_key", 5); }

Controller::~Controller() { tcsetattr(kfd, TCSANOW, &cooked); }

void Controller::getInput() {
  char c;
  bool data_received = false;
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("-------- MOVING ------------------------------------- ADJUST BODY -------------");
  puts("W: Increase forward speed            T: Increase body forward/backward position");
  puts("S: Decrease forward speed            G: Decrease body forward position");
  puts("A: Increase lateral (to left) speed  F: Increase body left/right position");
  puts("D: Decrease lateral speed            H: Decrease body left/right position");
  puts("Q: Increase turn speed               R: Increase body up/down position");
  puts("E: Decrease turn speed               Y: Decrease body up/down position");
  puts("X: Stop walking                      B: Reset body translation");
  puts("");
  puts("----------- OTHER -----------------");
  puts("1, 2, 3, 4: Change gait           |  I: Increase body pitch");
  puts("]: Increase stance width          |  K: Decrease body pitch");
  puts("[: Decrease stance width          |  J: Increase body roll");
  puts(";: Reset stance width             |  L: Decrease body roll");
  puts("                                  |  U: Increase body yaw");
  puts("#: Increase stride length         |  O: Decrease body yaw");
  puts("': Decrease stride length         |  ,: Reset body rotation");
  puts("/: Reset stride length            |--------------------------------------------");
  puts("}: Increase step time");
  puts("{: Decrease step time");
  puts(":: Reset step time");
  puts("=: Increase leg height        Enter: Headless mode / reset headless mode");
  puts("-: Decrease leg height        Backspace: Standard mode");
  puts("0: Reset leg height           `: (Press) Set legs to ground / (Hold) Raise body");
  puts("-------------------------------------------------------------------------------");
  puts("Space bar: toggle manual mode. While in manual mode...");
  puts("L: Leg mode / Cycle through legs      J: Joint mode / Cycle through joints");
  puts("While in Leg mode");
  puts("W: Increase foot in X    S: Decrease foot in X");
  puts("A: Increase foot in Y    D: Decrease foot in Y");
  puts("Q: Increase foot in Z    E: Decrease foot in Z");
  puts("While in Joint mode");
  puts("W: Increase joint    S: Decrease joint");
  puts("-------------------------------------------------------------------------------");
  puts("                        Now reading from keyboard...");


  for (;;) {
    // get the next event from the keyboard
    int read_result = read(kfd, &c, 1);
    if (read_result < 0) {
      perror("read():");
      exit(-1);
    } else if (read_result > 0) {
      data_received = true;
    }

    std_msgs::Int32 key;
    key.data = c;
    if (data_received) {
      pub_.publish(key);
      data_received = false;
    }
  }
  return;
}
