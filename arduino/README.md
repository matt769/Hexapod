
# Build
The physical robot I have been running this on is this [kit](https://www.trossenrobotics.com/phantomx-ax-hexapod.aspx) plus a [USB host controller](https://www.hobbytronics.co.uk/ps3-ps4-controller-bluetooth) and bluetooth dongle for connecting to the PS4 controller.
Also FTDI breakout board for connecting the computer to the board.

TODO ADD PICTURE


# Setup files

## Install Arduino IDE
https://docs.arduino.cc/software/ide-v2/tutorials/getting-started/ide-v2-downloading-and-installing/


## Add arbotix board definitions
I'm not sure if/where they are available online anymore. Can use the copy in [board_definition/](board_definition/).

```sh
mkdir -p /home/${USER}/Arduino/hardware/
cp board_definition/arbotix /home/${USER}/Arduino/hardware/
```

## Download the dependent libraries
```sh
mkdir -p /home/${USER}/Arduino/libraries/
git clone https://github.com/matt769/DynamixelAX12.git /home/${USER}/Arduino/libraries/
git clone https://github.com/matt769/PS4.git /home/${USER}/Arduino/libraries/
```

## Copy the files from the core package and the arduino programs
This step can be run with the provided `copy_to_arduino_library.sh` script (useful during development of the internals).


# Compile
Open one of the provided arduino programs in the Arduino IDE, select board = `Arbotix Std`. Compile.


# Upload
Connect computer to Arbotix board via USB->FTDI board. Upload using Arduino IDE.



# Run

## Using PS4 controller
See sketch [phantomx_ps4_control/phantomx_ps4_control.ino](phantomx_ps4_control/phantomx_ps4_control.ino)  
Follow instructions/links in the `https://github.com/matt769/PS4` repo to set up the PS4 controller + USB host board + bluetooth dongle.
Use the provided 


## Using keybord from connected computer
See sketch [phantomx_serial_control/phantomx_serial_control.ino](phantomx_serial_control/phantomx_serial_control.ino)  
Here the robot is connected by computer->USB->FTDI board->robot. We can provide input through the Arduino Serial Monitor using the keyboard.



# TODO
 - switch to using MightCore for AtMega644 if possible
 - reorganise core so can simply copy all files to arduino
 - check compiler warnings