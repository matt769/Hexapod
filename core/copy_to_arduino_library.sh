#!/bin/bash
# Move files into arduino library location for convenience
# Expects to be run from its current location
mkdir -p /home/$USER/Arduino/libraries/Hexapod/
cp include/* /home/$USER/Arduino/libraries/Hexapod/
cp src/hexapod.cpp /home/$USER/Arduino/libraries/Hexapod/
cp src/kinematics_support.cpp /home/$USER/Arduino/libraries/Hexapod/
cp src/leg.cpp /home/$USER/Arduino/libraries/Hexapod/
cp src/transformations.cpp /home/$USER/Arduino/libraries/Hexapod/
cp src/receiver.cpp /home/$USER/Arduino/libraries/Hexapod/
