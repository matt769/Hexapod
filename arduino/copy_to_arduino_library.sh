#!/bin/bash
# Move files into arduino location for convenience
# Expects to be run from its current location

mkdir -p /home/${USER}/Arduino/libraries/Hexapod/
cp ../core/include/* /home/${USER}/Arduino/libraries/Hexapod/
cp ../core/src/hexapod.cpp /home/${USER}/Arduino/libraries/Hexapod/
cp ../core/src/kinematics_support.cpp /home/${USER}/Arduino/libraries/Hexapod/
cp ../core/src/leg.cpp /home/${USER}/Arduino/libraries/Hexapod/
cp ../core/src/transformations.cpp /home/${USER}/Arduino/libraries/Hexapod/
cp ../core/src/receiver.cpp /home/${USER}/Arduino/libraries/Hexapod/
cp ../core/src/joint.cpp /home/${USER}/Arduino/libraries/Hexapod/
cp ../core/src/build_hexapod.cpp /home/${USER}/Arduino/libraries/Hexapod/


cp -r phantomx_ps4_control /home/${USER}/Arduino/
cp -r phantomx_serial_control /home/${USER}/Arduino/