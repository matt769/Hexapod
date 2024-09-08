#!/bin/bash
# Move files into arduino location for convenience
# Expects to be run from its current location

ARDUINO_HOME=/home/${USER}/Arduino
LIB_PATH=${ARDUINO_HOME}/libraries/Hexapod
mkdir -p ${LIB_PATH}

# Arduino won't look in subdirectories for headers so if we want them to be available for use in sketches then we have to put in main code folder
cp ../core/include/hexapod_core/* ${LIB_PATH}/
cp ../core/src/hexapod.cpp ${LIB_PATH}
cp ../core/src/kinematics_support.cpp /${LIB_PATH}
cp ../core/src/leg.cpp ${LIB_PATH}
cp ../core/src/transformations.cpp ${LIB_PATH}
cp ../core/src/receiver.cpp ${LIB_PATH}
cp ../core/src/joint.cpp ${LIB_PATH}
cp ../core/src/build_hexapod.cpp ${LIB_PATH}
# And because we've changed the relative location of the headers, we need to modify the #include directives
find ${LIB_PATH} -type f -name *.h -o -name *.cpp | xargs sed -i 's#hexapod_core/##g'

touch ${LIB_PATH}/.development


cp -r phantomx_ps4_control /home/${USER}/Arduino/
cp -r phantomx_serial_control /home/${USER}/Arduino/