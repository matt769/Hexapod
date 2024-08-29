#ifndef BUILD_FROM_URDF_H
#define BUILD_FROM_URDF_H

#include <hexapod_core/hexapod.h>

#include <string>

namespace hexapod_vis {

hexapod::Hexapod buildFromURDF(const std::string& robot_description_string);
}

#endif