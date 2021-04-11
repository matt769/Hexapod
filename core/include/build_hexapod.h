//
// Created by matt on 12/04/2021.
//

#ifndef HEXAPOD_BUILD_HEXAPOD_H_
#define HEXAPOD_BUILD_HEXAPOD_H_

#include "hexapod.h"

namespace hexapod {

/** @brief Returns a Hexapod object consistent with example file hexapod.urdf.xacro */
hexapod::Hexapod buildDefaultHexapod();
/** @brief Returns a Hexapod object consistent with example file hexapod2.urdf.xacro */
hexapod::Hexapod buildDefaultHexapod2();
hexapod::Hexapod buildPhantomX();
hexapod::Hexapod buildPhantomXForVis();
/** @brief Returns a Hexapod object consistent with example file octapod.urdf.xacro */
hexapod::Hexapod buildDefaultOctapod();

} // namespace hexapod

#endif // HEXAPOD_BUILD_HEXAPOD_H_
