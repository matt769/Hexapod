cmake_minimum_required(VERSION 2.8.3)
project(hexapod_core)

add_compile_options(-std=c++11 -Wall -Wextra -Wpedantic -Wshadow -Wunused
        -Wduplicated-cond -Wduplicated-branches
        -Wlogical-op -Wnull-dereference -Wuseless-cast
        -Wsign-conversion
        )

find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES hexapod_core
)

include_directories(include)

add_library(${PROJECT_NAME} STATIC
  src/hexapod.cpp
  src/leg.cpp
  src/kinematics_support.cpp
  src/transformations.cpp
)
