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

add_library(${PROJECT_NAME} STATIC
  src/hexapod.cpp
  src/leg.cpp
  src/kinematics_support.cpp
  src/transformations.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)

add_executable(${PROJECT_NAME}_example
        src/example.cpp)
target_include_directories(${PROJECT_NAME}_example PRIVATE include)
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME})
