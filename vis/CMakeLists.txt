cmake_minimum_required(VERSION 2.8.3)
project(hexapod_vis)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Wall -Wextra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs tf2 tf2_ros)
find_package(hexapod_core REQUIRED)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${hexapod_core_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

## Declare a C++ executable
# add_executable(test_ik
#     test/test_ik.cpp
#     )

add_executable(demo_node src/demo_node.cpp
    src/visualisation.cpp
    )

add_executable(test_tf
    test/test_tf.cpp
    )

add_executable(controller_node
    src/controller_node.cpp
    )

add_executable(receiver_node
    src/receiver_node.cpp
    src/receiver.cpp
    src/visualisation.cpp    
    )  

## Specify libraries to link a library or executable target against
# target_link_libraries(test_ik
#     ${hexapod_core_LIBRARIES}
#     ${catkin_LIBRARIES}
#     )

target_link_libraries(demo_node
    ${hexapod_core_LIBRARIES}
    ${catkin_LIBRARIES}
    )

target_link_libraries(test_tf
    ${hexapod_core_LIBRARIES}
    )

target_link_libraries(controller_node
    ${hexapod_core_LIBRARIES}
    ${catkin_LIBRARIES}
    )

target_link_libraries(receiver_node
    ${hexapod_core_LIBRARIES}
    ${catkin_LIBRARIES}
    )

