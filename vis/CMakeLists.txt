cmake_minimum_required(VERSION 2.8.3)
project(hexapod_vis)

add_compile_options(-std=c++14 -Wall -Wextra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs urdf)
find_package(hexapod_core REQUIRED)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${hexapod_core_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

add_executable(demo_node src/demo_node.cpp
        src/visualisation.cpp
        )

target_link_libraries(demo_node
        ${hexapod_core_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_executable(test_tf
        test/test_tf.cpp
        )

target_link_libraries(test_tf
        ${hexapod_core_LIBRARIES}
        )

add_executable(controller_node
        src/controller_node.cpp
        )

target_link_libraries(controller_node
        ${hexapod_core_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_executable(receiver_node
        src/receiver_node.cpp
        src/ros_receiver.cpp
        src/visualisation.cpp
        src/build_from_urdf.cpp
        )

target_link_libraries(receiver_node
        ${hexapod_core_LIBRARIES}
        ${catkin_LIBRARIES}
        )
