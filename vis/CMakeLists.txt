cmake_minimum_required(VERSION 3.22.1)
project(hexapod_vis)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(ament_cmake REQUIRED)

message(WARNING ${hexapod_core_FOUND})

find_package(hexapod_core REQUIRED)

message(WARNING ${hexapod_core_FOUND})

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
#     ${hexapod_core_INCLUDE_DIRS}
    # How can I make it find this properly
    # GET IT TO COMPILE, THEN FIX BUILD
    "/home/matt/ws/hexapod_ws/install/hexapod_core/include"
    )

message(WARNING ${CMAKE_INSTALL_INCLUDEDIR})

add_executable(demo_node src/demo_node.cpp
        src/visualisation.cpp
        )

# target_link_libraries(demo_node
#         ${hexapod_core_LIBRARIES}
#         )
ament_target_dependencies(demo_node hexapod_core rclcpp sensor_msgs tf2_geometry_msgs tf2 tf2_ros urdf)


# add_executable(test_tf
#         test/test_tf.cpp
#         )

# target_link_libraries(test_tf
#         ${hexapod_core_LIBRARIES}
#         )

# add_executable(controller_node
#         src/controller_node.cpp
#         )

# target_link_libraries(controller_node
#         ${hexapod_core_LIBRARIES}
#         ${catkin_LIBRARIES}
#         )

# add_executable(receiver_node
#         src/receiver_node.cpp
#         src/ros_receiver.cpp
#         src/visualisation.cpp
#         src/build_from_urdf.cpp
#         )

# target_link_libraries(receiver_node
#         ${hexapod_core_LIBRARIES}
#         ${catkin_LIBRARIES}
#         )





# install(
#         DIRECTORY launch
#         DESTINATION share/${PROJECT_NAME}
#         )

# ament_package()