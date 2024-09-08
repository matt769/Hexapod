cmake_minimum_required(VERSION 3.22.1)
project(hexapod_vis)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(ament_cmake REQUIRED)

find_package(hexapod_core REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

include_directories(
    include
)


add_executable(demo_node src/demo_node.cpp
        src/visualisation.cpp
        )
ament_target_dependencies(demo_node hexapod_core rclcpp sensor_msgs geometry_msgs visualization_msgs tf2_geometry_msgs tf2 tf2_ros urdf)


add_executable(test_tf
     test/test_tf.cpp
     )
ament_target_dependencies(test_tf hexapod_core tf2)



add_executable(controller_node
     src/controller_node.cpp
     )
ament_target_dependencies(controller_node hexapod_core rclcpp std_msgs)


add_executable(receiver_node
     src/receiver_node.cpp
     src/ros_receiver.cpp
     src/visualisation.cpp
     src/build_from_urdf.cpp
     )
ament_target_dependencies(receiver_node hexapod_core rclcpp sensor_msgs geometry_msgs visualization_msgs tf2_geometry_msgs tf2 tf2_ros urdf)


install(
        TARGETS demo_node controller_node receiver_node
        DESTINATION lib/${PROJECT_NAME}
)

install(
     DIRECTORY launch
     DESTINATION share/${PROJECT_NAME}
 )

install(
        DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME}
)

 ament_package()
