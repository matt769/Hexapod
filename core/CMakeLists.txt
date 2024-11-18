cmake_minimum_required(VERSION 3.22.1)
project(hexapod_core)

add_compile_options(-std=c++11 -Wall -Wextra -Wpedantic -Wshadow -Wunused
        -Wduplicated-cond -Wduplicated-branches
        -Wlogical-op -Wnull-dereference -Wuseless-cast
        -Wsign-conversion
        )

find_package(ament_cmake REQUIRED)
find_package(Catch2 REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/hexapod.cpp
  src/leg.cpp
  src/joint.cpp
  src/kinematics_support.cpp
  src/transformations.cpp
  src/receiver.cpp
  src/build_hexapod.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)



add_executable(${PROJECT_NAME}_example
        src/example.cpp)
target_include_directories(${PROJECT_NAME}_example PRIVATE include)
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME})


ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
#ament_export_dependencies(some_dependency)

install(
     DIRECTORY include/
     DESTINATION include
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        TARGETS ${PROJECT_NAME}_example
        RUNTIME DESTINATION bin
)

# INSTALL (
#     DIRECTORY ${CMAKE_SOURCE_DIR}/include/
#     DESTINATION include
#     FILES_MATCHING PATTERN "*.h*")

#install(EXPORT
#        "${PROJECT_NAME}Config"
#        DESTINATION "share/${PROJECT_NAME}/cmake"
#        # NAMESPACE ${PROJECT_NAME}::
#)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


add_executable(tests tests/test_main.cpp tests/test_joint.cpp)
target_link_libraries(tests PRIVATE ${PROJECT_NAME} Catch2::Catch2)



ament_package()
