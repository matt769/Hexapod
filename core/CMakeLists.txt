cmake_minimum_required(VERSION 3.22.1)
project(hexapod_core)

add_compile_options(-std=c++11 -Wall -Wextra -Wpedantic -Wshadow -Wunused
        -Wduplicated-cond -Wduplicated-branches
        -Wlogical-op -Wnull-dereference -Wuseless-cast
        -Wsign-conversion
        )

find_package(ament_cmake REQUIRED)

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

# This doesn't seem like a good way but it's the 'official' way?
# Can't (not advised to) use glob due to not picking up file changes
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "include/hexapod.h;include/leg.h;include/joint.h;include/kinematics_support.h;include/transformations.h;include/receiver.h;include/build_hexapod.h")


add_executable(${PROJECT_NAME}_example
        src/example.cpp)
target_include_directories(${PROJECT_NAME}_example PRIVATE include)
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME})

# message(WARNING ${CMAKE_INSTALL_PREFIX})
install(
        TARGETS ${PROJECT_NAME}
        EXPORT "${PROJECT_NAME}Config"
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
        PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
)
# install(
#         DIRECTORY include
#         DESTINATION include
# )
# INSTALL (
#     DIRECTORY ${CMAKE_SOURCE_DIR}/include/
#     DESTINATION include
#     FILES_MATCHING PATTERN "*.h*")

install(EXPORT
        "${PROJECT_NAME}Config"
        DESTINATION "share/${PROJECT_NAME}/cmake"
        # NAMESPACE ${PROJECT_NAME}::
)

ament_package()
