# cmake-format: off
# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
# cmake-format: on

cmake_minimum_required(VERSION 3.8)
project(provant_simulator_step_command_bridge CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Gazebo vendor packages for ROS 2 Jazzy
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)

# Gazebo Harmonic libraries
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(provant_step_command_bridge
  src/step_command_bridge.cpp
  src/step_command_bridge_node.cpp
)

target_include_directories(provant_step_command_bridge
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  provant_step_command_bridge
  rclcpp
  std_msgs
)

target_link_libraries(
  provant_step_command_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

target_compile_features(provant_step_command_bridge PUBLIC cxx_std_17)

install(
  TARGETS provant_step_command_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()