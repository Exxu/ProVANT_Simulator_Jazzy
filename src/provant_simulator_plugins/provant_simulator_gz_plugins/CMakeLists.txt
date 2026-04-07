# cmake-format: off
# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
# Copyright 2022 Guilherme V. Raffo
# cmake-format: on

cmake_minimum_required(VERSION 3.8)
project(provant_simulator_gz_plugins CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(provant_simulator_interfaces REQUIRED)

# Gazebo Harmonic
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

# Add libraries
set(LIB_TARGETS "")

# Add plugins
set(PLUGIN_TARGETS "")

# Legacy Gazebo Classic plugins.

# add_subdirectory(plugins/quadrotor_actuators)
# ament_export_libraries(provant_quadrotor_actuators_plugin)
# set(PLUGIN_TARGETS ${PLUGIN_TARGETS} provant_quadrotor_actuators_plugin)

# add_subdirectory(plugins/quadrotor_state_sensor)
# ament_export_libraries(provant_wait_until_ready_plugin)
# set(PLUGIN_TARGETS ${PLUGIN_TARGETS} provant_wait_until_ready_plugin)

# add_subdirectory(plugins/signal_monitor)
# ament_export_libraries(provant_signal_monitor_plugin)
# set(PLUGIN_TARGETS ${PLUGIN_TARGETS} provant_signal_monitor_plugin)

add_subdirectory(plugins/step_manager_system)
ament_export_libraries(provant_step_manager_system)
set(PLUGIN_TARGETS ${PLUGIN_TARGETS} provant_step_manager_system)

# add_subdirectory(plugins/wait_until_ready)
# ament_export_libraries(provant_quadrotor_state_sensor_plugin)
# set(PLUGIN_TARGETS ${PLUGIN_TARGETS} provant_quadrotor_state_sensor_plugin)

# From https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#building-a-library
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_srvs
  provant_simulator_interfaces
  gz-sim8
  gz-plugin2
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_find_gmock()

  # add_subdirectory(tests)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  install(DIRECTORY include/ DESTINATION include)
endif()

install(
  TARGETS ${LIB_TARGETS} ${PLUGIN_TARGETS}
  EXPORT ${PROJECT_NAME}_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

ament_package()