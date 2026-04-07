# cmake-format: off
# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
# Copyright 2022 Guilherme V. Raffo
# cmake-format: on

add_library(provant_step_manager_system SHARED)

target_include_directories(
  provant_step_manager_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(
  provant_step_manager_system PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_sources(
  provant_step_manager_system PRIVATE
  src/step_manager_system.cpp
)

ament_target_dependencies(
  provant_step_manager_system
  rclcpp
  std_srvs
  provant_simulator_interfaces
)

target_link_libraries(
  provant_step_manager_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}/
)