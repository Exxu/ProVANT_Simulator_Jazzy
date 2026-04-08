# cmake-format: off
# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
# Copyright 2022 Guilherme V. Raffo
# cmake-format: on

add_library(provant_wait_until_ready_system SHARED)

target_include_directories(
    provant_wait_until_ready_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(
  provant_wait_until_ready_system PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_sources(
  provant_wait_until_ready_system PRIVATE
  src/wait_until_ready_system.cpp
)

ament_target_dependencies(
  provant_wait_until_ready_system
  rclcpp
  std_msgs
)

target_link_libraries(provant_wait_until_ready_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

target_compile_features(provant_wait_until_ready_system PUBLIC cxx_std_17)

install(
  TARGETS provant_wait_until_ready_system
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)