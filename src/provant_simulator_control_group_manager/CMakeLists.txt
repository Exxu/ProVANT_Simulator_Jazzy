# cmake-format: off
# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
# Copyright 2022 Guilherme V. Raffo
# cmake-format: on

cmake_minimum_required(VERSION 3.21)
project(provant_simulator_control_group_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(provant_simulator_interfaces REQUIRED)

add_library(control_group_manager STATIC src/control_group_manager.cpp)
target_include_directories(
  control_group_manager PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
target_compile_features(control_group_manager PRIVATE c_std_99 cxx_std_17)
ament_target_dependencies(
  control_group_manager
  rclcpp
  provant_simulator_interfaces
  std_srvs
  std_msgs
)

add_executable(${PROJECT_NAME} src/control_group_manager_node.cpp)
target_link_libraries(${PROJECT_NAME} control_group_manager)
target_compile_features(${PROJECT_NAME} PRIVATE c_std_99 cxx_std_17)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    control_group_manager_test
    tests/test_main.cpp
    tests/test_control_group_manager.cpp
    tests/test_control_group_manager_complete.cpp
    tests/test_control_group_manager_edge_cases.cpp
    tests/test_control_group_manager_with_dist_gen.cpp
    tests/test_control_group_manager_with_estimator.cpp
  )
  target_link_libraries(control_group_manager_test control_group_manager)
endif()

ament_package()
