# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.21)
project(provant_simulator_zoh)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(provant_simulator_interfaces REQUIRED)

add_library(control_zoh SHARED src/control_inputs_zoh.cpp src/register_control_inputs_zoh.cpp)
target_include_directories(control_zoh PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(control_zoh PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(control_zoh PUBLIC rclcpp::rclcpp)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # target_compile_options(control_zoh PRIVATE -Werror)
endif()

ament_target_dependencies(control_zoh PUBLIC provant_simulator_interfaces std_srvs std_msgs rclcpp_components)
rclcpp_components_register_nodes(control_zoh "provant::ControlInputsZOH")

ament_export_include_directories(include)
ament_export_libraries(export_control_zoh HAS_LIBRARY_TARGET)

add_executable(control_zoh_node src/control_inputs_zoh.cpp src/control_inputs_zoh_node.cpp)
target_include_directories(control_zoh_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(control_zoh_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(control_zoh_node PUBLIC rclcpp::rclcpp)
ament_target_dependencies(control_zoh_node PUBLIC provant_simulator_interfaces std_srvs std_msgs)

install(TARGETS control_zoh control_zoh_node
  EXPORT export_control_zoh
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  #add_launch_test(test/test_control_zoh.py)
endif()

ament_package()
