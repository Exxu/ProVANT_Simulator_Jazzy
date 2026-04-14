# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

cmake_minimum_required(VERSION 3.21)
project(provant_simulator_reference_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(provant_simulator_interfaces REQUIRED)

add_library(ref_gen SHARED src/base_reference_generator.cpp src/reference_generator.cpp src/register_reference_generator.cpp)
target_include_directories(ref_gen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ref_gen PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(ref_gen PUBLIC rclcpp::rclcpp)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # target_compile_options(ref_gen PRIVATE -Werror)
endif()

ament_target_dependencies(ref_gen PUBLIC provant_simulator_interfaces std_srvs rclcpp_components)
rclcpp_components_register_nodes(ref_gen "provant::ReferenceGenerator")

ament_export_include_directories(include)
ament_export_libraries(export_ref_gen HAS_LIBRARY_TARGET)

add_executable(ref_gen_node src/base_reference_generator.cpp src/reference_generator.cpp src/reference_generator_node.cpp)
target_include_directories(ref_gen_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ref_gen_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(ref_gen_node PUBLIC rclcpp::rclcpp)
ament_target_dependencies(ref_gen_node PUBLIC provant_simulator_interfaces std_srvs)

add_executable(joystick_ref_gen_node src/base_reference_generator.cpp src/joystick_reference_generator.cpp src/joystick_reference_generator_node.cpp)
target_include_directories(joystick_ref_gen_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joystick_ref_gen_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(joystick_ref_gen_node PUBLIC rclcpp::rclcpp)
ament_target_dependencies(joystick_ref_gen_node PUBLIC provant_simulator_interfaces std_srvs sensor_msgs)

install(TARGETS ref_gen ref_gen_node joystick_ref_gen_node
  EXPORT export_ref_gen
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  #add_launch_test(test/test_ref_gen.py)
endif()

ament_package()