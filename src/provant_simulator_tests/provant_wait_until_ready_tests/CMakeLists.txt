cmake_minimum_required(VERSION 3.8)
project(provant_wait_until_ready_tests CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(provant_simulator_interfaces REQUIRED)

add_executable(wait_until_ready_tester_node
  src/wait_until_ready_tester_node.cpp
)

ament_target_dependencies(
  wait_until_ready_tester_node
  rclcpp
  std_msgs
  std_srvs
  provant_simulator_interfaces
)

install(
  TARGETS wait_until_ready_tester_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    launch/wait_until_ready_test.launch.py
    TIMEOUT 40
  )
endif()

ament_package()
