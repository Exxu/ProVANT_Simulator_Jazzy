cmake_minimum_required(VERSION 3.8)
project(provant_temporal_loop_tests CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(provant_simulator_interfaces REQUIRED)

find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(temporal_loop_tester_node
  src/temporal_loop_tester_node.cpp
)

ament_target_dependencies(
  temporal_loop_tester_node
  rclcpp
  std_msgs
  std_srvs
  provant_simulator_interfaces
)

target_link_libraries(
  temporal_loop_tester_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

target_compile_features(temporal_loop_tester_node PUBLIC cxx_std_17)

install(
  TARGETS temporal_loop_tester_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    launch/temporal_loop_test.launch.py
    TIMEOUT 50
  )
endif()

ament_package()
