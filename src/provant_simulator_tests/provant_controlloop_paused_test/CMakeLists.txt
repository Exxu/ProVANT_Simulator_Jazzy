cmake_minimum_required(VERSION 3.8)
project(provant_controlloop_test CXX)

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

add_executable(controlloop_tester_node src/controlloop_tester_node.cpp)
ament_target_dependencies(controlloop_tester_node rclcpp std_msgs std_srvs provant_simulator_interfaces)
target_link_libraries(controlloop_tester_node gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
target_compile_features(controlloop_tester_node PUBLIC cxx_std_17)

add_executable(fake_controller_node src/fake_controller_node.cpp)
ament_target_dependencies(fake_controller_node rclcpp provant_simulator_interfaces)
target_compile_features(fake_controller_node PUBLIC cxx_std_17)

add_executable(fake_disturbance_node src/fake_disturbance_node.cpp)
ament_target_dependencies(fake_disturbance_node rclcpp provant_simulator_interfaces)
target_compile_features(fake_disturbance_node PUBLIC cxx_std_17)

install(TARGETS
  controlloop_tester_node
  fake_controller_node
  fake_disturbance_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(launch/controlloop_test.launch.py TIMEOUT 60)
endif()

ament_package()
