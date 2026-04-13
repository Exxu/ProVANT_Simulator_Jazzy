cmake_minimum_required(VERSION 3.8)
project(provant_controller_zoh_loop_test CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(provant_simulator_interfaces REQUIRED)
find_package(provant_simulator_controller REQUIRED)

find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(controller_input_source_node src/controller_input_source_node.cpp)
ament_target_dependencies(controller_input_source_node rclcpp std_srvs provant_simulator_interfaces)
target_compile_features(controller_input_source_node PUBLIC cxx_std_17)

add_executable(test_controller_node src/test_controller_node.cpp)
ament_target_dependencies(test_controller_node rclcpp provant_simulator_interfaces provant_simulator_controller)
target_compile_features(test_controller_node PUBLIC cxx_std_17)

add_executable(provant_controller_zoh_loop_tester_node src/provant_controller_zoh_loop_tester_node.cpp)
ament_target_dependencies(provant_controller_zoh_loop_tester_node rclcpp std_msgs provant_simulator_interfaces)
target_link_libraries(provant_controller_zoh_loop_tester_node gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 Eigen3::Eigen)
target_compile_features(provant_controller_zoh_loop_tester_node PUBLIC cxx_std_17)

install(TARGETS
  controller_input_source_node
  test_controller_node
  provant_controller_zoh_loop_tester_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(launch/provant_controller_zoh_loop_test.launch.py TIMEOUT 90)
endif()

ament_package()
