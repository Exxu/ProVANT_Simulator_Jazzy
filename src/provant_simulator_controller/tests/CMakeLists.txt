ament_add_gmock(
  test_controller
  test_controller.cpp
  test_lti_state_feedback.cpp
  test_main.cpp
  WORKING_DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(test_controller ${PROJECT_NAME})
ament_target_dependencies(
  test_controller
  rclcpp
  provant_simulator_interfaces
  std_srvs
)
