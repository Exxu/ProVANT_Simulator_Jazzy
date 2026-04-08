// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file control_group_manager_node.cpp
/// @brief This file defines the entry point for the Control Group Manager node.
///
/// @author Júnio Eduardo de Morais Aquino

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "provant_simulator_control_group_manager/control_group_manager.hpp"

/// @brief Entry point for the Control Group Manager node.
/// Create an instance of the Control Group Manager node, and spin it using a single thread
/// executor.
/// @param argc Number of program arguments.
/// @param argv List of program arguments.
/// @return Program execution status. Zero if normal execution, and other numbers indicate failures.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<provant::ControlGroupManager>();

  RCLCPP_DEBUG(node->get_logger(), "Starting executor");
  rclcpp::executors::SingleThreadedExecutor executor{};
  RCLCPP_DEBUG(node->get_logger(), "Adding node to executor");
  executor.add_node(node);
  RCLCPP_DEBUG(node->get_logger(), "Spinning executor");
  executor.spin();

  RCLCPP_DEBUG(node->get_logger(), "Shutting down");
  rclcpp::shutdown();

  return 0;
}
