// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file simulation_manager_node.cpp
/// @brief This file contains the entry point for the simulation manager node.
///
/// @author Júnio Eduardo de Morais Aquino

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "provant_simulator_simulation_manager/simulation_manager.hpp"

/// @brief Entry point for the simulation manager node.
/// @details Initializes ROS, constructs an instance of the simulation manager node, add it to a
/// single threaded executor and spin it until the user cancels execution.
/// @param argc Number of command line arguments.
/// @param argv List of command line arguments.
/// @return Zero if execution is successful and other integers in case of errors.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<provant::SimulationManager>(rclcpp::NodeOptions{});

  rclcpp::executors::SingleThreadedExecutor executor{};
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
}
