// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file control_inputs_zoh_node.cpp
/// @brief This file contains the main function that spins the ControlInputsZOH Node.
///
/// @author Pedro Otávio Fonseca Pires

#include <memory>

#include "provant_simulator_zoh/control_inputs_zoh.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<provant::zoh::ControlInputsZOH>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();

  return 0;
}
