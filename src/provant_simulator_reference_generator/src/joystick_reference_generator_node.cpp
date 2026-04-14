/*
* This file is part of the ProVANT simulator project.
* Licensed under the terms of the MIT open source license. More details at
* https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
* Copyright 2022 Guilherme V. Raffo
*/
/**
* @file joystick_reference_generator_node.cpp
* @brief
*
* @author Lorena Oliveira
* @author Guilherme Barbosa
*/

#include <memory>

#include "provant_simulator_reference_generator/joystick_reference_generator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<provant::ref_gen::JoystickReferenceGenerator>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();

  return 0;
}
