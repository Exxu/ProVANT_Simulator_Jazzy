// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "provant_step_command_bridge/step_command_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<provant::StepCommandBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}