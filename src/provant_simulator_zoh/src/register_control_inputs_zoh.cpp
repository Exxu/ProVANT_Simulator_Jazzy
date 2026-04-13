// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file register_control_inputs_zoh.cpp
/// @brief This file registers the ControlInputsZOH node as a ROS2 composable node.
///
/// @author Pedro Otávio Fonseca Pires


// clang-format off
#include "provant_simulator_zoh/control_inputs_zoh.hpp" // NOLINT
#include <rclcpp_components/register_node_macro.hpp> // NOLINT
#include <string> // NOLINT
// clang-format on
RCLCPP_COMPONENTS_REGISTER_NODE(provant::zoh::ControlInputsZOH)
