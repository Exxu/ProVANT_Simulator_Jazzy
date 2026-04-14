/*
* This file is part of the ProVANT simulator project.
* Licensed under the terms of the MIT open source license. More details at
* https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
* Copyright 2022 Guilherme V. Raffo
*/
/**
* @file reference_generator.cpp
* @brief This file contains the implementation of the ReferenceGenerator class.
*
* @author Pedro Otávio Fonseca Pires
*/

#include "provant_simulator_reference_generator/reference_generator.hpp"

#include <cmath>
#include <vector>

using provant::ref_gen::ReferenceGenerator;

std::vector<double> ReferenceGenerator::computeReferences(uint32_t /*step*/, rclcpp::Time t)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "Computing the reference vector.");

  auto const sec = static_cast<double>(t.nanoseconds()) / 1e9;
  return {
    sin(sec / 2),         // x
    cos(sec / 2),         // y
    sec / 10,             // z
    0,                    // roll angle
    0,                    // pitch angle
    0,                    // yaw angle
    0.5 * cos(sec / 2),   // x_dot
    -0.5 * sin(sec / 2),  // y_dot
    0.1,                  // z_dot
    0,                    // roll_dot
    0,                    // pitch_dot
    0                     // yaw_dot
  };
}

void ReferenceGenerator::reset() { RCLCPP_DEBUG_STREAM(get_logger(), "Reset method called."); }
