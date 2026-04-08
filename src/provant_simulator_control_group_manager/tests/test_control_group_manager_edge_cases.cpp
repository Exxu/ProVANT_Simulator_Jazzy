// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file test_control_group_manager.cpp
/// @brief This file contains the tests cases for complex edge cases of the control group manager.
///
/// This file is intended to contain the death tests for the error cases of the control group
/// manager.
///
/// @author Júnio Eduardo de Morais Aquino

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "provant_simulator_control_group_manager/control_group_manager.hpp"
#include "test_fixture.hpp"

class ControlGroupManagerEdgeCasesTest : public ControlGroupTestFixture
{
};
