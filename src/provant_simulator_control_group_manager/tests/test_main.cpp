// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file test_main.cpp
/// @brief This file contains the entry point for the test cases of the ProVANT Simulator Control
/// Group Manager.
///
/// @author Júnio Eduardo de Morais Aquino

#include <gtest/gtest.h>

#include <iostream>
#include <rclcpp/rclcpp.hpp>

/// @brief Entry point for the ProVANT Simulator Control Group Manager tests.
/// @param argc Number of command line arguments for the program.
/// @param argv List of command line arguments for the program.
/// @return Test execution code. 0 in case of success and other integers in case of error.
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  try {
    return RUN_ALL_TESTS();
  } catch (const std::exception & e) {
    std::cerr << "While running the tests, an unexpected exception with message \"" << e.what()
              << "\" was caught.";
  }

  return 0;
}
