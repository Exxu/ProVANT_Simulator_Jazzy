// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file test_main.cpp
/// @brief This file contains the entry point for the simulation manager node tests.
///
/// @author Júnio Eduardo de Morais Aquino

#include <gtest/gtest.h>

/// @brief Entry point for the simulation manager tests.
/// @details Executes all registered test cases.
/// @param argc Number of command line arguments.
/// @param argv List of command line arguments.
/// @return Zero if the tests are successful and other integers in case of errors.
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
