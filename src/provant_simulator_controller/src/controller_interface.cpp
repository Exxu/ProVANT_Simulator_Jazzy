// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file controller_interface.cpp
/// @brief
///
/// @author Júnio Eduardo de Morais Aquino

#include "provant_simulator_controller/controller_interface.hpp"

#include <vector>

bool provant::IController::setUp() { return this->config(); }

bool provant::IController::config() { return true; }

std::vector<double> provant::IController::compute(
  const std::vector<double> & states, const std::vector<double> & references)
{
  return this->execute(states, references);
}

std::vector<double> provant::IController::references() const { return this->getReferences(); }

std::vector<double> provant::IController::stateVector() const { return this->getStateVector(); }

std::vector<double> provant::IController::errorVector() const { return this->getErrorVector(); }

void provant::IController::resetControlStrategy() { this->reset(); }
