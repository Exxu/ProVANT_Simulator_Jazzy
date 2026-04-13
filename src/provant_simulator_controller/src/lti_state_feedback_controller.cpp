// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file lti_state_feedback_controller.cpp
/// @brief This file contains the implementation of the LTIStateFeedbackController class.
///
/// @author Júnio Eduardo de Morais Aquino

#include "provant_simulator_controller/lti_state_feedback_controller.hpp"

#include <vector>

bool provant::LTIStateFeedbackController::config()
{
  _gains = this->gainMatrix();
  _stateEquilibrium = this->stateEquilibrium();
  _controlInputsEquilibrium = this->controlInputsEquilibrium();

  return true;
}

std::vector<double> provant::LTIStateFeedbackController::execute(
  const std::vector<double> & states, const std::vector<double> & references)
{
  // Convert the states and references to eigen vectors
  _states = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>{
    states.data(), static_cast<int>(states.size())};
  _references = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>{
    references.data(), static_cast<int>(references.size())};

  // Subtract the state equilibrium from the state vector
  _states -= _stateEquilibrium;

  // Obtain the error vector
  _error = _states - _references;
  _error = calculateErrorVector(_error);

  // Calculate the control inputs
  Eigen::VectorXd ctrl = _gains * _error;

  // Add the control inputs equilibrium
  ctrl += _controlInputsEquilibrium;

  // Copy the control inputs and report the vector
  return std::vector<double>{ctrl.data(), ctrl.data() + ctrl.size()};
}

void provant::LTIStateFeedbackController::reset() {}

std::vector<double> provant::LTIStateFeedbackController::getErrorVector() const
{
  return std::vector<double>{_error.data(), _error.data() + _error.size()};
}

std::vector<double> provant::LTIStateFeedbackController::getReferences() const
{
  return std::vector<double>{_references.data(), _references.data() + _references.size()};
}

std::vector<double> provant::LTIStateFeedbackController::getStateVector() const
{
  return std::vector<double>{_states.data(), _states.data() + _states.size()};
}

const Eigen::MatrixXd & provant::LTIStateFeedbackController::getGainMatrix() const
{
  return _gains;
}

const Eigen::VectorXd & provant::LTIStateFeedbackController::getStateEquilibrium() const
{
  return _stateEquilibrium;
}

const Eigen::VectorXd & provant::LTIStateFeedbackController::getControlInputsEquilibrium() const
{
  return _controlInputsEquilibrium;
}

Eigen::VectorXd provant::LTIStateFeedbackController::calculateErrorVector(
  const Eigen::VectorXd & error)
{
  return error;
}
