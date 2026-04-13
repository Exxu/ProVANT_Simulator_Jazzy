// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file mock_control_strategy.hpp
/// @brief
///
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROLLER__TESTS__MOCK_CONTROL_STRATEGY_HPP_
#define PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROLLER__TESTS__MOCK_CONTROL_STRATEGY_HPP_

#include <gmock/gmock.h>

#include <vector>

#include "provant_simulator_controller/controller_interface.hpp"

namespace provant
{
/// @brief A Mock control strategy to be used while testing the controller node.
class MockControlStrategy : public IController
{
public:
  MOCK_METHOD(bool, configImpl, (), ());
  MOCK_METHOD(
    std::vector<double>, executeImpl,
    (const std::vector<double> & states, const std::vector<double> & references), ());
  MOCK_METHOD(std::vector<double>, getErrorVectorImpl, (), (const));
  MOCK_METHOD(std::vector<double>, getReferencesImpl, (), (const));
  MOCK_METHOD(std::vector<double>, getStateVectorImpl, (), (const));
  MOCK_METHOD(void, resetImpl, (), ());

protected:
  bool config() override { return this->configImpl(); }

  std::vector<double> execute(
    const std::vector<double> & states, const std::vector<double> & references) override
  {
    return this->executeImpl(states, references);
  }

  std::vector<double> getErrorVector() const override { return this->getErrorVectorImpl(); }

  std::vector<double> getReferences() const override { return this->getReferencesImpl(); }

  std::vector<double> getStateVector() const override { return this->getStateVectorImpl(); }

  void reset() override { this->resetImpl(); }
};
}  // namespace provant

#endif  // PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROLLER__TESTS__MOCK_CONTROL_STRATEGY_HPP_
