/*
* This file is part of the ProVANT simulator project.
* Licensed under the terms of the MIT open source license. More details at
* https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
* Copyright 2022 Guilherme V. Raffo
*/
/**
* @file Reference_generator.hpp
* @brief This file contains the declaration of the ReferenceGenerator class.
*
* @author Pedro Otávio Fonseca Pires
*/

#ifndef PROVANT_SIMULATOR_REFERENCE_GENERATOR__REFERENCE_GENERATOR_HPP_
#define PROVANT_SIMULATOR_REFERENCE_GENERATOR__REFERENCE_GENERATOR_HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <vector>

#include "provant_simulator_reference_generator/base_reference_generator.hpp"

namespace provant::ref_gen
{
class ReferenceGenerator : public BaseReferenceGenerator
{
  using BaseReferenceGenerator::BaseReferenceGenerator;

protected:
  std::vector<double> computeReferences(uint32_t step, rclcpp::Time t) override;
  void reset() override;
};
}  // namespace provant::ref_gen

#endif  // PROVANT_SIMULATOR_REFERENCE_GENERATOR__REFERENCE_GENERATOR_HPP_
