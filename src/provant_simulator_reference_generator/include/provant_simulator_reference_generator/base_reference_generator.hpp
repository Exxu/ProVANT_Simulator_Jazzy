/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 * Copyright 2022 Guilherme V. Raffo
 */
/**
 * @file ref_gen.hpp
 * @brief This file contains the declaration of the BaseReferenceGenerator class.
 *
 * @author Pedro Otávio Fonseca Pires
 */

#ifndef PROVANT_SIMULATOR_REFERENCE_GENERATOR__BASE_REFERENCE_GENERATOR_HPP_
#define PROVANT_SIMULATOR_REFERENCE_GENERATOR__BASE_REFERENCE_GENERATOR_HPP_

#include <provant_simulator_interfaces/msg/float64_array.hpp>
#include <provant_simulator_interfaces/msg/step_clock.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>

namespace provant::ref_gen
{
class BaseReferenceGenerator : public rclcpp::Node
{
public:
  explicit BaseReferenceGenerator(const rclcpp::NodeOptions & options);

protected:
  virtual std::vector<double> computeReferences(uint32_t step, rclcpp::Time t) = 0;
  virtual void reset() = 0;

private:
  using StepClockMsg = provant_simulator_interfaces::msg::StepClock;

  rclcpp::Subscription<StepClockMsg>::SharedPtr _stepClockSubscription;
  rclcpp::Publisher<provant_simulator_interfaces::msg::Float64Array>::SharedPtr _publisher;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _resetServiceServer;
  rclcpp::Clock _systemClock;
};
}  // namespace provant::ref_gen

#endif  // PROVANT_SIMULATOR_REFERENCE_GENERATOR__BASE_REFERENCE_GENERATOR_HPP_
