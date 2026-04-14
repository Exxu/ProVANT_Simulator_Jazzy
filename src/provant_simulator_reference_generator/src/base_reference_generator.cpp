/*
* This file is part of the ProVANT simulator project.
* Licensed under the terms of the MIT open source license. More details at
* https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
* Copyright 2022 Guilherme V. Raffo
*/
/**
* @file base_reference_generator.cpp
* @brief This file contains the implementation of the BaseReferenceGenerator class.
*
* @author Pedro Otávio Fonseca Pires
*/

#include "provant_simulator_reference_generator/base_reference_generator.hpp"

#include <chrono>
#include <utility>

using provant::ref_gen::BaseReferenceGenerator;

BaseReferenceGenerator::BaseReferenceGenerator(const rclcpp::NodeOptions & options)
: rclcpp::Node("reference_generator", options), _systemClock{rcl_clock_type_t::RCL_SYSTEM_TIME}
{
  using EmptyRequestPtr = std_srvs::srv::Empty::Request::SharedPtr;
  using EmptyResponsePtr = std_srvs::srv::Empty::Response::SharedPtr;
  using StepClockMsgPtr = provant_simulator_interfaces::msg::StepClock::UniquePtr;

  // Log initialization
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Initializing the " << this->get_fully_qualified_name() << " reference generator node.");

  // Provide a server for the reset service
  RCLCPP_DEBUG(this->get_logger(), "Creating server for the ref_gen/reset service.");
  _resetServiceServer = this->create_service<std_srvs::srv::Empty>(
    "ref_gen/reset", [this](const EmptyRequestPtr /*request*/, EmptyResponsePtr /*response*/) {
      RCLCPP_DEBUG(this->get_logger(), "Reset service invoked, calling the reset method.");
      this->reset();
    });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Published server for the " << _resetServiceServer->get_service_name() << " service.");

  // Advertise to the References topic
  RCLCPP_DEBUG(this->get_logger(), "Advertising to the references topic.");
  _publisher =
    this->create_publisher<provant_simulator_interfaces::msg::Float64Array>("references", 10);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(), "Advertised to the " << _publisher->get_topic_name() << " topic.");

  // Subscribe to the step_clock topic
  RCLCPP_DEBUG(this->get_logger(), "Subscribing to the step_clock topic.");
  _stepClockSubscription = this->create_subscription<provant_simulator_interfaces::msg::StepClock>(
    "step_clock", 10, [this](StepClockMsgPtr msg) {
      using RefMsg = provant_simulator_interfaces::msg::Float64Array;
      using clock = std::chrono::high_resolution_clock;

      RefMsg res{};

      // Compute the References, and store the computation time
      auto const start = clock::now();
      res.data = this->computeReferences(msg->step, msg->time);
      auto const end = clock::now();

      // Fill the message header
      res.pheader.step = msg->step;
      res.pheader.computation_time = rclcpp::Duration(end - start);
      res.pheader.timestamp = _systemClock.now();

      // Publish the message
      this->_publisher->publish(res);
    });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Subscribed to the " << _stepClockSubscription->get_topic_name() << " topic.");

  RCLCPP_INFO_STREAM(this->get_logger(), "Node initialized successfully");
}
