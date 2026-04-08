// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file test_fixture.hpp
/// @brief This file contains a text fixture that can be used to test the Control Group Manager
/// node.
///
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROL_GROUP_MANAGER__TESTS__TEST_FIXTURE_HPP_
#define PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROL_GROUP_MANAGER__TESTS__TEST_FIXTURE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "test_node.hpp"

/// @brief A test fixture to test the Control Group Manager node.
/// @details Contains a Control Group Manager node, a test node and an executor to allow execution
/// of the tests.
class ControlGroupTestFixture : public testing::Test
{
public:
  /// @brief Initializes the default rclcpp context.
  [[maybe_unused]] static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  /// @brief Shutdown the default rclcpp context.
  [[maybe_unused]] static void TearDownTestCase() { rclcpp::shutdown(); }

protected:
  /// @brief Creates an instance of a control group manager and a test node.
  void SetUp() override
  {
    Test::SetUp();

    auto const opts = getOptions();

    this->node = std::make_shared<provant::ControlGroupManager>(opts);
    this->executor.add_node(this->node);

    this->testNode = std::make_shared<provant::testing::ControlGroupManagerTestNode>();
    this->executor.add_node(this->testNode);
  }

  /// @brief Destroys the control group manager and test nodes.
  void TearDown() override
  {
    Test::TearDown();

    this->executor.cancel();
    this->node.reset();
    this->testNode.reset();
  }

  /// @brief The set of NodeOptions to pass to the Control Group Manager node.
  /// @details If testing variations of a control group manager, such as with disturbance
  /// generators and state estimators, this method can be overridden to specify the desired
  /// parameters.
  virtual rclcpp::NodeOptions getOptions()
  {
    rclcpp::NodeOptions opts;

    std::vector<rclcpp::Parameter> parameters;
    parameters.reserve(3);

    parameters.emplace_back("control_step", 0.003);
    parameters.emplace_back("has_disturbances", false);
    parameters.emplace_back("has_estimator", false);

    opts.parameter_overrides(parameters);

    std::vector<std::string> arguments;
    arguments.reserve(3);

    arguments.emplace_back("--ros-args");
    arguments.emplace_back("--log-level");
    arguments.emplace_back("debug");

    opts.arguments(arguments);

    return opts;
  }

  /// @brief An instance of a control group manager node.
  std::shared_ptr<provant::ControlGroupManager> node{};
  /// @brief A node containing the required facilities to test the control group manager.
  std::shared_ptr<provant::testing::ControlGroupManagerTestNode> testNode;
  /// @brief An executor to spin the control group manager and test nodes.
  rclcpp::executors::SingleThreadedExecutor executor{};
};

#endif  // PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROL_GROUP_MANAGER__TESTS__TEST_FIXTURE_HPP_
