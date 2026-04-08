// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file test_node.hpp
/// @brief This file contains a test node with the facilities required to test the simulation
/// manager.
///
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_SIMULATION_MANAGER__TESTS__TEST_NODE_HPP_
#define PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_SIMULATION_MANAGER__TESTS__TEST_NODE_HPP_

#include <mutex>
#include <provant_simulator_interfaces/msg/empty.hpp>
#include <provant_simulator_interfaces/srv/control_group_register.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <utility>
#include <vector>

template <class MsgT>
class TopicSubscriber
{
public:
  void onMessage(MsgT msg)
  {
    std::lock_guard lk{_mutex};
    _msgs.emplace_back(std::move(msg));
  }

  const std::vector<MsgT> & messages() const
  {
    std::lock_guard lk{_mutex};
    return _msgs;
  }

private:
  std::vector<MsgT> _msgs;
  mutable std::mutex _mutex;
};

/// @brief A test node for testing the behavior of the simulation manager.
class TestNode : public rclcpp::Node
{
public:
  /// @brief Constructs a new instance of the test node.
  /// @param opts Options for the test node creation.
  /// @param ns The node namespace.
  explicit TestNode(const rclcpp::NodeOptions & opts, const std::string & ns = std::string{})
  : rclcpp::Node("test_node", ns, opts)
  {
    this->setSimulationStatePublisher =
      this->create_publisher<StrMsg>("/provant_simulator/set_simulation_state", 10);

    this->registerGroupClient =
      this->create_client<RegisterSrv>("/provant_simulator/register_group");

    this->simulationStateSubscription = this->create_subscription<StrMsg>(
      "/provant_simulator/simulation_state", 10,
      [this](StrMsg msg) { this->simulationStateMsgs.onMessage(std::move(msg)); });

    this->stepSubscription = this->create_subscription<StepMsg>(
      "/provant_simulator/step", 10, [this](StepMsg msg) { this->stepMsgs.onMessage(msg); });

    this->resetServiceClient = this->create_client<EmptySrv>("/provant_simulator/reset");
  }

  /// @brief The type of the service used by the control register service.
  using RegisterSrv = provant_simulator_interfaces::srv::ControlGroupRegister;
  /// @brief Service used to register control groups.
  rclcpp::Client<RegisterSrv>::SharedPtr registerGroupClient;

  /// @brief The type of the message used in the simulation_state topics.
  using StrMsg = std_msgs::msg::String;
  /// @brief A publisher for the /provant_simulator/set_simulation_state topic.
  rclcpp::Publisher<StrMsg>::SharedPtr setSimulationStatePublisher;

  /// @brief A subscription for the /provant_simulator/simulation_state topic.
  rclcpp::Subscription<StrMsg>::SharedPtr simulationStateSubscription;
  /// @brief The messages received in the /provant_simulator/simulation_state topic.
  TopicSubscriber<StrMsg> simulationStateMsgs;

  /// @brief Type of the message used by the /provant_simulator/step topic.
  using StepMsg = std_msgs::msg::Empty;
  /// @brief A subscription to the /provant_simulator/step topic.
  rclcpp::Subscription<StepMsg>::SharedPtr stepSubscription;
  /// @brief The messages received on the /provant_simulator/step topic.
  TopicSubscriber<StepMsg> stepMsgs;

  /// @brief Type of the service used by the /provant_simulator/reset service.
  using EmptySrv = std_srvs::srv::Empty;
  /// @brief A client for the /provant_simulator/reset service.
  rclcpp::Client<EmptySrv>::SharedPtr resetServiceClient;
};

#endif  // PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_SIMULATION_MANAGER__TESTS__TEST_NODE_HPP_
