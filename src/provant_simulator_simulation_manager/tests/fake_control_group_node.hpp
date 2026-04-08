// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file fake_control_group_node.hpp
/// @brief This file contains a fake control group manager node that is used for testing the
/// simulation manager.
///
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_SIMULATION_MANAGER__TESTS__FAKE_CONTROL_GROUP_NODE_HPP_
#define PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_SIMULATION_MANAGER__TESTS__FAKE_CONTROL_GROUP_NODE_HPP_

#include <mutex>
#include <provant_simulator_interfaces/msg/empty.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <vector>

/// @brief A fake control group manager node for testing the simulation manager.
class FakeControlGroup : public rclcpp::Node
{
public:
  /// @brief Constructs an instance of a Fake Control Group node.
  /// @param ns Namespace of the group.
  /// @param opts Options for the node creation.
  explicit FakeControlGroup(
    const std::string & ns, const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{})
  : rclcpp::Node("fake_control_group_manager", ns, opts)
  {
    this->resetServiceServer = this->create_service<EmptySrv>(
      "reset", [this](const EmptySrv::Request::SharedPtr req, EmptySrv::Response::SharedPtr res) {
        (void)res;
        this->onResetServiceCall(req);
      });

    this->readyPublisher = this->create_publisher<EmptyMsg>("ready", 10);
  }

  /// @brief Type of the service used by the reset service.
  using EmptySrv = std_srvs::srv::Empty;
  /// @brief Provides a server for the resetServiceServer.
  rclcpp::Service<EmptySrv>::SharedPtr resetServiceServer;
  /// @brief The requests received by the reset service.
  std::vector<EmptySrv::Request> resetServiceRequests;

  /// @brief Type of the messages used by the ready topic.
  using EmptyMsg = provant_simulator_interfaces::msg::Empty;
  /// @brief A publisher for the ready topic.
  rclcpp::Publisher<EmptyMsg>::SharedPtr readyPublisher;

  /// @brief Publish a message on the ready topic signaling that this control group is ready.
  void publishReady()
  {
    EmptyMsg msg{};
    this->readyPublisher->publish(msg);
  }

private:
  /// @brief Protects access to the resetServiceRequests variable.
  std::mutex _resetServerMutex;

  /// @brief Add the request to the list of reset requests.
  /// @param req The request message received by the reset service.
  void onResetServiceCall(const EmptySrv::Request::SharedPtr & req)
  {
    std::lock_guard lk{_resetServerMutex};
    resetServiceRequests.emplace_back(*req);
  }
};

#endif  // PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_SIMULATION_MANAGER__TESTS__FAKE_CONTROL_GROUP_NODE_HPP_
