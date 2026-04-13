// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file test_controller_node.hpp
/// @brief This file contains a node with the necessary facilities to test the controller node.
///
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROLLER__TESTS__TEST_CONTROLLER_NODE_HPP_
#define PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROLLER__TESTS__TEST_CONTROLLER_NODE_HPP_

#include <mutex>
#include <provant_simulator_interfaces/msg/float64_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <utility>
#include <vector>

/// @brief ROS node for testing the controller node.
class TestControllerNode : public rclcpp::Node
{
public:
  /// @brief Construct a new instance of the test controller node.
  /// @param ns Namespace of the node.
  /// @param opts Node options.
  TestControllerNode(
    const std::string & ns = "", const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{})
  : rclcpp::Node("test_node", ns, opts)
  {
    this->referencesPublisher = this->create_publisher<FloatArrayMsg>("references", 10);
    this->stateVectorPublisher = this->create_publisher<FloatArrayMsg>("state_vector", 10);
    this->controlInputsSubscription = this->create_subscription<FloatArrayMsg>(
      "control_inputs", 10,
      [this](FloatArrayMsg msg) { this->onControlInputsMsgs(std::move(msg)); });
    this->referencesSubscription = this->create_subscription<FloatArrayMsg>(
      "controller/references", 10,
      [this](FloatArrayMsg msg) { this->onReferencesMessage(std::move(msg)); });
    this->stateVecSubscription = this->create_subscription<FloatArrayMsg>(
      "controller/state_vector", 10,
      [this](FloatArrayMsg msg) { this->onStateVectorMsgs(std::move(msg)); });
    this->errorVecSubscription = this->create_subscription<FloatArrayMsg>(
      "controller/error_vector", 10,
      [this](FloatArrayMsg msg) { this->onErrorMsgs(std::move(msg)); });
    this->resetServiceClient = this->create_client<EmptySrv>("reset");
  }

  /// @brief Type of message used to receive arrays of data.
  using FloatArrayMsg = provant_simulator_interfaces::msg::Float64Array;
  /// @brief Type of service used in the reset service.
  using EmptySrv = std_srvs::srv::Empty;

  /// @brief ROS publisher for the references topic.
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr referencesPublisher;

  /// @brief Publish a message on the references topic.
  /// @param data Reference values.
  /// @param step Current simulation step.
  /// @param time Current simulation time.
  void publishReferences(
    const std::vector<double> & data, uint32_t step = 0,
    const rclcpp::Time & time = rclcpp::Time{0, 0, RCL_ROS_TIME})
  {
    FloatArrayMsg msg{};
    msg.data = data;
    msg.pheader.step = step;
    msg.pheader.timestamp = time;

    this->referencesPublisher->publish(msg);
  }

  /// @brief ROS publisher for the state_vector topic.
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr stateVectorPublisher;

  /// @brief Publish a message on the state_vector topic.
  /// @param data State vector data.
  /// @param step Current simulation step.
  /// @param time Current simulation time.
  void publishStateVector(
    const std::vector<double> & data, uint32_t step = 0,
    const rclcpp::Time & time = rclcpp::Time{0, 0, RCL_ROS_TIME})
  {
    FloatArrayMsg msg{};
    msg.data = data;
    msg.pheader.step = step;
    msg.pheader.timestamp = time;

    this->stateVectorPublisher->publish(msg);
  }

  /// @brief Stores the control inputs messages received in the control_inputs topic.
  std::vector<FloatArrayMsg> controlInputsMsgs;
  /// @brief Subscription for the control_inputs topic.
  rclcpp::Subscription<FloatArrayMsg>::SharedPtr controlInputsSubscription;

  /// @brief Stores the reference log messages published by the controller.
  std::vector<FloatArrayMsg> refMsgs;
  /// @brief Subscription for the controller/references topic.
  rclcpp::Subscription<FloatArrayMsg>::SharedPtr referencesSubscription;

  /// @brief Stores the log state vectors messages published by the controller.
  std::vector<FloatArrayMsg> stateVecMsgs;
  /// @brief Subscription for the controller/state_vector topic.
  rclcpp::Subscription<FloatArrayMsg>::SharedPtr stateVecSubscription;

  /// @brief Stores the error vector log messages published by the controller.
  std::vector<FloatArrayMsg> errorVecMsgs;
  /// @brief Subscription for the controller/error_vector topic.
  rclcpp::Subscription<FloatArrayMsg>::SharedPtr errorVecSubscription;

  /// @brief Client for the reset service.
  rclcpp::Client<EmptySrv>::SharedPtr resetServiceClient;

protected:
  void onControlInputsMsgs(FloatArrayMsg msg)
  {
    std::lock_guard lk{_controlInputsMutex};
    controlInputsMsgs.push_back(std::move(msg));
  }

  void onReferencesMessage(FloatArrayMsg msg)
  {
    std::lock_guard lk{_refMutex};
    refMsgs.push_back(std::move(msg));
  }

  void onStateVectorMsgs(FloatArrayMsg msg)
  {
    std::lock_guard lk{_stateVecMutex};
    stateVecMsgs.push_back(std::move(msg));
  }

  void onErrorMsgs(FloatArrayMsg msg)
  {
    std::lock_guard lk{_errorMutex};
    errorVecMsgs.push_back(std::move(msg));
  }

private:
  std::mutex _controlInputsMutex;
  std::mutex _refMutex;
  std::mutex _stateVecMutex;
  std::mutex _errorMutex;
};

#endif  // PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROLLER__TESTS__TEST_CONTROLLER_NODE_HPP_
