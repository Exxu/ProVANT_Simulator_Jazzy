// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file control_inputs_zoh.hpp
/// @brief This file contains the declaration of the ControlInputsZOH class.
///
/// @author Pedro Otávio Fonseca Pires
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_CONTROL_INPUTS_ZOH__CONTROL_INPUTS_ZOH_HPP_
#define PROVANT_SIMULATOR_CONTROL_INPUTS_ZOH__CONTROL_INPUTS_ZOH_HPP_

#include <chrono>
#include <vector>
#include <utility>
#include <provant_simulator_interfaces/msg/float64_array.hpp>
#include <provant_simulator_interfaces/msg/empty.hpp>
#include <provant_simulator_interfaces/msg/actuator.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/empty.hpp>

namespace provant::zoh
{
/// @brief The ControlInputsZOH is a ROS node that creates a Zero Order Hold (ZOH) for the control inputs signals.
class ControlInputsZOH : public rclcpp::Node
{
public:
  /// @brief Initializes a ControlInputsZOH Node.
  ///@details It reads the values of the <initial_value>
  /// and <actuators> parameters and stores them.
  /// Then it subscribes to the "zoh_trigger"
  /// and "control_inputs" topics, provides a server for the control_zoh/reset service and sets
  /// the control inputs values according to the <initial_value> parameter.
  /// @param opts Node options to pass to rclcpp::Node.
  explicit ControlInputsZOH(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{});

private:
  /// @brief Type of the message used for zoh_trigger topic.
  using ZOHTriggerMsg = provant_simulator_interfaces::msg::Empty;
  /// @brief Type of the message used in the control_inputs topic.
  using FloatArrayMsg = provant_simulator_interfaces::msg::Float64Array;
  /// @brief Type of the message used on the reset server.
  using EmptySrv = std_srvs::srv::Empty;
  /// @brief clock used to measure the computation time of the operations.
  using Clock = std::chrono::high_resolution_clock;

  /// @brief Stores the value of the <initial_value> parameter.
  std::vector<double> _initialValues;

  /// @brief Stores the name of the topics that shall receive the control input updates.
  std::vector<std::string> _actuatorTopicNames;

  /// @brief Stores the latest values received on the control_inputs topic.
  std::vector<double> _mostRecentControlInputs;

  /// @brief Subscriber to the control_inputs topic.
  rclcpp::Subscription<FloatArrayMsg>::SharedPtr _controlInputsSubscription;
  /// @brief Subscriber for the zoh_trigger topic.
  rclcpp::Subscription<ZOHTriggerMsg>::SharedPtr _zohTriggerSubscription;

  /// @brief Publisher for the control_inputs topic.
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr _controlInputsPublisher;

  /// @brief @brief ControlInputsZOH reset service server.
  /// @details When called, the reset service will set the values of the control inputs to the
  /// initial value and publish the values for each registered control input.
  rclcpp::Service<EmptySrv>::SharedPtr _resetServiceServer;

  /// @throws std::runtime_error If the actuators parameter contains an empty list.
  void readActuatorTopicNames();

  void readInitialValues(std::size_t expectedSize);

  using ActuatorMessage = provant_simulator_interfaces::msg::Actuator;
  using ActuatorPublisherPtr = rclcpp::Publisher<ActuatorMessage>::SharedPtr;

  std::vector<ActuatorPublisherPtr> _actuatorPublishers;

  void advertiseActuatorTopics(const rclcpp::QoS & qos);
  void createSubscriptions(const rclcpp::QoS & qos);
  void provideResetServer(const rclcpp::QoS & qos);
  void publishMostRecentControlInputs(
    uint32_t step, const rclcpp::Time & simTime, const Clock::time_point & rxTimestamp);

  /// @brief Update the control_inputs values when a new message is received on the control_inputs topic
  /// and publishes the most recent control inputs values for each registered control input.
  /// @param msg The FloatArrayMsg message received on the control_inputs topic.
  void onControlInputsMsg(FloatArrayMsg::UniquePtr msg);
  /// @brief When a message is received in the zoh_trigger topic, then the Control Inputs ZOH publishes
  /// the most recent control_inputs value for each registered control input.
  /// @param msg The EmptyMsg message received on the zoh_trigger topic.
  void onZohTriggerMsg(ZOHTriggerMsg::UniquePtr msg);
  /// @brief Reset the values of the control inputs to the initial value.
  void onReset();
};
}  // namespace provant::zoh

#endif  // PROVANT_SIMULATOR_CONTROL_INPUTS_ZOH__CONTROL_INPUTS_ZOH_HPP_
