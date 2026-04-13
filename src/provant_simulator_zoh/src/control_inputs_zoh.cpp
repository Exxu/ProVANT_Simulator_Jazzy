// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file control_inputs_zoh.cpp
/// @brief This file contains the implementation of the ControlInputsZOH class.
///
/// @author Pedro Otávio Fonseca Pires
/// @author Júnio Eduardo de Morais Aquino

#include "provant_simulator_zoh/control_inputs_zoh.hpp"

using provant::zoh::ControlInputsZOH;

ControlInputsZOH::ControlInputsZOH(const rclcpp::NodeOptions & options)
: rclcpp::Node("control_inputs_zoh", options)
{
  // Log initialization
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Initializing the " << this->get_fully_qualified_name() << " control inputs zoh node.");

  // Read the list of the topic names used to communicate with the actuators
  this->readActuatorTopicNames();

  // Read the initial values for the actuators
  this->readInitialValues(_actuatorTopicNames.size());

  // Set the control inputs values according to the <initial_value> parameter
  _mostRecentControlInputs = _initialValues;

  // Define the QoS to use in subscriptions
  const rclcpp::QoS qos{10};

  // Advertise the actuator topics
  this->advertiseActuatorTopics(qos);

  // Provide a server for the reset service
  this->provideResetServer(qos);

  // Subscribe to the zoh_trigger and control_inputs topics
  this->createSubscriptions(qos);

  // Node created!
  RCLCPP_INFO_STREAM(this->get_logger(), "Node initialized successfully");
}

void ControlInputsZOH::createSubscriptions(const rclcpp::QoS & qos)
{
  // Subscribe to the control_inputs topic
  RCLCPP_DEBUG(this->get_logger(), "Subscribing to the control_inputs topic.");
  _controlInputsSubscription = this->create_subscription<FloatArrayMsg>(
    "control_inputs", qos,
    [this](FloatArrayMsg::UniquePtr msg) { this->onControlInputsMsg(std::move(msg)); });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Subscribed to the " << _controlInputsSubscription->get_topic_name() << " topic.");

  // Subscribe to the zoh_trigger topic
  RCLCPP_DEBUG(this->get_logger(), "Subscribing to the zoh_trigger topic.");
  _zohTriggerSubscription = this->create_subscription<ZOHTriggerMsg>(
    "zoh_trigger", qos, [this](ZOHTriggerMsg::UniquePtr msg) { this->onZohTriggerMsg(std::move(msg)); });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Subscribed to the " << _zohTriggerSubscription->get_topic_name() << " topic.");
}

void ControlInputsZOH::provideResetServer(const rclcpp::QoS & qos)
{
  auto const qosProfile = qos.get_rmw_qos_profile();

  RCLCPP_DEBUG(this->get_logger(), "Creating server for the control_zoh/reset service.");
  this->_resetServiceServer = this->create_service<EmptySrv>(
    "control_zoh/reset",
    [this](const EmptySrv::Request::SharedPtr req, EmptySrv::Response::SharedPtr res) {  // NOLINT
      (void)req;
      (void)res;
      RCLCPP_DEBUG(this->get_logger(), "Reset service invoked, calling the reset method.");
      this->onReset();
    },
    qosProfile);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Published server for the " << _resetServiceServer->get_service_name() << " service.");
}

void ControlInputsZOH::onControlInputsMsg(ControlInputsZOH::FloatArrayMsg::UniquePtr msg)
{
  auto const rxTimestamp = Clock::now();

  // Update the most recent control input values
  _mostRecentControlInputs = msg->data;

  // Publish the most recent control inputs values to the actuators
  publishMostRecentControlInputs(msg->pheader.step, msg->pheader.timestamp, rxTimestamp);
}

void ControlInputsZOH::onZohTriggerMsg(ControlInputsZOH::ZOHTriggerMsg::UniquePtr msg)
{
  auto const rxTimestamp = Clock::now();
  publishMostRecentControlInputs(msg->pheader.step, msg->pheader.timestamp, rxTimestamp);
}

void ControlInputsZOH::onReset()
{
  auto const rxTimestamp = Clock::now();

  // Reset the value of the most recent control inputs to the initial values
  _mostRecentControlInputs = _initialValues;

  // Publish the values to the actuators to ensure they are reset
  const rclcpp::Time simulationStartTime{0, 0, RCL_ROS_TIME};
  publishMostRecentControlInputs(0, simulationStartTime, rxTimestamp);
}

void ControlInputsZOH::publishMostRecentControlInputs(uint32_t step, const rclcpp::Time& simTime, const Clock::time_point& rxTimestamp) {
  auto publisherIt = _actuatorPublishers.cbegin();
  auto valueIt = _mostRecentControlInputs.cbegin();

  auto const publisherEnd = _actuatorPublishers.cend();
  auto const valuesEnd = _mostRecentControlInputs.cend();

  while(publisherIt != publisherEnd && valueIt != valuesEnd)
  {
    ControlInputsZOH::ActuatorMessage msg{};
    msg.pheader.step = step;
    msg.pheader.timestamp = simTime;
    msg.effort = *valueIt;

    msg.pheader.computation_time = rclcpp::Duration{Clock::now() - rxTimestamp};

    publisherIt->get()->publish(msg);

    ++publisherIt;
    ++valueIt;
  }
}

void ControlInputsZOH::readActuatorTopicNames()
{
  auto const ACTUATORS_PARAM_NAME = "actuators";

  try {
    this->declare_parameter<std::vector<std::string>>(ACTUATORS_PARAM_NAME);
    this->get_parameter(ACTUATORS_PARAM_NAME, _actuatorTopicNames);

    if (_actuatorTopicNames.empty()) {
      RCLCPP_FATAL_STREAM(
        this->get_logger(),
        "The actuators parameter was empty. Please provide a list of the actuators present in this "
        "control group instance.");
      throw std::runtime_error("An empty list was found in the actuator parameter list");
    }

  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL_STREAM(
      this->get_logger(),
      "An error occurred while trying to read the value of the actuators parameter.");
    RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "A ParameterTypeException was caught while trying to read the actuators parameter value. "
      "Exception message: "
        << e.what());
    std::terminate();
  }
}

void ControlInputsZOH::readInitialValues(std::size_t expectedSize)
{
  auto const INITIAL_VALUES_PARAM_NAME = "initial_value";

  try {
    _initialValues.reserve(expectedSize);

    this->declare_parameter<std::vector<double>>(INITIAL_VALUES_PARAM_NAME);
    this->get_parameter(INITIAL_VALUES_PARAM_NAME, _initialValues);

    if (_initialValues.size() != expectedSize) {
      RCLCPP_FATAL_STREAM(
        this->get_logger(),
        "The number of values present ("
          << _initialValues.size()
          << ") was different from the numbers of actuators present in the system (" << expectedSize
          << "). The " << INITIAL_VALUES_PARAM_NAME << " must contain exactly " << expectedSize
          << " values");

      throw std::runtime_error(
        std::string{"A initial values list with "} + std::to_string(expectedSize) +
        std::string{" values was expected, but only "} + std::to_string(_initialValues.size()) +
        std::string{" were present."});
    }
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_FATAL_STREAM(
      this->get_logger(), "An error occurred while trying to read the value of the "
                            << INITIAL_VALUES_PARAM_NAME
                            << " parameter. Please ensure that this parameter was correctly set.");
    RCLCPP_DEBUG_STREAM(
      this->get_logger(), "A ParameterTypeException was caught while trying to read the "
                            << INITIAL_VALUES_PARAM_NAME
                            << " parameter value with message: " << e.what());
    std::terminate();
  }
}

void ControlInputsZOH::advertiseActuatorTopics(const rclcpp::QoS & qos)
{
  _actuatorPublishers.reserve(_actuatorTopicNames.size());

  for (auto const & topicName : _actuatorTopicNames) {
    _actuatorPublishers.push_back(
      this->create_publisher<ControlInputsZOH::ActuatorMessage>(topicName, qos));
  }
}
