// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file controller.cpp
/// @brief This file contains the implementation of the Controller class.
///
/// @author Júnio Eduardo de Morais Aquino

#include "provant_simulator_controller/controller.hpp"

#include <memory>
#include <utility>

provant::Controller::Controller(
  const rclcpp::NodeOptions & opts, std::unique_ptr<IController> controlStrategy)
: rclcpp::Node("controller", opts), _controlStrategy(std::move(controlStrategy))
{
  RCLCPP_INFO(this->get_logger(), "Initializing the node.");

  RCLCPP_DEBUG(this->get_logger(), "Creating a server for the controller/reset service.");
  this->_resetServiceClient = this->create_service<EmptySrv>(
    "reset", [this](const EmptySrv::Request::SharedPtr req, EmptySrv::Response::SharedPtr res) {
      (void)req;
      (void)res;
      this->onResetCall();
    });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Created a server for the " << this->_resetServiceClient->get_service_name() << " service.");

  RCLCPP_DEBUG(this->get_logger(), "Advertising the control_inputs topic.");
  this->_controlInputsPub = this->create_publisher<FloatArrayMsg>("control_inputs", 10);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Advertised the " << this->_controlInputsPub->get_topic_name() << " topic.");

  RCLCPP_DEBUG(this->get_logger(), "Advertising the controller/references topic.");
  this->_referencesPublisher = this->create_publisher<FloatArrayMsg>("controller/references", 10);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Advertised the " << this->_referencesPublisher->get_topic_name() << " topic.");

  RCLCPP_DEBUG(this->get_logger(), "Advertising the controller/state_vector topic.");
  this->_stateVectorPublisher =
    this->create_publisher<FloatArrayMsg>("controller/state_vector", 10);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Advertised the " << this->_stateVectorPublisher->get_topic_name() << " topic.");

  RCLCPP_DEBUG(this->get_logger(), "Advertising the controller/error_vector topic.");
  this->_errorPublisher = this->create_publisher<FloatArrayMsg>("controller/error_vector", 10);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(), "Advertised the " << this->_errorPublisher->get_topic_name() << " topic.");

  RCLCPP_DEBUG(this->get_logger(), "Subscribing to the state_vector topic.");
  this->_stateVecSub = this->create_subscription<FloatArrayMsg>(
    "state_vector", 10,
    [this](FloatArrayMsg::UniquePtr msg) { this->onStateVectorMessage(std::move(msg)); });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(), "Subscribed to the " << this->_stateVecSub->get_topic_name() << " topic.");

  RCLCPP_DEBUG(this->get_logger(), "Subscribing to the references topic.");
  this->_referencesSub = this->create_subscription<FloatArrayMsg>(
    "references", 10,
    [this](FloatArrayMsg::UniquePtr msg) { this->onReferencesMessage(std::move(msg)); });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Subscribed to the " << this->_referencesSub->get_topic_name() << " topic.");

  RCLCPP_DEBUG(this->get_logger(), "Initializing control strategy.");
  auto const res = this->_controlStrategy->setUp();
  if (res) {
    RCLCPP_DEBUG(this->get_logger(), "Initialized control strategy.");
  } else {
    RCLCPP_FATAL(
      this->get_logger(), "An error occurred while trying to initialize the control strategy.");
    throw std::runtime_error("The control strategy config function returned false.");
  }

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Node " << this->get_fully_qualified_name() << " initialized successfully.");
}

bool provant::Controller::referencesReady() const
{
  std::lock_guard lk{_updateMutex};
  return _references.has_value();
}

bool provant::Controller::stateVectorReady() const
{
  std::lock_guard lk{_updateMutex};
  return _stateVector.has_value();
}

void provant::Controller::onStateVectorMessage(FloatArrayMsg::UniquePtr msg)
{
  std::lock_guard lk{_updateMutex};
  _stateVector = msg->data;
  _header = msg->pheader;
  checkControlLaw();
}

void provant::Controller::onReferencesMessage(FloatArrayMsg::UniquePtr msg)
{
  std::lock_guard lk{_updateMutex};
  _references = msg->data;
  checkControlLaw();
}

void provant::Controller::checkControlLaw()
{
  if (_stateVector.has_value() && _references.has_value()) {
    executeControlLaw();
  }
}

void provant::Controller::executeControlLaw()
{
  FloatArrayMsg msg{};
  auto const start = Clock::now();
  msg.data = this->_controlStrategy->compute(_stateVector.value(), _references.value());
  auto const end = Clock::now();

  msg.pheader.step = _header.value().step;
  msg.pheader.timestamp = _header.value().timestamp;
  msg.pheader.computation_time = rclcpp::Duration{end - start};

  _controlInputsPub->publish(msg);

  FloatArrayMsg logMsgs{};
  logMsgs.pheader = msg.pheader;

  // Publish references
  logMsgs.data = _controlStrategy->references();
  _referencesPublisher->publish(logMsgs);

  // Publish state vector
  logMsgs.data = _controlStrategy->stateVector();
  _stateVectorPublisher->publish(logMsgs);

  // Publish error
  logMsgs.data = _controlStrategy->errorVector();
  _errorPublisher->publish(logMsgs);

  // Reset the internal state
  _header.reset();
  _stateVector.reset();
  _references.reset();
}

void provant::Controller::onResetCall()
{
  _header.reset();
  _stateVector.reset();
  _references.reset();

  _controlStrategy->resetControlStrategy();
}
