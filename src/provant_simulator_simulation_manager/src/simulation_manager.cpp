// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file simulation_manager.cpp
/// @brief This file contains the implementation of the SimulationManager class.
///
/// @author Júnio Eduardo de Morais Aquino

#include <memory>
#include <provant_simulator_simulation_manager/simulation_manager.hpp>
#include <string>
#include <utility>

using provant::SimulationManager;

provant::SimulationManager::SimulationManager(const rclcpp::NodeOptions & opts)
: rclcpp::Node("simulation_manager", opts), _simulationState(SimulationState::paused)
{
  RCLCPP_INFO(this->get_logger(), "Initializing node.");

  RCLCPP_DEBUG(this->get_logger(), "Advertising the /provant_simulator/step topic.");
  this->_stepPublisher = this->create_publisher<StepMsg>("/provant_simulator/step", 10);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(), "Advertised the " << this->_stepPublisher->get_topic_name() << " topic.");

  RCLCPP_DEBUG(this->get_logger(), "Advertising the /provant_simulator/simulation_state topic.");
  this->_simulationStatePublisher =
    this->create_publisher<StrMsg>("/provant_simulator/simulation_state", 10);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Advertised the " << this->_simulationStatePublisher->get_topic_name() << " topic.");

  RCLCPP_DEBUG(
    this->get_logger(), "Creating a server for the /provant_simulator/register_group service.");
  this->_registerGroupServer = this->create_service<RegisterSrv>(
    "/provant_simulator/register_group",
    [this](
      const SimulationManager::RegisterSrv::Request::SharedPtr req,
      SimulationManager::RegisterSrv::Response::SharedPtr res) {
      this->onRegisterGroupCall(req, res);
    });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Created a server for the " << this->_registerGroupServer->get_service_name() << " service.");

  RCLCPP_DEBUG(this->get_logger(), "Creating a server for the /provant_simulator/reset service.");
  this->_resetServer = this->create_service<EmptySrv>(
    "/provant_simulator/reset",
    [this](const EmptySrv::Request::SharedPtr req, EmptySrv::Response::SharedPtr res) {
      (void)req;
      (void)res;
      this->reset();
    });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Created a server for the " << this->_resetServer->get_service_name() << " service.");

  RCLCPP_DEBUG(
    this->get_logger(), "Subscribing to the /provant_simulator/set_simulation_state topic.");
  this->_setSimulationStateSub = this->create_subscription<StrMsg>(
    "/provant_simulator/set_simulation_state", 10,
    [this](StrMsg msg) { this->onSetSimulationStateMsg(msg); });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Subscribed to the " << this->_setSimulationStateSub->get_topic_name() << " topic.");

  RCLCPP_INFO(this->get_logger(), "Node initialized successfully.");
}

bool provant::SimulationManager::isGroupRegistered(const std::string & group) const
{
  std::lock_guard lk{_readyMutex};
  return _readyState.find(group) != _readyState.end();
}

void SimulationManager::onRegisterGroupCall(
  const SimulationManager::RegisterSrv::Request::SharedPtr & request,
  SimulationManager::RegisterSrv::Response::SharedPtr & res)
{
  std::lock_guard lk{_readyMutex};

  auto const isRegistered = _readyState.find(request->group_namespace) != _readyState.end();
  if (isRegistered) {
    res->result = false;
    res->message = std::string{"A group is already registered with the \""} +
                   request->group_namespace + std::string{"\" namespace."};
    return;
  }

  // Add the group on the _readyState
  _readyState.emplace(std::make_pair(request->group_namespace, false));

  // Connect to the namespaced ready topic
  auto const topicName = request->group_namespace.back() == '/'
                           ? request->group_namespace + std::string{"ready"}
                           : request->group_namespace + std::string{"/ready"};
  ;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Subscribing to the " << topicName << " topic.");
  _readySubscriptions.push_back(this->create_subscription<EmptyMsg>(
    topicName, 10,
    [this, request](EmptyMsg msg) { this->setReady(request->group_namespace, msg); }));
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Advertised the " << _readySubscriptions.back()->get_topic_name() << " topic.");

  // Create a client for the namespaced reset client
  auto const serviceName = request->group_namespace.back() == '/'
                             ? request->group_namespace + std::string{"reset"}
                             : request->group_namespace + std::string{"/reset"};
  RCLCPP_DEBUG_STREAM(
    this->get_logger(), "Creating a client for the " << serviceName << " service.");
  _resetClients.push_back(this->create_client<EmptySrv>(serviceName));
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Created a client for the " << _resetClients.back()->get_service_name() << " service.");

  // Return a positive response
  res->result = true;
}

void provant::SimulationManager::setReady(
  const std::string & group_namespace, SimulationManager::EmptyMsg msg)
{
  (void)msg;
  std::lock_guard lk{_readyMutex};
  auto iter = _readyState.find(group_namespace);

  if (iter != _readyState.end()) {
    iter->second = true;
    stepIfRequired();
  } else {
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "Tried to update the state of the \""
                            << group_namespace << "\" group. But no such group is registered.");
  }
}

void provant::SimulationManager::stepIfRequired()
{
  // If any of the registered groups is not ready, returns.
  for (const auto & group : this->_readyState) {
    if (!group.second) {
      return;
    }
  }

  // If all groups are ready, check the simulation state
  if (
    this->_simulationState == SimulationState::running ||
    this->_simulationState == SimulationState::stepping) {
    // Set all groups to not ready
    for (auto & group : this->_readyState) {
      group.second = false;
    }

    // Update the simulation state
    if (this->_simulationState == SimulationState::stepping) {
      updateSimulationState(SimulationState::paused);
    } else {
      // Publish a step request
      StepMsg stepMsg{};
      this->_stepPublisher->publish(stepMsg);
    }
  }
}

/// @details Check if the new simulation state is different from the current one. And in this case,
/// updates the simulation state and publish a message on the /provant_simulator/simulation_state
/// topic notifying the change.
void provant::SimulationManager::updateSimulationState(provant::SimulationState state)
{
  if (_simulationState != state) {
    _simulationState = state;

    switch (_simulationState) {
      case SimulationState::paused:
        publishSimulationState("paused");
        break;
      case SimulationState::stepping:
        publishSimulationState("stepping");
        break;
      case SimulationState::running:
        publishSimulationState("running");
        break;
      case SimulationState::stopped:
        publishSimulationState("stopped");
        break;
      case SimulationState::resetting:
        publishSimulationState("resetting");
        break;
    }
  }
}

void provant::SimulationManager::publishSimulationState(const std::string & state)
{
  StrMsg msg{};
  msg.data = state;
  this->_simulationStatePublisher->publish(msg);
}

/// @details If the group is not registered, returns false.
bool provant::SimulationManager::isGroupReady(const std::string & group) const
{
  std::lock_guard lk{_readyMutex};
  auto const iter = _readyState.find(group);
  return iter != _readyState.end() && iter->second;
}

std::size_t SimulationManager::registeredGroupCount() const
{
  std::lock_guard lk{_readyMutex};
  return _readyState.size();
}

/// @details Check if the value contained in the message represents a valid simulation state, and
/// updates the simulation status.
/// If the simulation status changes, publish a message on the /provant_simulator/simulation_state
/// topic signaling the change to the new status.
/// If the message contains an invalid simulation state, emits a log message informing the user
/// about this error.
void provant::SimulationManager::onSetSimulationStateMsg(SimulationManager::StrMsg & msg)
{
  std::lock_guard lk{_readyMutex};

  if (msg.data == std::string{"start"}) {
    updateSimulationState(SimulationState::running);
  } else if (msg.data == std::string{"step"}) {
    updateSimulationState(SimulationState::stepping);
  } else if (msg.data == std::string{"pause"}) {
    updateSimulationState(SimulationState::paused);
  } else if (msg.data == std::string{"stop"}) {
    updateSimulationState(SimulationState::stopped);
  } else {
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "Received an invalid simulation state update request. The value \""
                            << msg.data << "\" is not a valid simulation state.");
  }
}

provant::SimulationState SimulationManager::simulationState() const
{
  std::lock_guard lk{_readyMutex};
  return _simulationState;
}

void provant::SimulationManager::reset()
{
  std::lock_guard lk{_readyMutex};

  // Set the simulation state to resetting
  updateSimulationState(SimulationState::resetting);

  // Set the status of all registered groups to not ready
  for (auto & group : _readyState) {
    group.second = false;
  }

  // Invoke the reset services for all registered groups
  using ServiceResponseFuture = rclcpp::Client<EmptySrv>::SharedFutureWithRequest;
  auto request = std::make_shared<EmptySrv::Request>();
  for (auto & client : _resetClients) {
    client->async_send_request(
      request, [logger = this->get_logger()](ServiceResponseFuture future) {
        // Do nothing, but clear the pending service response.
        (void)future.get();
      });
  }

  // Set the simulation state to paused
  updateSimulationState(SimulationState::paused);
}
