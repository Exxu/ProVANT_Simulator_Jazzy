// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file simulation_manager.hpp
/// @brief This file contains the declaration of the SimulationManager class.
///
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_SIMULATION_MANAGER__SIMULATION_MANAGER_HPP_
#define PROVANT_SIMULATOR_SIMULATION_MANAGER__SIMULATION_MANAGER_HPP_

#include <map>
#include <mutex>
#include <provant_simulator_interfaces/msg/empty.hpp>
#include <provant_simulator_interfaces/srv/control_group_register.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <vector>

namespace provant
{
/// @brief The valid simulation states.
enum class SimulationState : uint32_t { running, paused, stepping, stopped, resetting };

/// @brief The Simulation Manager is a ROS node that controls the simulation timing.
/// @details The control groups present in a simulation are expected to call the
/// /provant_simulator/register_group to register themselves with the simulation manager.
/// The simulation manager will them wait until every one of the nodes is ready before proceeding
/// with another simulation step.
///
/// The simulation manager also controls the simulation step, and notifies the remaining components
/// of changes in this status.
class SimulationManager : public rclcpp::Node
{
public:
  /// @brief Constructs a new instance of the Simulation Manager node.
  /// @param opts Options for the node creation.
  explicit SimulationManager(const rclcpp::NodeOptions & opts);

  /// @brief Get the number of currently registered control groups.
  /// @return Number of registered control groups.
  std::size_t registeredGroupCount() const;
  /// @brief Check if a group is ready.
  /// @param group Group namespace to check.
  /// @return True if the group is ready and false otherwise.
  bool isGroupReady(const std::string & group) const;
  /// @brief Check if a group is registered.
  /// @param group Name of the group.
  /// @return True if the group is registered and false otherwise.
  bool isGroupRegistered(const std::string & group) const;
  /// @brief Get the current simulation state.
  /// @return A value of the SimulationState enumeration indicating the current simulation state.
  SimulationState simulationState() const;

private:
  /// @brief Type of message used by the /provant_simulator/ready topic
  using EmptyMsg = provant_simulator_interfaces::msg::Empty;
  /// @brief Type of the service used by the group register service.
  using RegisterSrv = provant_simulator_interfaces::srv::ControlGroupRegister;
  /// @brief Type of the service used by the reset clients.
  using EmptySrv = std_srvs::srv::Empty;
  /// @brief Type of the message used by the /provant_simulator/step topic.
  using StepMsg = std_msgs::msg::Empty;
  /// @brief Type of the message used by the /provant_simulator/set_simulation_state topic.
  using StrMsg = std_msgs::msg::String;

  /// @brief A subscription to the /provant_simulator/set_simulation_state topic.
  rclcpp::Subscription<StrMsg>::SharedPtr _setSimulationStateSub;
  /// @brief A publisher for the /provant_simulator/simulation_state topic.
  rclcpp::Publisher<StrMsg>::SharedPtr _simulationStatePublisher;

  /// @brief Handles incoming messages on the /provant_simulator/set_simulation_state topic.
  /// @param msg The message received.
  void onSetSimulationStateMsg(StrMsg & msg);

  /// @brief ROS server for the /provant_simulator/register_group service.
  rclcpp::Service<RegisterSrv>::SharedPtr _registerGroupServer;

  /// @brief Mutex that protects the updates to the internal node state.
  mutable std::mutex _readyMutex;
  /// @brief A map storing the status of each of the registered control groups.
  /// @details If the value associated with a group name is false, the group is considered not ready
  /// and ready otherwise.
  std::map<std::string, bool> _readyState;
  /// @brief A list of connections to the ready topic of the registered control groups.
  std::vector<rclcpp::Subscription<EmptyMsg>::SharedPtr> _readySubscriptions;
  /// @brief A list of clients for the reset service of the registered control groups.
  std::vector<rclcpp::Client<EmptySrv>::SharedPtr> _resetClients;
  /// @brief The current simulation state.
  SimulationState _simulationState;

  /// @brief A publisher for the /provant_simulator/step topic.
  rclcpp::Publisher<StepMsg>::SharedPtr _stepPublisher;

  /// @brief Register a given control group within the simulation manager.
  /// @param request Request containing the group name.
  /// @param response Result of the registration.
  void onRegisterGroupCall(
    const RegisterSrv::Request::SharedPtr & request, RegisterSrv::Response::SharedPtr & response);

  /// @brief Set the state of the provided group_namespace to ready.
  /// @param group_namespace Group to set the state.
  /// @param msg Message containing the step and simulation time in which the ready message
  /// originated.
  void setReady(const std::string & group_namespace, EmptyMsg msg);

  /// @brief Update the simulation state.
  /// @param state The new simulation state
  void updateSimulationState(SimulationState state);
  /// @brief Publish a message on the /provant_simulator/simulation_state topic notifying a
  /// change in status.
  /// @param state New simulation status.
  void publishSimulationState(const std::string & state);
  /// @brief Check if a step increment request is necessary and if it is publish a message on the
  /// /provant_simulator/step topic.
  void stepIfRequired();

  /// @brief A server for the /provant_simulator/reset service.
  /// @details When called this server resets the ProVANT Simulator components.
  rclcpp::Service<EmptySrv>::SharedPtr _resetServer;
  /// @brief Resets the internal state of the simulation manager, and call the reset service of
  /// the registered groups.
  void reset();
};
}  // namespace provant

#endif  // PROVANT_SIMULATOR_SIMULATION_MANAGER__SIMULATION_MANAGER_HPP_
