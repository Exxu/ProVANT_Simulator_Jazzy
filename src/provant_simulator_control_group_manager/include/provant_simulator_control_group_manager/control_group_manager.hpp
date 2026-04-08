// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file control_group_manager.hpp
/// @brief This file contains the declaration of the ControlGroupManager class.
///
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_CONTROL_GROUP_MANAGER__CONTROL_GROUP_MANAGER_HPP_
#define PROVANT_SIMULATOR_CONTROL_GROUP_MANAGER__CONTROL_GROUP_MANAGER_HPP_

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <provant_simulator_interfaces/msg/empty.hpp>
#include <provant_simulator_interfaces/msg/float64_array.hpp>
#include <provant_simulator_interfaces/msg/step_clock.hpp>
#include <provant_simulator_interfaces/srv/control_group_register.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <thread>

namespace provant
{
/// @brief The Control Group Manager is a ROS node that manages the execution of a control group.
/// @details The control group is composed at a minimum by a controller, a reference generator
/// and a Zero Order Hold (ZOH) for the control inputs, and optionally by a state estimator,
/// disturbance generator and disturbances zero order hold.
/// The control group manager connects to the global step_clock topic, and implements the necessary
/// logic to determine when a control group update is necessary given a desired control step, it
/// then publishes and subscribes to the appropriate topics necessary to trigger and monitor the
/// execution of the control group update.
/// When the control update is finished, the control group manager will notify the simulation
/// manager that is update is complete, and in its point of view the simulation is ready to proceed.
class ControlGroupManager : public rclcpp::Node
{
public:
  /// @brief Initializes a Control Group Manager Node.
  /// @param opts Node options to pass to rclcpp::Node.
  explicit ControlGroupManager(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{});

  /// @brief Destructs the Control Group Manager Node.
  ~ControlGroupManager() override;

  /// @brief Indicates if the current control group a disturbance generator node.
  /// @return True if the current control group has a disturbance generator and false otherwise.
  bool hasDisturbanceGenerator() const;
  /// @brief Indicates if the current control group has a state estimator node.
  /// @return True if the current control group has a state estimator and false otherwise.
  bool hasStateEstimator() const;
  /// @brief The control execution step.
  /// @return The time interval between executions of a control group update.
  const rclcpp::Duration & controlStep() const;

  /// @brief Indicates if the control inputs have already been published in the current control
  /// group update.
  /// @return True if the controller update is complete and false otherwise.
  bool controlReady() const;
  /// @brief Indicates if the disturbances have already been published in the current control
  /// group update.
  /// @return True if the disturbance generator update is complete and false otherwise.
  bool disturbancesReady() const;
  /// @brief  Indicates the timestamp in which the last control group update was executed.
  /// @return The timestamp in which the last control group update was executed.
  std::optional<rclcpp::Time> lastExecutedOn() const;

private:
  /// @brief Type of the message used to publish and subscribe to the step_clock topics.
  using StepClockMsg = provant_simulator_interfaces::msg::StepClock;
  /// @brief Type of the message used for zoh_trigger topic.
  using EmptyMsg = provant_simulator_interfaces::msg::Empty;
  /// @brief Type of the message used for the ready topic.
  using ReadyMsg = provant_simulator_interfaces::msg::Empty;
  /// @brief Type of the message used in the disturbances and control inputs topics.
  using FloatArrayMsg = provant_simulator_interfaces::msg::Float64Array;
  /// @brief Type of the message used on the reset servers.
  using EmptySrv = std_srvs::srv::Empty;
  /// @brief Type fo the service messages used to register the control group.
  using RegistrationSrv = provant_simulator_interfaces::srv::ControlGroupRegister;
  /// @brief std::chrono clock used to measure the control loop update time.
  using Clock = std::chrono::high_resolution_clock;

  /// @brief Duration of a control step.
  rclcpp::Duration _controlStep{0, 0};
  /// @brief Indicates if the current control group has a disturbance generator node.
  bool _hasDisturbances = false;
  /// @brief Indicates if the current control group has a state estimator node.
  bool _hasEstimator = false;

  /// @brief Timestamp storing the last time that this control group was updated.
  /// @details Must be empty on the first execution or after a reset.
  std::optional<rclcpp::Time> _lastExecutedOn;

  /// @brief Mutex to protected the _disturbancesReady and _controlReady member variables.
  mutable std::mutex _updateMutex;
  /// @brief Indicates if the disturbances have already been computed in the current controller
  /// step.
  bool _disturbancesReady = false;
  /// @brief Indicates if the control inputs have already been computed in the current controller
  /// step.
  bool _controlReady = false;

  /// @brief Subscription to the /provant_simulator/step_clock topic.
  rclcpp::Subscription<StepClockMsg>::SharedPtr _stepClockSubscription;
  /// @brief Subscription to the control_inputs topic.
  rclcpp::Subscription<FloatArrayMsg>::SharedPtr _controlInputsSubscription;
  /// @brief Subscription to the disturbances topic.
  rclcpp::Subscription<FloatArrayMsg>::SharedPtr _disturbancesSubscription;

  /// @brief Publisher for the step_clock topic.
  rclcpp::Publisher<StepClockMsg>::SharedPtr _stepClockPublisher;
  /// @brief Publisher for the ready topic.
  rclcpp::Publisher<ReadyMsg>::SharedPtr _readyPublisher;
  /// @brief Publisher for the zoh_trigger topic.
  rclcpp::Publisher<EmptyMsg>::SharedPtr _zohTriggerPublisher;

  /// @brief Control Group reset service server.
  /// @details When called, the reset service will reset an entire control group.
  rclcpp::Service<EmptySrv>::SharedPtr _resetServer;
  /// @brief Controller node reset service client.
  rclcpp::Client<EmptySrv>::SharedPtr _controllerResetClient;
  /// @brief Reference generator reset service client.
  rclcpp::Client<EmptySrv>::SharedPtr _refGenResetClient;
  /// @brief State estimator reset service client.
  /// @details Will be a nullptr if the control group does not have a state estimator.
  rclcpp::Client<EmptySrv>::SharedPtr _stateEstimatorResetClient;
  /// @brief Control inputs ZOH reset service client.
  rclcpp::Client<EmptySrv>::SharedPtr _controlZohResetClient;
  /// @brief Disturbances ZOH reset service client.
  /// @details Will be a nullptr if the control group does not have a disturbance generator.
  rclcpp::Client<EmptySrv>::SharedPtr _distZohResetClient;
  /// @brief Disturbance generator reset service client.
  /// @details Will be a nullptr if the control group does not have a disturbance generator.
  rclcpp::Client<EmptySrv>::SharedPtr _distGenResetClient;
  /// @brief Control group registration service.
  rclcpp::Client<RegistrationSrv>::SharedPtr _registrationClient;

  /// @brief Indicates if the execution of the reset thread should be cancelled.
  std::atomic<bool> _cancelExecution = false;
  /// @brief Thread used to complete the reset process.
  std::thread _innerResetThread;
  /// @brief Thread used to register the execution of the control group with the simulation manager.
  std::thread _registrationThread;

  /// @brief Declare the ROS parameters of the Control Group Manager node.
  void declareParameters();
  /// @brief Read the value of the ROS parameters of the Control Group Manager node.
  void readParameters();
  /// @brief Subscribe to the topics used by the Control Group Manager node.
  /// @param qos Quality of service settings for the topics.
  void createSubscriptions(const rclcpp::QoS & qos);
  /// @brief Advertise the topics used by the Control Group Manager node.
  /// @param qos Quality of service settings for the topics.
  void cratePublishers(const rclcpp::QoS & qos);
  /// @brief Create clients for the ROS services used by the Control Group Manager node.
  /// @param qos Quality of service settings for the service clients.
  void createServiceClients(const rclcpp::QoS & qos);
  /// @brief Create the ROS services servers of the Control Group Manager node.
  /// @param qos Quality of service settings for the services.
  void createServiceServers(const rclcpp::QoS & qos);
  /// @brief Register the control group with the simulation manager.
  void registerControlGroup();

  /// @brief Publish a message on the ready topic indicating that the control update is complete.
  /// @param start Timestamp indicating when the control loop update started. Used to obtain the
  /// control group update computation time.
  /// @param step The current simulation step.
  /// @param timestamp The current simulation time.
  void publishReadyMessage(Clock::time_point start, uint32_t step, const rclcpp::Time & timestamp);

  /// @brief Update the control group computations when a message is received on the
  /// /provant_simulator/step_clock topic.
  /// @param msg The step clock message received.
  void onStepClockMsg(StepClockMsg::UniquePtr msg);
  /// @brief Update the control inputs state when a new message is received on the control_inputs
  /// topic.
  /// @param msg The control inputs message received.
  void onControlInputsMsg(FloatArrayMsg::UniquePtr msg);
  /// @brief Update the disturbances state when a new message is received on the disturbances topic.
  void onDisturbancesMsg(FloatArrayMsg::UniquePtr msg);
  /// @brief Reset the internal state of the control group manager and call the reset services of
  /// the remaining control group nodes.
  void onReset();
};
}  // namespace provant

#endif  // PROVANT_SIMULATOR_CONTROL_GROUP_MANAGER__CONTROL_GROUP_MANAGER_HPP_
