// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file control_group_manager.cpp
/// @brief This file contains the implementation of the control group manager class.
///
/// @author Júnio Eduardo de Morais Aquino

#include "provant_simulator_control_group_manager/control_group_manager.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

using provant::ControlGroupManager;

ControlGroupManager::ControlGroupManager(const rclcpp::NodeOptions & opts)
: rclcpp::Node("control_group_manager", opts)
{
  // Declare and read the parameters for the node.
  declareParameters();
  readParameters();

  // Define the QoS to use in subscriptions
  rclcpp::QoS qos{10};

  // Connect to the necessary ROS topics and services
  createSubscriptions(qos);
  createServiceClients(qos);

  cratePublishers(qos);
  createServiceServers(qos);

  // Register the control group with the simulation manager
  _registrationClient = this->create_client<RegistrationSrv>("/provant_simulator/register_group");
  _registrationThread = std::thread([this]() { this->registerControlGroup(); });
}

/// @details Registers the control_step, has_disturbances and has_estimator ROS parameters.
void ControlGroupManager::declareParameters()
{
  this->declare_parameter<double>("control_step");
  this->declare_parameter<bool>("has_disturbances", false);
  this->declare_parameter<bool>("has_estimator", false);
}

/// @details Read the values of the has_disturbances and has_estimator, and control_step ROS
/// parameters and convert them to the appropriate values.
void ControlGroupManager::readParameters()
{
  // Read and convert the control_step parameter to a duration in nanoseconds.
  double ctrlStep;
  this->get_parameter("control_step", ctrlStep);
  // Convert the control step to nanoseconds
  std::chrono::nanoseconds ctrlStepNsecs{
    static_cast<std::chrono::nanoseconds::rep>(ctrlStep * 1e9)};
  this->_controlStep = rclcpp::Duration{ctrlStepNsecs};

  this->get_parameter("has_disturbances", _hasDisturbances);
  this->get_parameter("has_estimator", _hasEstimator);
}

/// @details Check if the control group should be updated in the current simulation step, and signal
/// the reference generator, disturbance generator, controller and state estimator nodes to start
/// their computations.
/// If the control group will not be updated, signal the Zero Order Holds (ZOH) to publish their
/// latest readings and notify the simulation manager that this control group is ready.
void provant::ControlGroupManager::onStepClockMsg(StepClockMsg::UniquePtr msg)
{
  using clock = std::chrono::high_resolution_clock;
  auto const start = clock::now();

  // Check if the controller must be executed
  try {
    const rclcpp::Time now{msg->time, RCL_ROS_TIME};

    if (!this->_lastExecutedOn.has_value()) {
      this->_stepClockPublisher->publish(std::move(msg));
      this->_lastExecutedOn = now;
    } else {
      auto const diff = now - this->_lastExecutedOn.value();
      if (diff >= _controlStep) {
        this->_stepClockPublisher->publish(std::move(msg));
        this->_lastExecutedOn = now;
      } else {
        EmptyMsg triggerMsg{};

        triggerMsg.pheader.step = msg->step;
        triggerMsg.pheader.timestamp = msg->time;
        triggerMsg.pheader.computation_time = rclcpp::Duration{Clock::now() - start};

        this->_zohTriggerPublisher->publish(triggerMsg);

        publishReadyMessage(start, msg->step, msg->time);
      }
    }
  } catch (const std::overflow_error & e) {
    RCLCPP_FATAL_STREAM(
      this->get_logger(),
      "An overflow error with message \""
        << e.what()
        << "\" occurred while trying to determine if the control group should be updated.");
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(
      this->get_logger(),
      "An incompatible time source was obtained when trying to determine if the control group "
      "should be updated. Please verify if the use_sim_time ROS parameter is configured "
      "correctly. Exception message: \""
        << e.what() << "\".");
  }
}

/// @details When a new message on the control step topic is received, this method will check
/// if the control group has a disturbance generator node.
/// If it does, it will check if the disturbance update was already finished, and in such case will
/// publish a message on the ready topic notifying the simulation manager that this control group
/// update is ready.
/// If the disturbance update is not finished, it will set the control inputs to ready, so that
/// when a message is received on the disturbances topic, the ready message can be published.
/// If the control group does not have a disturbances generator, this method will always publish
/// a message on the ready topic, as the control inputs are the only thing that needs to be updated
/// on a given control step.
void ControlGroupManager::onControlInputsMsg(ControlGroupManager::FloatArrayMsg::UniquePtr msg)
{
  auto const start = Clock::now();
  if (_hasDisturbances) {
    std::lock_guard lk{_updateMutex};

    if (_disturbancesReady) {
      // If the disturbances have already been published, when the control inputs are received,
      // the control group is ready.
      publishReadyMessage(start, msg->pheader.step, msg->pheader.timestamp);
    } else {
      // If the control group has a disturbance generator, and the disturbances are not ready, the
      // control ready state should be updated.
      _controlReady = true;
    }
  } else {
    // If the control group has no disturbance generator, when the controller publishes the control
    // inputs, the control group is ready.
    publishReadyMessage(start, msg->pheader.step, msg->pheader.timestamp);
  }
}

/// @details When a disturbances message is received, this method will check if the control inputs
/// have already been published in the current control step.
/// If they have, the disturbances are the last thing necessary for the control group update to
/// be finished, and thus this method will publish a message on the ready topic, notifying the
/// simulation manager that the control group update is complete.
/// If they don't, this method will update the disturbances state to ready, so that when the
/// control inputs are published, it it possible to detect that the control group update is
/// complete.
void ControlGroupManager::onDisturbancesMsg(ControlGroupManager::FloatArrayMsg::UniquePtr msg)
{
  auto const start = Clock::now();
  std::lock_guard lk{_updateMutex};

  if (_controlReady) {
    // If the control inputs are ready, when the disturbances are published, the control group
    // is ready.
    publishReadyMessage(start, msg->pheader.step, msg->pheader.timestamp);
  } else {
    // If the control inputs are not ready, indicate that the disturbances are ready, and wait
    // for the control inputs.
    _disturbancesReady = true;
  }
}

/// @details Assemble a ready message with the given step and timestamp, and calculates the
/// computation time given the timestamp in which the control update started.
void ControlGroupManager::publishReadyMessage(
  ControlGroupManager::Clock::time_point start, uint32_t step, const rclcpp::Time & timestamp)
{
  _disturbancesReady = false;
  _controlReady = false;

  ReadyMsg readyMsg{};
  readyMsg.pheader.step = step;
  readyMsg.pheader.timestamp = timestamp;
  readyMsg.pheader.computation_time = rclcpp::Duration{Clock::now() - start};

  this->_readyPublisher->publish(readyMsg);
}

void provant::ControlGroupManager::createSubscriptions(const rclcpp::QoS & qos)
{
  RCLCPP_DEBUG(this->get_logger(), "Subscribing to the /provant_simulator/step_clock topic.");
  this->_stepClockSubscription = this->create_subscription<StepClockMsg>(
    "/provant_simulator/step_clock", qos,
    [this](StepClockMsg::UniquePtr msg) { this->onStepClockMsg(std::move(msg)); });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Subscribed to the " << this->_stepClockSubscription->get_topic_name() << " topic.");

  RCLCPP_DEBUG(this->get_logger(), "Subscribing to the control_inputs topic.");
  this->_controlInputsSubscription = this->create_subscription<FloatArrayMsg>(
    "control_inputs", qos,
    [this](FloatArrayMsg::UniquePtr msg) { this->onControlInputsMsg(std::move(msg)); });
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Subscribed to the " << this->_controlInputsSubscription->get_topic_name() << " topic.");

  if (this->_hasDisturbances) {
    RCLCPP_DEBUG(this->get_logger(), "Subscribing to the disturbances topic.");
    this->_disturbancesSubscription = this->create_subscription<FloatArrayMsg>(
      "disturbances", qos,
      [this](FloatArrayMsg::UniquePtr msg) { this->onDisturbancesMsg(std::move(msg)); });
    RCLCPP_DEBUG_STREAM(
      this->get_logger(), "Subscribed to the " << this->_disturbancesSubscription->get_topic_name()
                                               << "disturbances topic.");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "The control group doesn't have a disturbances generator.");
  }
}

void ControlGroupManager::cratePublishers(const rclcpp::QoS & qos)
{
  RCLCPP_DEBUG(this->get_logger(), "Advertising the ready topic.");
  this->_readyPublisher = this->create_publisher<ReadyMsg>("ready", qos);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(), "Advertised the " << this->_readyPublisher->get_topic_name() << " topic.");

  RCLCPP_DEBUG(this->get_logger(), "Advertising the step_clock topic.");
  this->_stepClockPublisher = this->create_publisher<StepClockMsg>("step_clock", qos);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Advertised the " << this->_stepClockPublisher->get_topic_name() << " topic.");

  RCLCPP_DEBUG(this->get_logger(), "Advertising the zoh_trigger topic.");
  this->_zohTriggerPublisher = this->create_publisher<EmptyMsg>("zoh_trigger", qos);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Advertised the " << this->_zohTriggerPublisher->get_topic_name() << " topic.");
}

void provant::ControlGroupManager::createServiceClients(const rclcpp::QoS & qos)
{
  auto const qosProfile = qos.get_rmw_qos_profile();

  RCLCPP_DEBUG(this->get_logger(), "Creating a client for the controller/reset service.");
  this->_controllerResetClient = this->create_client<EmptySrv>("controller/reset", qosProfile);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Created a client for the " << this->_controllerResetClient->get_service_name() << " service.");

  RCLCPP_DEBUG(this->get_logger(), "Creating a client for the ref_gen/reset service.");
  this->_refGenResetClient = this->create_client<EmptySrv>("ref_gen/reset", qosProfile);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Created a client for the " << this->_refGenResetClient->get_service_name() << " service.");

  RCLCPP_DEBUG(this->get_logger(), "Creating a client for the control_zoh/reset service.");
  this->_controlZohResetClient = this->create_client<EmptySrv>("control_zoh/reset", qosProfile);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Created a client for the " << this->_controlZohResetClient->get_service_name() << " service.");

  if (_hasDisturbances) {
    RCLCPP_DEBUG(this->get_logger(), "Creating a client for the dist_zoh/reset service.");
    this->_distZohResetClient = this->create_client<EmptySrv>("dist_zoh/reset", qosProfile);
    RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "Created a client for the " << this->_distZohResetClient->get_service_name() << " service.");

    RCLCPP_DEBUG(this->get_logger(), "Creating a client for the dist_gen/reset service.");
    this->_distGenResetClient = this->create_client<EmptySrv>("dist_gen/reset", qosProfile);
    RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "Created a client for the " << this->_distGenResetClient->get_service_name() << " service.");
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "The control group does not have a disturbance generator, therefore clients for the "
      "dist_gen/reset and dist_zoh/reset services will not be created.");
  }

  if (_hasEstimator) {
    RCLCPP_DEBUG(this->get_logger(), "Creating a client for the estimator/reset service.");
    this->_stateEstimatorResetClient = this->create_client<EmptySrv>("estimator/reset", qosProfile);
    RCLCPP_DEBUG_STREAM(
      this->get_logger(), "Created a client for the "
                            << this->_stateEstimatorResetClient->get_service_name() << " service.");
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "The control group does not have a state estimator, therefore a client for the "
      "estimator/reset service will not be created.");
  }
}

void ControlGroupManager::createServiceServers(const rclcpp::QoS & qos)
{
  auto const qosProfile = qos.get_rmw_qos_profile();

  RCLCPP_DEBUG(this->get_logger(), "Creating a server for the reset service.");
  this->_resetServer = this->create_service<EmptySrv>(
    "reset",
    [this](const EmptySrv::Request::SharedPtr req, EmptySrv::Response::SharedPtr res) {  // NOLINT
      (void)req;
      (void)res;
      this->onReset();
    },
    qosProfile);
  RCLCPP_DEBUG_STREAM(
    this->get_logger(), "Created a server for the " << this->_resetServer << " service.");
}

/// @details When the reset service is triggered, this method will trigger the reset service of
/// all other control group components, and wait for their completion.
/// This method will also reset the internal state of the control group manager, allowing the
/// simulation to proceed as if it was starting from the beginning.
void provant::ControlGroupManager::onReset()
{
  auto const req = std::make_shared<EmptySrv::Request>();

  using ReqType = decltype(this->_controllerResetClient->async_send_request(req));
  std::vector<ReqType> serviceCalls;
  serviceCalls.reserve(6);

  serviceCalls.push_back(this->_controllerResetClient->async_send_request(req));
  serviceCalls.push_back(this->_refGenResetClient->async_send_request(req));
  serviceCalls.push_back(this->_controlZohResetClient->async_send_request(req));

  if (_hasEstimator) {
    serviceCalls.push_back(this->_stateEstimatorResetClient->async_send_request(req));
  }
  if (_hasDisturbances) {
    serviceCalls.push_back(this->_distGenResetClient->async_send_request(req));
    serviceCalls.push_back(this->_distZohResetClient->async_send_request(req));
  }

  // Reset the internal state
  {
    std::lock_guard lk{_updateMutex};

    _controlReady = false;
    _disturbancesReady = false;
    _lastExecutedOn.reset();
  }

  // NOTE: If the call to the wait method of the service shared futures is made on the same thread
  // that executes the reset service callback, execution will hang (deadlock).
  // In order to avoid this error, the waiting is deferred to a thread.
  RCLCPP_DEBUG(this->get_logger(), "Deferring the wait for the reset services.");
  _innerResetThread = std::thread([this, serviceCalls = std::move(serviceCalls),
                                   logger = this->get_logger()]() {
    // Wait for all the control group services to finish
    for (auto & call : serviceCalls) {
      if (call.valid()) {
        using namespace std::chrono_literals;

        bool canceled = false;
        std::future_status status;

        do {
          status = call.wait_for(100ms);
          if (status == std::future_status::timeout) {
            if (!rclcpp::ok() || _cancelExecution) {
              canceled = true;
              break;
            }
          }
        } while (status != std::future_status::ready);

        if (status == std::future_status::deferred) {
          RCLCPP_WARN(  // NOLINT(bugprone-lambda-function-name)
            logger,
            "The result of a call to a reset service shared future reported a deferred execution. "
            "This service will not be executed.");
        }

        if (canceled) {
          break;
        }
      } else {
        RCLCPP_ERROR(  // NOLINT
          this->get_logger(),
          "An error occurred while waiting for a call to one of the control group reset services.");
      }
    }
  });
}

bool provant::ControlGroupManager::hasDisturbanceGenerator() const { return _hasDisturbances; }

bool provant::ControlGroupManager::hasStateEstimator() const { return _hasEstimator; }

const rclcpp::Duration & provant::ControlGroupManager::controlStep() const { return _controlStep; }

bool provant::ControlGroupManager::controlReady() const
{
  std::lock_guard lk{_updateMutex};
  return _controlReady;
}

/// @details If the current control group doesn't have a disturbance generator node, this method
/// will always return true.
bool provant::ControlGroupManager::disturbancesReady() const
{
  if (_hasDisturbances) {
    std::lock_guard lk{_updateMutex};
    return _disturbancesReady;
  } else {
    return true;
  }
}

std::optional<rclcpp::Time> provant::ControlGroupManager::lastExecutedOn() const
{
  return _lastExecutedOn;
}

ControlGroupManager::~ControlGroupManager()
{
  _cancelExecution = true;
  if (_innerResetThread.joinable()) {
    _innerResetThread.join();
  }
  if (_registrationThread.joinable()) {
    _registrationThread.join();
  }
}

void ControlGroupManager::registerControlGroup()
{
  using namespace std::chrono_literals;

  // Wait for the service to become available
  // Wait for a total of 60 seconds, but check every 100ms if it should keep waiting, or if the
  // node is closing.
  for (int i = 0; i < 600; i++) {
    if (!this->_registrationClient->wait_for_service(100ms)) {
      if (!rclcpp::ok() || _cancelExecution) {
        break;
      }
    } else {
      break;
    }
  }

  if (!this->_registrationClient->service_is_ready()) {
    RCLCPP_FATAL(
      this->get_logger(),
      "The control group registration service is not available. And thus the control group could "
      "not be registered with the simulation, and will terminated its execution.");
    rclcpp::shutdown(
      nullptr, "The control group registration service is not available after waiting for 60s.");
    std::terminate();
  }

  // Register the node
  auto request = std::make_shared<RegistrationSrv::Request>();
  request->group_namespace = this->get_namespace();

  auto call = this->_registrationClient->async_send_request(request);
  if (call.valid()) {
    for (int i = 0; i < 600; i++) {
      bool cancel = false;
      auto const status = call.wait_for(100ms);
      switch (status) {
        case std::future_status::timeout: {
          // Verify if it is possible to keep waiting
          if (!rclcpp::ok() || _cancelExecution) {
            RCLCPP_WARN(
              this->get_logger(),
              "The call to the control group registration service was aborted because the node is "
              "being destroyed or because rclcpp was shutdown.");
            cancel = true;
          }
          break;
        }
        case std::future_status::deferred: {
          RCLCPP_FATAL(
            this->get_logger(),
            "The call to the control group registration service resulted in a deferred shared "
            "future. This is not valid, and execution will terminate.");
          std::terminate();
        }
        case std::future_status::ready: {
          RCLCPP_INFO(this->get_logger(), "The control group was successfully registered.");

          // TODO(jeduardo): Get the result and verify that the group was registered.

          break;
        }
      }

      if (status != std::future_status::timeout || cancel) {
        break;
      }
    }
  } else {
    RCLCPP_FATAL(
      this->get_logger(),
      "The call to the control group registration service resulted in an invalid shared future.");
    std::terminate();
  }
}
