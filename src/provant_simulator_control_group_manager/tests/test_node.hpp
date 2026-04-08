// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file test_node.hpp
/// @brief This file contains a ROS2 node with the necessary topics and services for testing
/// of the Control Group Manager node.
///
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROL_GROUP_MANAGER__TESTS__TEST_NODE_HPP_
#define PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROL_GROUP_MANAGER__TESTS__TEST_NODE_HPP_

#include <atomic>
#include <cmath>
#include <mutex>
#include <provant_simulator_interfaces/msg/empty.hpp>
#include <provant_simulator_interfaces/msg/float64_array.hpp>
#include <provant_simulator_interfaces/msg/step_clock.hpp>
#include <provant_simulator_interfaces/srv/control_group_register.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <utility>
#include <vector>

namespace provant::testing
{
/// @brief Stub to simulate a ROS2 service server.
/// @tparam T Type of the service.
template <typename T = std_srvs::srv::Empty>
class ServiceServer
{
public:
  /// @brief Method called when a request is received.
  /// @details Stores the request and response messages, without modifying any fields in the
  /// response.
  /// @param req Request that originated the service call.
  /// @param res Result of service call.
  virtual void onRequest(
    const typename T::Request::SharedPtr req, typename T::Response::SharedPtr res)
  {
    std::lock_guard lk{_mutex};
    requests.push_back(*req);
    responses.push_back(*res);
  }

  /// @brief Requests messages received by server.
  std::vector<typename T::Request> requests;
  /// @brief Response messages sent by the server.
  std::vector<typename T::Response> responses;

private:
  /// @brief Mutex to protect access to the requests and responses members.
  std::mutex _mutex;
};

/// @brief Stub to simulate a subscription to a ROS2 topic.
/// @tparam T Type of the topic.
template <typename T>
class TopicSubscriber
{
public:
  /// @brief Method called when a message is received on the topic.
  /// @details Stores the message in the msgs member variable.
  /// @param msg Message received.
  void onMessage(T msg)
  {
    std::lock_guard lk{_mutex};
    msgs.emplace_back(msg);
  }

  /// @brief Messages received on the topic.
  std::vector<T> msgs;

private:
  /// @brief Mutex to protect access to the topic.
  std::mutex _mutex;
};

/// @brief A Test Node containing the facilities required to test the Control Group Manager node.
class ControlGroupManagerTestNode : public rclcpp::Node
{
public:
  /// @brief Construct a new test node.
  /// @param name Name of the node.
  /// @param ns Namespace of the test node. Should be equal to the control group manager node under
  /// test.
  /// @param opts Test node options.
  explicit ControlGroupManagerTestNode(
    const std::string & name = "test_node", const std::string & ns = "",
    const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{})
  : rclcpp::Node(name, ns, opts)
  {
    this->stepClockPub = this->create_publisher<StepClockMsg>("/provant_simulator/step_clock", 10);
    this->controlInputsPublisher = this->create_publisher<FloatArrayMsg>("control_inputs", 10);
    this->disturbancesPublisher = this->create_publisher<FloatArrayMsg>("disturbances", 10);

    this->stepClockSub = this->create_subscription<StepClockMsg>(
      "step_clock", 10, [this](StepClockMsg msg) { this->stepClockMsgs.onMessage(msg); });

    this->readySub = this->create_subscription<ReadyMsg>(
      "ready", 10, [this](ReadyMsg msg) { this->readyMsgs.onMessage(msg); });

    this->zohTriggerSub = this->create_subscription<EmptyMsg>(
      "zoh_trigger", 10, [this](EmptyMsg msg) { this->zohTriggerMsgs.onMessage(msg); });

    this->controllerResetServer = this->create_service<EmptySrv>(
      "controller/reset",
      [this](const EmptySrv::Request::SharedPtr req, EmptySrv::Response::SharedPtr res) {
        this->controllerReset.onRequest(req, std::move(res));
      });

    this->controlZohResetServer = this->create_service<EmptySrv>(
      "control_zoh/reset",
      [this](const EmptySrv::Request::SharedPtr req, EmptySrv::Response::SharedPtr res) {
        this->controlZohReset.onRequest(req, std::move(res));
      });

    this->refGenResetServer = this->create_service<EmptySrv>(
      "ref_gen/reset",
      [this](const EmptySrv::Request::SharedPtr req, EmptySrv::Response::SharedPtr res) {
        this->refGenReset.onRequest(req, std::move(res));
      });

    this->estimatorResetServer = this->create_service<EmptySrv>(
      "estimator/reset",
      [this](const EmptySrv::Request::SharedPtr req, EmptySrv::Response::SharedPtr res) {
        this->estimatorReset.onRequest(req, std::move(res));
      });

    this->distGenResetServer = this->create_service<EmptySrv>(
      "dist_gen/reset",
      [this](const EmptySrv::Request::SharedPtr req, EmptySrv::Response::SharedPtr res) {
        this->distGenReset.onRequest(req, std::move(res));
      });

    this->distZohResetServer = this->create_service<EmptySrv>(
      "dist_zoh/reset",
      [this](const EmptySrv::Request::SharedPtr req, EmptySrv::Response::SharedPtr res) {
        this->distZohReset.onRequest(req, std::move(res));
      });

    this->resetClient = this->create_client<EmptySrv>("reset");

    this->registrationServer = this->create_service<RegistrationSrv>(
      "/provant_simulator/register_group",
      [this](
        const RegistrationSrv::Request::SharedPtr req, RegistrationSrv::Response::SharedPtr res) {
        this->onRegistrationRequest(req, std::move(res));
      });
  }

  /// @brief Type of the step clock message.
  using StepClockMsg = provant_simulator_interfaces::msg::StepClock;

  /// @brief ROS publisher for the /provant_simulator/step_clock topic.
  rclcpp::Publisher<StepClockMsg>::SharedPtr stepClockPub;
  /// @brief Publish a message on the /provant_simulator/step_clock topic.
  /// @param step Current simulation step.
  /// @param time Current simulation time.
  void publishStepClock(uint32_t step, const rclcpp::Time & time)
  {
    StepClockMsg msg{};
    msg.step = step;
    msg.time = time;

    this->stepClockPub->publish(msg);
  }

  /// @brief Publish a message on the /provant_simulator/step_clock topic.
  /// @details The simulation time is calculated as the product of the current step and the step
  /// size.
  /// @param step Current simulation step.
  /// @param stepSize Time interval of one simulation step.
  void publishStepClock(uint32_t step, double stepSize = 0.001)
  {
    auto const currentTime = step * stepSize;

    double wholeSecs;
    auto const fracSecs = std::modf(currentTime, &wholeSecs);

    auto const secs = static_cast<int32_t>(wholeSecs);
    auto const nanosecs = static_cast<uint32_t>(fracSecs * 1e9);

    publishStepClock(step, rclcpp::Time{secs, nanosecs, RCL_ROS_TIME});
  }

  /// @brief Subscription to the namespace step_clock topic.
  rclcpp::Subscription<StepClockMsg>::SharedPtr stepClockSub;
  /// @brief Stores the messages received on the namespaced step_clock topic.
  TopicSubscriber<StepClockMsg> stepClockMsgs;

  /// @brief Type used by the namespaced ready topic.
  using ReadyMsg = provant_simulator_interfaces::msg::Empty;
  /// @brief Subscription to the namespaced ready topic.
  rclcpp::Subscription<ReadyMsg>::SharedPtr readySub;
  /// @brief Stores the messages received on the namespaced ready topic.
  TopicSubscriber<ReadyMsg> readyMsgs;

  /// @brief Type of the messages used by the zoh_trigger topic.
  using EmptyMsg = std_msgs::msg::Empty;
  /// @brief Subscription to the namespaced zoh_trigger topic.
  rclcpp::Subscription<EmptyMsg>::SharedPtr zohTriggerSub;
  /// @brief Stores the messages received on the zoh_trigger topic.
  TopicSubscriber<EmptyMsg> zohTriggerMsgs;

  /// @brief Type of the messages used by the disturbances and control inputs topics.
  using FloatArrayMsg = provant_simulator_interfaces::msg::Float64Array;

  /// @brief Publisher for the control_inputs topic.
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr controlInputsPublisher;
  /// @brief Publish the vector of control inputs.
  /// @param ctrl Control inputs.
  /// @param step Current simulation step.
  /// @param time Current simulation time.
  void publishControlInputs(
    std::vector<double> ctrl, uint32_t step = 0,
    const rclcpp::Time & time = rclcpp::Time{0, 0, RCL_ROS_TIME})
  {
    FloatArrayMsg msg{};

    msg.pheader.step = step;
    msg.pheader.timestamp = time;
    msg.data = std::move(ctrl);

    this->controlInputsPublisher->publish(msg);
  }

  /// @brief Publisher for the disturbances topic.
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr disturbancesPublisher;
  /// @brief Publish the vector of external disturbances.
  /// @param dist Vector of external disturbances.
  /// @param step Current simulation step.
  /// @param time Current simulation time.
  void publishDisturbances(
    std::vector<double> dist, uint32_t step = 0,
    const rclcpp::Time & time = rclcpp::Time{0, 0, RCL_ROS_TIME})
  {
    FloatArrayMsg msg{};

    msg.pheader.step = step;
    msg.pheader.timestamp = time;
    msg.data = std::move(dist);

    this->disturbancesPublisher->publish(msg);
  }

  /// @brief Type used by the reset services.
  using EmptySrv = std_srvs::srv::Empty;
  /// @brief Client for the /provant_simulator/reset service.
  rclcpp::Client<EmptySrv>::SharedPtr resetClient;

  /// @brief Service to reset the controller.
  rclcpp::Service<EmptySrv>::SharedPtr controllerResetServer;
  /// @brief Simulates the controller reset service.
  ServiceServer<EmptySrv> controllerReset;

  /// @brief Service to reset the control inputs ZOH.
  rclcpp::Service<EmptySrv>::SharedPtr controlZohResetServer;
  /// @brief Simulate the control inputs ZOH reset service.
  ServiceServer<EmptySrv> controlZohReset;

  /// @brief Service to reset the reference generator.
  rclcpp::Service<EmptySrv>::SharedPtr refGenResetServer;
  /// @brief Simulate the reference generator reset service.
  ServiceServer<EmptySrv> refGenReset;

  /// @brief Service to reset the state estimator node.
  [[maybe_unused]] rclcpp::Service<EmptySrv>::SharedPtr estimatorResetServer;
  /// @brief Simulate the state estimator reset service.
  ServiceServer<EmptySrv> estimatorReset;

  /// @brief Service to reset the disturbance generator node.
  [[maybe_unused]] rclcpp::Service<EmptySrv>::SharedPtr distGenResetServer;
  /// @brief Simulate the disturbance generator reset service.
  ServiceServer<EmptySrv> distGenReset;

  /// @brief Service to reset the disturbances ZOH node.
  [[maybe_unused]] rclcpp::Service<EmptySrv>::SharedPtr distZohResetServer;
  /// @brief Simulate the disturbance ZOH reset service.
  ServiceServer<EmptySrv> distZohReset;

  /// @brief Type of the control group registration service.
  using RegistrationSrv = provant_simulator_interfaces::srv::ControlGroupRegister;
  /// @brief Server for the /provant_simulator/register_group service.
  rclcpp::Service<RegistrationSrv>::SharedPtr registrationServer;
  /// @brief Protects access to the registrationRequests and registrationResponses members.
  std::mutex registrationMutex;
  /// @brief The request messages received in the group registration service.
  std::vector<RegistrationSrv::Request> registrationRequests;
  /// @brief The response messages sent by the group registration service.
  std::vector<RegistrationSrv::Response> registrationResponses;

  /// @brief The result of the group registration call.
  bool registrationResult = true;
  /// @brief The result message of the registration call.
  std::string registrationMsg{};

  /// @brief Method called when the control group registration service is called.
  /// @details Simulate the registration of a control group and responds as defined by the
  /// registrationResult and registrationMsg members.
  /// @param req The request that originated the service call.
  /// @param res The service execution result.
  void onRegistrationRequest(
    const RegistrationSrv::Request::SharedPtr req, RegistrationSrv::Response::SharedPtr res)
  {
    std::lock_guard lk{registrationMutex};

    registrationRequests.push_back(*req);

    res->result = registrationResult;
    res->message = registrationMsg;
    registrationResponses.push_back(*res);
  }
};
}  // namespace provant::testing

#endif  // PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_ROS2__PROVANT_SIMULATOR_CONTROL_GROUP_MANAGER__TESTS__TEST_NODE_HPP_
