// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

#include "wait_until_ready_system/wait_until_ready_system.hpp"

#include <utility>

#include <gz/plugin/Register.hh>

namespace provant::simulator
{

WaitUntilReadySystem::WaitUntilReadySystem()
: stepPermit_(0)
{
}

WaitUntilReadySystem::~WaitUntilReadySystem()
{
  shutdownRequested_.store(true);

  // Release any blocked wait during shutdown.
  stepPermit_.post();
}

void WaitUntilReadySystem::parseSdf(const std::shared_ptr<const sdf::Element> & sdf)
{
  if (!sdf) {
    return;
  }

  if (sdf->HasElement("ros_node_name")) {
    rosNodeName_ = sdf->Get<std::string>("ros_node_name");
  }

  if (sdf->HasElement("ready_topic")) {
    readyTopic_ = sdf->Get<std::string>("ready_topic");
  }
}

void WaitUntilReadySystem::spinSomeRos()
{
  if (rosNode_) {
    rclcpp::spin_some(rosNode_);
  }
}

void WaitUntilReadySystem::Configure(
  const gz::sim::Entity & /*entity*/,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & /*ecm*/,
  gz::sim::EventManager & /*eventMgr*/)
{
  parseSdf(sdf);

  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  rosNode_ = std::make_shared<rclcpp::Node>(rosNodeName_);

  readySub_ = rosNode_->create_subscription<std_msgs::msg::Empty>(
    readyTopic_,
    rclcpp::QoS(10),
    std::bind(&WaitUntilReadySystem::onReadyMsg, this, std::placeholders::_1));

  configured_.store(true);

  RCLCPP_INFO(
    rosNode_->get_logger(),
    "WaitUntilReadySystem configured. ready_topic='%s'",
    readyTopic_.c_str());
}

void WaitUntilReadySystem::onReadyMsg(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  if (shutdownRequested_.load()) {
    return;
  }

  // Do not accumulate multiple pending permissions.
  bool expected = false;
  if (permitPending_.compare_exchange_strong(expected, true)) {
    stepPermit_.post();

    if (rosNode_) {
      RCLCPP_DEBUG(
        rosNode_->get_logger(),
        "Ready received. Released one waiting simulation step.");
    }
  } else {
    if (rosNode_) {
      RCLCPP_DEBUG(
        rosNode_->get_logger(),
        "Ready received, but a permit is already pending. Ignoring duplicate permit.");
    }
  }
}

void WaitUntilReadySystem::PreUpdate(
  const gz::sim::UpdateInfo & info,
  gz::sim::EntityComponentManager & /*ecm*/)
{
  if (!configured_.load()) {
    return;
  }

  if (shutdownRequested_.load() || info.paused) {
    spinSomeRos();
    return;
  }

  spinSomeRos();

  // If a step is already in progress, there is nothing to do here.
  if (stepInProgress_.load()) {
    return;
  }

  // Barrier: wait until a permit exists.
  while (!shutdownRequested_.load()) {
    bool expected = true;
    if (permitPending_.compare_exchange_strong(expected, false)) {
      stepInProgress_.store(true);

      if (rosNode_) {
        RCLCPP_DEBUG(
          rosNode_->get_logger(),
          "Consumed one permit in PreUpdate. Allowing exactly one simulation step.");
      }
      return;
    }

    stepPermit_.wait();
    spinSomeRos();
  }
}

void WaitUntilReadySystem::PostUpdate(
  const gz::sim::UpdateInfo & info,
  const gz::sim::EntityComponentManager & /*ecm*/)
{
  if (!configured_.load()) {
    return;
  }

  spinSomeRos();

  if (shutdownRequested_.load() || info.paused) {
    return;
  }

  // If one step was allowed through PreUpdate, close the barrier again here.
  if (stepInProgress_.exchange(false)) {
    if (rosNode_) {
      RCLCPP_DEBUG(
        rosNode_->get_logger(),
        "Completed one simulation step in PostUpdate. Barrier closed again.");
    }
  }
}

}  // namespace provant::simulator

GZ_ADD_PLUGIN(
  provant::simulator::WaitUntilReadySystem,
  gz::sim::System,
  provant::simulator::WaitUntilReadySystem::ISystemConfigure,
  provant::simulator::WaitUntilReadySystem::ISystemPreUpdate,
  provant::simulator::WaitUntilReadySystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  provant::simulator::WaitUntilReadySystem,
  "provant::simulator::WaitUntilReadySystem")