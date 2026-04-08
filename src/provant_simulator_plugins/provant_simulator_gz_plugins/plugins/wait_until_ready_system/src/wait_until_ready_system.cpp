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

  if (executor_) {
    executor_->cancel();
  }

  if (rosSpinThread_.joinable()) {
    rosSpinThread_.join();
  }

  if (executor_ && rosNode_) {
    executor_->remove_node(rosNode_);
  }

  readySub_.reset();
  rosNode_.reset();
  executor_.reset();
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

  if (sdf->HasElement("warmup_cycles")) {
    warmupCycles_ = sdf->Get<uint64_t>("warmup_cycles");
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

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(rosNode_);

  rosSpinThread_ = std::thread([this]() {
    executor_->spin();
  });

  configured_.store(true);

  RCLCPP_INFO(
    rosNode_->get_logger(),
    "WaitUntilReadySystem configured. ready_topic='%s', warmup_cycles=%lu",
    readyTopic_.c_str(),
    static_cast<unsigned long>(warmupCycles_));
}

void WaitUntilReadySystem::onReadyMsg(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  if (shutdownRequested_.load()) {
    return;
  }

  if (!barrierArmed_.load()) {
    RCLCPP_DEBUG(
      rosNode_->get_logger(),
      "Ready received during warmup. Ignoring because barrier is not armed yet.");
    return;
  }

  std::lock_guard<std::mutex> lock(stateMutex_);

  // Do not accumulate multiple pending permissions.
  if (!permitPending_.load()) {
    permitPending_.store(true);
    stepPermit_.post();

    RCLCPP_INFO(
      rosNode_->get_logger(),
      "Ready received. Released one waiting simulation step.");
  } else {
    RCLCPP_DEBUG(
      rosNode_->get_logger(),
      "Ready received, but a permit is already pending. Ignoring duplicate permit.");
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
    return;
  }

  // During warmup, do not block the simulation.
  if (!barrierArmed_.load()) {
    return;
  }

  // If one step is already in progress, just let this frame continue.
  if (stepInProgress_.load()) {
    return;
  }

  while (!shutdownRequested_.load()) {
    {
      std::lock_guard<std::mutex> lock(stateMutex_);

      if (permitPending_.load()) {
        permitPending_.store(false);
        stepInProgress_.store(true);

        RCLCPP_DEBUG(
          rosNode_->get_logger(),
          "Consumed one permit in PreUpdate. Allowing exactly one simulation step.");
        return;
      }
    }

    stepPermit_.wait();

    if (shutdownRequested_.load() || info.paused) {
      return;
    }
  }
}

void WaitUntilReadySystem::PostUpdate(
  const gz::sim::UpdateInfo & info,
  const gz::sim::EntityComponentManager & /*ecm*/)
{
  if (!configured_.load()) {
    return;
  }

  if (shutdownRequested_.load() || info.paused) {
    return;
  }

  if (!barrierArmed_.load()) {
    const auto count = postUpdateCount_.fetch_add(1) + 1;

    if (count >= warmupCycles_) {
      barrierArmed_.store(true);

      RCLCPP_INFO(
        rosNode_->get_logger(),
        "Barrier armed after %lu PostUpdate cycles.",
        static_cast<unsigned long>(count));
    }

    return;
  }

  if (stepInProgress_.exchange(false)) {
    RCLCPP_DEBUG(
      rosNode_->get_logger(),
      "Completed one simulation step in PostUpdate. Barrier closed again.");
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