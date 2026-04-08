#include "step_manager_system/step_manager_system.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <thread>
#include <utility>

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>

namespace provant::simulator
{

namespace
{
rclcpp::QoS makeReliableQoS(const std::size_t depth)
{
  rclcpp::QoS qos(depth);
  qos.keep_last(depth);
  qos.reliable();
  qos.durability_volatile();
  return qos;
}
}  // namespace

StepManagerSystem::StepManagerSystem() = default;

StepManagerSystem::~StepManagerSystem() { stopExecutorThread(); }

void StepManagerSystem::Configure(
  const gz::sim::Entity& /*entity*/,
  const std::shared_ptr<const sdf::Element>& sdf,
  gz::sim::EntityComponentManager& /*ecm*/,
  gz::sim::EventManager& /*eventMgr*/)
{
  this->rosNodeName_ =
    getStringParam(sdf, "ros_node_name", "provant_step_manager_system");
  this->stepClockTopic_ =
    getStringParam(sdf, "step_clock_topic", "/provant_simulator/step_clock");
  this->resetService_ =
    getStringParam(sdf, "reset_service", "/provant_simulator/reset");

  ensureRosIsInitialized();

  this->node_ = std::make_shared<rclcpp::Node>(this->rosNodeName_);
  this->executor_ =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->executor_->add_node(this->node_);

  auto qos = makeReliableQoS(10);

  this->stepClockPublisher_ =
    this->node_->create_publisher<provant_simulator_interfaces::msg::StepClock>(
      this->stepClockTopic_, qos);

  this->resetClient_ =
    this->node_->create_client<std_srvs::srv::Empty>(this->resetService_);

  startExecutorThread();

  RCLCPP_INFO(
    this->node_->get_logger(),
    "StepManagerSystem configured. step_clock_topic='%s', reset_service='%s'",
    this->stepClockTopic_.c_str(),
    this->resetService_.c_str());
}

void StepManagerSystem::PreUpdate(
  const gz::sim::UpdateInfo& info,
  gz::sim::EntityComponentManager& /*ecm*/)
{
  if (!this->node_ || !this->stepClockPublisher_) {
    return;
  }

  const bool forcePublishAfterReset =
    this->publishFirstClockAfterReset_.load();

  if (!forcePublishAfterReset && info.paused) {
    return;
  }

  const auto currentIteration = info.iterations;

  bool shouldPublish = false;

  {
    std::scoped_lock<std::mutex> lock(this->mutex_);

    if (forcePublishAfterReset) {
      shouldPublish = true;
      this->lastPublishedIteration_ = currentIteration;
      this->publishFirstClockAfterReset_.store(false);
    } else if (
      !this->lastPublishedIteration_.has_value() ||
      this->lastPublishedIteration_.value() != currentIteration) {
      shouldPublish = true;
      this->lastPublishedIteration_ = currentIteration;
    }
  }

  if (!shouldPublish) {
    return;
  }

  if (this->resetPending_.exchange(false)) {
    sendResetNotification();
  }

  publishStepClock(info);
}

void StepManagerSystem::Reset(
  const gz::sim::UpdateInfo& /*info*/,
  gz::sim::EntityComponentManager& /*ecm*/)
{
  {
    std::scoped_lock<std::mutex> lock(this->mutex_);
    this->lastPublishedIteration_.reset();
  }

  this->publishFirstClockAfterReset_.store(true);
  requestResetNotification();
}

void StepManagerSystem::ensureRosIsInitialized()
{
  if (rclcpp::ok()) {
    return;
  }

  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);
}

void StepManagerSystem::startExecutorThread()
{
  this->running_.store(true);
  this->executorThread_ = std::thread([this]() {
    while (this->running_.load()) {
      this->executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  });
}

void StepManagerSystem::stopExecutorThread()
{
  this->running_.store(false);

  if (this->executor_) {
    this->executor_->cancel();
  }

  if (this->executorThread_.joinable()) {
    this->executorThread_.join();
  }

  if (this->executor_ && this->node_) {
    this->executor_->remove_node(this->node_);
  }

  this->executor_.reset();
  this->node_.reset();
  this->stepClockPublisher_.reset();
  this->resetClient_.reset();
}

void StepManagerSystem::requestResetNotification()
{
  this->resetPending_.store(true);

  if (this->node_) {
    RCLCPP_INFO(
      this->node_->get_logger(),
      "World reset detected. Reset notification scheduled.");
  }
}

void StepManagerSystem::sendResetNotification()
{
  if (!this->resetClient_) {
    return;
  }

  if (!this->resetClient_->service_is_ready()) {
    RCLCPP_WARN(
      this->node_->get_logger(),
      "World reset detected, but service '%s' is not ready.",
      this->resetService_.c_str());
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future = this->resetClient_->async_send_request(
    request,
    [logger = this->node_->get_logger()](
      rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
      try {
        (void)future.get();
        RCLCPP_INFO(logger, "Reset notification sent successfully.");
      } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Reset notification failed: %s", e.what());
      }
    });

  (void)future;
}

void StepManagerSystem::publishStepClock(const gz::sim::UpdateInfo& info)
{
  provant_simulator_interfaces::msg::StepClock msg;
  msg.step = static_cast<uint32_t>(info.iterations);

  const auto simNs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();

  if (simNs < 0) {
    RCLCPP_WARN(
      this->node_->get_logger(),
      "Negative simTime received (%ld ns). Publishing zero time.",
      static_cast<long>(simNs));
    msg.time.sec = 0;
    msg.time.nanosec = 0;
  } else {
    const auto totalNs = static_cast<uint64_t>(simNs);
    msg.time.sec = static_cast<int32_t>(totalNs / 1000000000ULL);
    msg.time.nanosec =
      static_cast<uint32_t>(totalNs % 1000000000ULL);
  }

  this->stepClockPublisher_->publish(msg);

  RCLCPP_DEBUG(
    this->node_->get_logger(),
    "Published step_clock: step=%u time=(%d,%u)",
    msg.step,
    msg.time.sec,
    msg.time.nanosec);
}

std::string StepManagerSystem::getStringParam(
  const std::shared_ptr<const sdf::Element>& sdf,
  const std::string& tag,
  const std::string& defaultValue) const
{
  if (!sdf || !sdf->HasElement(tag)) {
    return defaultValue;
  }

  return sdf->Get<std::string>(tag);
}

}  // namespace provant::simulator

GZ_ADD_PLUGIN(
  provant::simulator::StepManagerSystem,
  gz::sim::System,
  provant::simulator::StepManagerSystem::ISystemConfigure,
  provant::simulator::StepManagerSystem::ISystemPreUpdate,
  provant::simulator::StepManagerSystem::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(
  provant::simulator::StepManagerSystem,
  "provant::simulator::StepManagerSystem")