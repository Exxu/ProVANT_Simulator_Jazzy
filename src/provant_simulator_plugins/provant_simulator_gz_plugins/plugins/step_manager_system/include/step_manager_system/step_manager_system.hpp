#ifndef PROVANT_STEP_MANAGER_SYSTEM__STEP_MANAGER_SYSTEM_HPP_
#define PROVANT_STEP_MANAGER_SYSTEM__STEP_MANAGER_SYSTEM_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>

#include <rclcpp/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_srvs/srv/empty.hpp>

#include <provant_simulator_interfaces/msg/step_clock.hpp>

namespace provant::simulator
{

class StepManagerSystem final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemReset
{
public:
  StepManagerSystem();
  ~StepManagerSystem() override;

  void Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo& info,
    gz::sim::EntityComponentManager& ecm) override;

  void Reset(
    const gz::sim::UpdateInfo& info,
    gz::sim::EntityComponentManager& ecm) override;

private:
  void ensureRosIsInitialized();
  void startExecutorThread();
  void stopExecutorThread();
  void requestResetNotification();
  void sendResetNotification();

  void publishStepClock(const gz::sim::UpdateInfo& info);

  std::string getStringParam(
    const std::shared_ptr<const sdf::Element>& sdf,
    const std::string& tag,
    const std::string& defaultValue) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<provant_simulator_interfaces::msg::StepClock>::SharedPtr
    stepClockPublisher_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr resetClient_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executorThread_;

  std::string rosNodeName_{"provant_step_manager_system"};
  std::string stepClockTopic_{"/provant_simulator/step_clock"};
  std::string resetService_{"/provant_simulator/reset"};

  std::mutex mutex_;
  std::optional<uint64_t> lastPublishedIteration_;
  std::atomic<bool> resetPending_{false};
  std::atomic<bool> publishFirstClockAfterReset_{false};
  std::atomic<bool> running_{false};
};

}  // namespace provant::simulator

#endif  // PROVANT_STEP_MANAGER_SYSTEM__STEP_MANAGER_SYSTEM_HPP_