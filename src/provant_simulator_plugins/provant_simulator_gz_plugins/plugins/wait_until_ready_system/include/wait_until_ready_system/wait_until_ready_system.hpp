#ifndef PROVANT_WAIT_UNTIL_READY_SYSTEM__WAIT_UNTIL_READY_SYSTEM_HPP_
#define PROVANT_WAIT_UNTIL_READY_SYSTEM__WAIT_UNTIL_READY_SYSTEM_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/empty.hpp>

namespace provant::simulator
{

class WaitUntilReadySystem final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  WaitUntilReadySystem();
  ~WaitUntilReadySystem() override;

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override;

private:
  void onReadyMsg(const std_msgs::msg::Empty::SharedPtr msg);
  void parseSdf(const std::shared_ptr<const sdf::Element> & sdf);

  std::shared_ptr<rclcpp::Node> rosNode_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr readySub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread rosSpinThread_;

  std::string rosNodeName_{"provant_wait_until_ready_system"};
  std::string readyTopic_{"/provant_simulator/ready"};

  boost::interprocess::interprocess_semaphore stepPermit_;

  std::atomic<bool> configured_{false};
  std::atomic<bool> shutdownRequested_{false};

  // True when one ready token is pending and may be consumed by PreUpdate.
  std::atomic<bool> permitPending_{false};

  // True after PreUpdate consumes a permit and until PostUpdate closes the barrier again.
  std::atomic<bool> stepInProgress_{false};

  // Warmup phase to avoid blocking Gazebo internal startup too early.
  std::atomic<bool> barrierArmed_{false};
  std::atomic<uint64_t> postUpdateCount_{0};
  uint64_t warmupCycles_{10};

  std::mutex stateMutex_;
};

}  // namespace provant::simulator

#endif  // PROVANT_WAIT_UNTIL_READY_SYSTEM__WAIT_UNTIL_READY_SYSTEM_HPP_