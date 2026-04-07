#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <provant_simulator_interfaces/msg/step_clock.hpp>

using namespace std::chrono_literals;

class StepManagerSystemTestNode : public rclcpp::Node
{
public:
  StepManagerSystemTestNode()
  : Node("test_step_manager_system_node")
  {
    step_clock_sub_ = this->create_subscription<provant_simulator_interfaces::msg::StepClock>(
      "/provant_simulator/step_clock",
      rclcpp::QoS(10),
      std::bind(&StepManagerSystemTestNode::onStepClock, this, std::placeholders::_1));

    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "/provant_simulator/reset",
      std::bind(
        &StepManagerSystemTestNode::onReset,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    timer_ = this->create_wall_timer(
      100ms, std::bind(&StepManagerSystemTestNode::onTimer, this));

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "StepManagerSystem test node started.");
  }

  int exitCode() const
  {
    return exit_code_;
  }

  bool done() const
  {
    return done_;
  }

private:
  static int64_t timeToNanoseconds(const builtin_interfaces::msg::Time& time_msg)
  {
    return static_cast<int64_t>(time_msg.sec) * 1000000000LL +
           static_cast<int64_t>(time_msg.nanosec);
  }

  void onStepClock(const provant_simulator_interfaces::msg::StepClock::SharedPtr msg)
  {
    saw_step_clock_ = true;
    last_step_ = msg->step;
    last_time_ns_ = timeToNanoseconds(msg->time);

    if (!saw_first_step_) {
      saw_first_step_ = true;
      first_step_ = msg->step;
      first_time_ns_ = last_time_ns_;
      RCLCPP_INFO(
        this->get_logger(),
        "First step_clock received: step=%u time_ns=%ld",
        msg->step,
        static_cast<long>(last_time_ns_));
      return;
    }

    if (waiting_post_reset_) {
      const bool step_went_back = msg->step < pre_reset_step_;
      const bool time_went_back = last_time_ns_ < pre_reset_time_ns_;

      if (step_went_back || time_went_back) {
        RCLCPP_INFO(
          this->get_logger(),
          "Success: post-reset step_clock observed. "
          "pre_reset_step=%u post_reset_step=%u pre_reset_time_ns=%ld post_reset_time_ns=%ld",
          pre_reset_step_,
          msg->step,
          static_cast<long>(pre_reset_time_ns_),
          static_cast<long>(last_time_ns_));

        done_ = true;
        exit_code_ = EXIT_SUCCESS;
        rclcpp::shutdown();
      }
    }
  }

  void onReset(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    saw_reset_service_call_ = true;
    waiting_post_reset_ = true;
    pre_reset_step_ = last_step_;
    pre_reset_time_ns_ = last_time_ns_;

    RCLCPP_INFO(
      this->get_logger(),
      "Reset service called. Captured pre-reset state: step=%u time_ns=%ld",
      pre_reset_step_,
      static_cast<long>(pre_reset_time_ns_));
  }

  void onTimer()
  {
    const auto elapsed = this->now() - start_time_;

    if (done_) {
      return;
    }

    if (elapsed.seconds() > 20.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timeout. saw_step_clock=%s saw_reset_service_call=%s waiting_post_reset=%s",
        saw_step_clock_ ? "true" : "false",
        saw_reset_service_call_ ? "true" : "false",
        waiting_post_reset_ ? "true" : "false");
      done_ = true;
      exit_code_ = EXIT_FAILURE;
      rclcpp::shutdown();
      return;
    }

    if (!saw_step_clock_ && elapsed.seconds() > 5.0) {
      RCLCPP_WARN(this->get_logger(), "Still waiting for first step_clock...");
    }

    if (saw_step_clock_ && !saw_reset_service_call_ && elapsed.seconds() > 8.0) {
      RCLCPP_WARN(this->get_logger(), "step_clock received, waiting for world reset...");
    }

    if (saw_reset_service_call_ && waiting_post_reset_ && elapsed.seconds() > 12.0) {
      RCLCPP_WARN(this->get_logger(), "Reset received, waiting for post-reset step_clock...");
    }
  }

  rclcpp::Subscription<provant_simulator_interfaces::msg::StepClock>::SharedPtr step_clock_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;

  bool saw_step_clock_{false};
  bool saw_first_step_{false};
  bool saw_reset_service_call_{false};
  bool waiting_post_reset_{false};
  bool done_{false};
  int exit_code_{EXIT_FAILURE};

  uint32_t first_step_{0};
  uint32_t last_step_{0};
  uint32_t pre_reset_step_{0};

  int64_t first_time_ns_{0};
  int64_t last_time_ns_{0};
  int64_t pre_reset_time_ns_{0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StepManagerSystemTestNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && !node->done()) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  const int code = node->exitCode();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return code;
}