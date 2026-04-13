#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <provant_simulator_interfaces/msg/step_clock.hpp>

using namespace std::chrono_literals;

class ControlLoopTesterNode : public rclcpp::Node
{
public:
  ControlLoopTesterNode() : Node("controlloop_tester_node")
  {
    step_clock_topic_ = this->declare_parameter<std::string>(
      "step_clock_topic", "/test_group/step_clock");
    required_cycles_ = this->declare_parameter<int>("required_cycles", 5);
    startup_timeout_ms_ = this->declare_parameter<int>("startup_timeout_ms", 10000);
    cycle_timeout_ms_ = this->declare_parameter<int>("cycle_timeout_ms", 2000);

    step_clock_sub_ = this->create_subscription<provant_simulator_interfaces::msg::StepClock>(
      step_clock_topic_, rclcpp::QoS(100),
      std::bind(&ControlLoopTesterNode::onStepClock, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(20ms, std::bind(&ControlLoopTesterNode::onTimer, this));

    start_time_ = this->now();
    last_cycle_time_ = start_time_;

    RCLCPP_INFO(
      this->get_logger(),
      "Control group free-run tester started. topic=%s required_cycles=%d",
      step_clock_topic_.c_str(), required_cycles_);
  }

  bool done() const { return done_; }
  int exitCode() const { return exit_code_; }

private:
  static int64_t toNanoseconds(const builtin_interfaces::msg::Time & t)
  {
    return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
  }

  void onStepClock(const provant_simulator_interfaces::msg::StepClock::SharedPtr msg)
  {
    const auto step = msg->step;
    const auto time_ns = toNanoseconds(msg->time);

    received_first_clock_ = true;

    if (!has_last_step_) {
      has_last_step_ = true;
      last_step_ = step;
      last_time_ns_ = time_ns;
      observed_cycles_ = 1;
      last_cycle_time_ = this->now();

      RCLCPP_INFO(
        this->get_logger(),
        "Cycle %d/%d observed on control_group_manager: step=%u time_ns=%ld",
        observed_cycles_, required_cycles_, last_step_, static_cast<long>(last_time_ns_));

      checkSuccess();
      return;
    }

    if (step == last_step_) {
      return;
    }

    if (step < last_step_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Step clock regressed on control_group_manager: previous_step=%u new_step=%u",
        last_step_, step);
      fail();
      return;
    }

    last_step_ = step;
    last_time_ns_ = time_ns;
    ++observed_cycles_;
    last_cycle_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Cycle %d/%d observed on control_group_manager: step=%u time_ns=%ld",
      observed_cycles_, required_cycles_, last_step_, static_cast<long>(last_time_ns_));

    checkSuccess();
  }

  void onTimer()
  {
    if (done_) {
      return;
    }

    const auto now = this->now();

    if (!received_first_clock_) {
      if ((now - start_time_).nanoseconds() / 1000000 >= startup_timeout_ms_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Timed out waiting for the first control_group_manager cycle on topic %s.",
          step_clock_topic_.c_str());
        fail();
      }
      return;
    }

    if ((now - last_cycle_time_).nanoseconds() / 1000000 >= cycle_timeout_ms_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for a new control_group_manager cycle after %d observed cycles.",
        observed_cycles_);
      fail();
    }
  }

  void checkSuccess()
  {
    if (observed_cycles_ < required_cycles_) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "control_group_manager completed %d free cycles successfully. OK.",
      observed_cycles_);

    done_ = true;
    exit_code_ = EXIT_SUCCESS;
    rclcpp::shutdown();
  }

  void fail()
  {
    done_ = true;
    exit_code_ = EXIT_FAILURE;
    rclcpp::shutdown();
  }

  rclcpp::Subscription<provant_simulator_interfaces::msg::StepClock>::SharedPtr step_clock_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  rclcpp::Time last_cycle_time_;

  bool done_{false};
  bool received_first_clock_{false};
  bool has_last_step_{false};

  int exit_code_{EXIT_FAILURE};
  int required_cycles_{5};
  int observed_cycles_{0};
  int startup_timeout_ms_{10000};
  int cycle_timeout_ms_{2000};

  uint32_t last_step_{0};
  int64_t last_time_ns_{0};

  std::string step_clock_topic_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlLoopTesterNode>();
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
