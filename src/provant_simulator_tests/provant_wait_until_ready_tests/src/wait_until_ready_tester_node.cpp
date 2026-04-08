#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>

#include <provant_simulator_interfaces/msg/step_clock.hpp>

using namespace std::chrono_literals;

class WaitUntilReadyTesterNode : public rclcpp::Node
{
public:
  WaitUntilReadyTesterNode()
  : Node("wait_until_ready_tester_node")
  {
    startup_wait_ms_ = this->declare_parameter<int>("startup_wait_ms", 300);
    stall_window_ms_ = this->declare_parameter<int>("stall_window_ms", 500);
    step_timeout_ms_ = this->declare_parameter<int>("step_timeout_ms", 2000);

    step_clock_sub_ =
      this->create_subscription<provant_simulator_interfaces::msg::StepClock>(
        "/provant_simulator/step_clock",
        rclcpp::QoS(50),
        std::bind(&WaitUntilReadyTesterNode::onStepClock, this, std::placeholders::_1));

    ready_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/provant_simulator/ready", rclcpp::QoS(10));

    sim_reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "/provant_simulator/reset",
      std::bind(
        &WaitUntilReadyTesterNode::onSimulationReset,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    timer_ = this->create_wall_timer(
      50ms, std::bind(&WaitUntilReadyTesterNode::onTimer, this));

    start_time_ = this->now();
    phase_start_time_ = start_time_;
    last_step_change_time_ = start_time_;

    RCLCPP_INFO(
      this->get_logger(),
      "Tester started. startup_wait_ms=%d stall_window_ms=%d step_timeout_ms=%d",
      startup_wait_ms_, stall_window_ms_, step_timeout_ms_);
  }

  bool done() const
  {
    return done_;
  }

  int exitCode() const
  {
    return exit_code_;
  }

private:
  enum class Phase
  {
    WAITING_FIRST_CLOCK,
    WAITING_INITIAL_STALL,
    PUBLISHING_READY_1,
    WAITING_STEP_1,
    WAITING_STALL_AFTER_STEP_1,
    PUBLISHING_READY_2,
    WAITING_STEP_2,
    WAITING_STALL_AFTER_STEP_2,
    SUCCESS,
    FAILURE
  };

  void onSimulationReset(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    ++sim_reset_count_;
    RCLCPP_INFO(this->get_logger(), "Simulation reset service called.");
  }

  void onStepClock(const provant_simulator_interfaces::msg::StepClock::SharedPtr msg)
  {
    const auto step = msg->step;

    if (!has_step_clock_) {
      has_step_clock_ = true;
      last_step_ = step;
      stabilized_step_ = step;
      last_step_change_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Initial step_clock received: step=%u", step);
      return;
    }

    if (step != last_step_) {
      last_step_ = step;
      last_step_change_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Observed step_clock change: step=%u", step);
    }

    if (phase_ == Phase::WAITING_STEP_1 || phase_ == Phase::WAITING_STEP_2) {
      if (step <= expected_base_step_) {
        return;
      }

      const auto delta =
        static_cast<int64_t>(step) - static_cast<int64_t>(expected_base_step_);

      if (delta != 1) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Expected exactly one released step, but observed delta=%ld (base=%u new_step=%u).",
          static_cast<long>(delta),
          expected_base_step_,
          step);
        fail();
        return;
      }

      if (phase_ == Phase::WAITING_STEP_1) {
        RCLCPP_INFO(
          this->get_logger(),
          "Ready #1 released exactly one step: %u -> %u",
          expected_base_step_,
          step);
        phase_ = Phase::WAITING_STALL_AFTER_STEP_1;
        phase_start_time_ = this->now();
        stabilized_step_ = step;
        return;
      }

      if (phase_ == Phase::WAITING_STEP_2) {
        RCLCPP_INFO(
          this->get_logger(),
          "Ready #2 released exactly one step: %u -> %u",
          expected_base_step_,
          step);
        phase_ = Phase::WAITING_STALL_AFTER_STEP_2;
        phase_start_time_ = this->now();
        stabilized_step_ = step;
      }
    }
  }

  void onTimer()
  {
    if (done_) {
      return;
    }

    const auto now = this->now();

    if ((now - start_time_).seconds() > 30.0) {
      RCLCPP_ERROR(this->get_logger(), "Global timeout.");
      fail();
      return;
    }

    switch (phase_) {
      case Phase::WAITING_FIRST_CLOCK:
        handleWaitingFirstClock(now);
        return;
      case Phase::WAITING_INITIAL_STALL:
        handleWaitingForStall(now, Phase::PUBLISHING_READY_1, "Initial barrier stall confirmed.");
        return;
      case Phase::PUBLISHING_READY_1:
        handlePublishingReady(1, Phase::WAITING_STEP_1);
        return;
      case Phase::WAITING_STEP_1:
        handleWaitingStep(now, 1);
        return;
      case Phase::WAITING_STALL_AFTER_STEP_1:
        handleWaitingForStall(now, Phase::PUBLISHING_READY_2, "Barrier stalled again after first released step.");
        return;
      case Phase::PUBLISHING_READY_2:
        handlePublishingReady(2, Phase::WAITING_STEP_2);
        return;
      case Phase::WAITING_STEP_2:
        handleWaitingStep(now, 2);
        return;
      case Phase::WAITING_STALL_AFTER_STEP_2:
        handleFinalStall(now);
        return;
      case Phase::SUCCESS:
      case Phase::FAILURE:
        return;
    }
  }

  void handleWaitingFirstClock(const rclcpp::Time & now)
  {
    if (!has_step_clock_) {
      return;
    }

    if ((now - start_time_).nanoseconds() / 1000000 >= startup_wait_ms_) {
      phase_ = Phase::WAITING_INITIAL_STALL;
      phase_start_time_ = now;
      stabilized_step_ = last_step_;

      RCLCPP_INFO(
        this->get_logger(),
        "Starting stall detection after startup from step=%u.",
        stabilized_step_);
    }
  }

  void handleWaitingForStall(
    const rclcpp::Time & now,
    Phase nextPhase,
    const char * successMessage)
  {
    if ((now - last_step_change_time_).nanoseconds() / 1000000 >= stall_window_ms_) {
      stabilized_step_ = last_step_;

      RCLCPP_INFO(
        this->get_logger(),
        "%s step stabilized at %u for %d ms.",
        successMessage,
        stabilized_step_,
        stall_window_ms_);

      phase_ = nextPhase;
      phase_start_time_ = now;
      return;
    }

    if ((now - phase_start_time_).seconds() > 8.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for barrier stall. Last observed step=%u.",
        last_step_);
      fail();
    }
  }

  void handlePublishingReady(int readyIndex, Phase nextPhase)
  {
    expected_base_step_ = stabilized_step_;

    std_msgs::msg::Empty msg;
    ready_pub_->publish(msg);

    phase_ = nextPhase;
    phase_start_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Published /provant_simulator/ready #%d using stabilized base step=%u.",
      readyIndex,
      expected_base_step_);
  }

  void handleWaitingStep(const rclcpp::Time & now, int readyIndex)
  {
    if ((now - phase_start_time_).nanoseconds() / 1000000 >= step_timeout_ms_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for released step after ready #%d.",
        readyIndex);
      fail();
    }
  }

  void handleFinalStall(const rclcpp::Time & now)
  {
    if ((now - last_step_change_time_).nanoseconds() / 1000000 >= stall_window_ms_) {
      stabilized_step_ = last_step_;

      RCLCPP_INFO(
        this->get_logger(),
        "Final barrier stall confirmed. step stabilized at %u for %d ms.",
        stabilized_step_,
        stall_window_ms_);

      phase_ = Phase::SUCCESS;
      done_ = true;
      exit_code_ = EXIT_SUCCESS;
      RCLCPP_INFO(this->get_logger(), "Test succeeded.");
      rclcpp::shutdown();
      return;
    }

    if ((now - phase_start_time_).seconds() > 8.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for final barrier stall. Last observed step=%u.",
        last_step_);
      fail();
    }
  }

  void fail()
  {
    phase_ = Phase::FAILURE;
    done_ = true;
    exit_code_ = EXIT_FAILURE;
    rclcpp::shutdown();
  }

  rclcpp::Subscription<provant_simulator_interfaces::msg::StepClock>::SharedPtr step_clock_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ready_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr sim_reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  rclcpp::Time phase_start_time_;
  rclcpp::Time last_step_change_time_;

  Phase phase_{Phase::WAITING_FIRST_CLOCK};

  bool done_{false};
  bool has_step_clock_{false};

  int exit_code_{EXIT_FAILURE};

  uint32_t last_step_{0};
  uint32_t stabilized_step_{0};
  uint32_t expected_base_step_{0};

  int startup_wait_ms_{300};
  int stall_window_ms_{500};
  int step_timeout_ms_{2000};

  int sim_reset_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaitUntilReadyTesterNode>();
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