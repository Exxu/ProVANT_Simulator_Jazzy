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

class StepCommandBridgeTesterNode : public rclcpp::Node
{
public:
  StepCommandBridgeTesterNode()
  : Node("step_command_bridge_tester_node")
  {
    total_requests_ = this->declare_parameter<int>("total_requests", 5);
    request_period_ms_ = this->declare_parameter<int>("request_period_ms", 300);
    startup_wait_ms_ = this->declare_parameter<int>("startup_wait_ms", 1500);

    step_sub_ =
      this->create_subscription<provant_simulator_interfaces::msg::StepClock>(
        "/provant_simulator/step_clock",
        rclcpp::QoS(50),
        std::bind(&StepCommandBridgeTesterNode::onStepClock, this, std::placeholders::_1));

    step_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/provant_simulator/step", rclcpp::QoS(10));

    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "/provant_simulator/reset",
      std::bind(
        &StepCommandBridgeTesterNode::onReset,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    timer_ = this->create_wall_timer(
      50ms, std::bind(&StepCommandBridgeTesterNode::onTimer, this));

    start_time_ = this->now();
    phase_start_time_ = start_time_;

    RCLCPP_INFO(
      this->get_logger(),
      "Tester started. total_requests=%d request_period_ms=%d startup_wait_ms=%d",
      total_requests_, request_period_ms_, startup_wait_ms_);
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
    READY_TO_STEP,
    WAITING_STEP_RESULT,
    SUCCESS,
    FAILURE
  };

  void onReset(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    RCLCPP_INFO(this->get_logger(), "Reset service called during test.");
  }

  void onStepClock(const provant_simulator_interfaces::msg::StepClock::SharedPtr msg)
  {
    const auto step = msg->step;

    if (!has_last_step_) {
      has_last_step_ = true;
      last_step_ = step;
      RCLCPP_INFO(this->get_logger(), "Initial step_clock received: step=%u", step);
      return;
    }

    if (phase_ == Phase::WAITING_STEP_RESULT) {
      if (step <= expected_base_step_) {
        return;
      }

      const auto delta =
        static_cast<int64_t>(step) - static_cast<int64_t>(expected_base_step_);

      if (delta == 1) {
        ++successful_requests_;
        last_step_ = step;
        phase_ = Phase::READY_TO_STEP;
        phase_start_time_ = this->now();

        RCLCPP_INFO(
          this->get_logger(),
          "Step request %d/%d succeeded. Observed delta=1, base=%u new_step=%u",
          successful_requests_,
          total_requests_,
          expected_base_step_,
          step);

        if (successful_requests_ >= total_requests_) {
          phase_ = Phase::SUCCESS;
          done_ = true;
          exit_code_ = EXIT_SUCCESS;
          RCLCPP_INFO(this->get_logger(), "Test succeeded.");
          rclcpp::shutdown();
        }
        return;
      }

      RCLCPP_ERROR(
        this->get_logger(),
        "Expected exactly one world step, but observed delta=%ld (base=%u new_step=%u).",
        static_cast<long>(delta),
        expected_base_step_,
        step);
      fail();
      return;
    }

    last_step_ = step;
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

    if (phase_ == Phase::WAITING_FIRST_CLOCK) {
      if (has_last_step_ &&
          (now - start_time_).nanoseconds() / 1000000 >= startup_wait_ms_) {
        phase_ = Phase::READY_TO_STEP;
        phase_start_time_ = now;
        RCLCPP_INFO(
          this->get_logger(),
          "Starting step-by-step validation from current step=%u.",
          last_step_);
      }
      return;
    }

    if (phase_ == Phase::READY_TO_STEP) {
      if ((now - phase_start_time_).nanoseconds() / 1000000 >= request_period_ms_) {
        expected_base_step_ = last_step_;

        std_msgs::msg::Empty msg;
        step_pub_->publish(msg);

        phase_ = Phase::WAITING_STEP_RESULT;
        phase_start_time_ = now;
        ++sent_requests_;

        RCLCPP_INFO(
          this->get_logger(),
          "Published step request %d/%d with base step=%u.",
          sent_requests_,
          total_requests_,
          expected_base_step_);
      }
      return;
    }

    if (phase_ == Phase::WAITING_STEP_RESULT) {
      if ((now - phase_start_time_).seconds() > 2.0) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Timed out waiting for world step after request %d.",
          sent_requests_);
        fail();
      }
      return;
    }
  }

  void fail()
  {
    phase_ = Phase::FAILURE;
    done_ = true;
    exit_code_ = EXIT_FAILURE;
    rclcpp::shutdown();
  }

  rclcpp::Subscription<provant_simulator_interfaces::msg::StepClock>::SharedPtr step_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr step_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  rclcpp::Time phase_start_time_;

  Phase phase_{Phase::WAITING_FIRST_CLOCK};

  bool has_last_step_{false};
  bool done_{false};
  int exit_code_{EXIT_FAILURE};

  uint32_t last_step_{0};
  uint32_t expected_base_step_{0};

  int total_requests_{5};
  int request_period_ms_{300};
  int startup_wait_ms_{1500};

  int sent_requests_{0};
  int successful_requests_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StepCommandBridgeTesterNode>();
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