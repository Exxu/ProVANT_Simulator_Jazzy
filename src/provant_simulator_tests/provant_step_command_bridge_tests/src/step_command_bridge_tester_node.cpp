#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <thread>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>

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
    world_name_ = this->declare_parameter<std::string>("world_name", "step_command_bridge_test");
    control_timeout_ms_ = this->declare_parameter<int>("control_timeout_ms", 3000);
    stable_checks_required_ = this->declare_parameter<int>("stable_checks_required", 3);

    control_service_ = "/world/" + world_name_ + "/control";

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
      "Tester started. total_requests=%d request_period_ms=%d startup_wait_ms=%d world_name='%s'",
      total_requests_, request_period_ms_, startup_wait_ms_, world_name_.c_str());
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
    REQUESTING_INITIAL_PAUSE,
    WAITING_WORLD_TO_STABILIZE,
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
    last_step_ = msg->step;
    has_last_step_ = true;
    step_changed_since_last_check_ = true;

    if (!received_first_clock_) {
      received_first_clock_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial step_clock received: step=%u", last_step_);
      return;
    }

    if (phase_ == Phase::WAITING_STEP_RESULT) {
      if (last_step_ <= expected_base_step_) {
        return;
      }

      const auto delta =
        static_cast<int64_t>(last_step_) - static_cast<int64_t>(expected_base_step_);

      if (delta == 1) {
        ++successful_requests_;
        phase_ = Phase::READY_TO_STEP;
        phase_start_time_ = this->now();

        RCLCPP_INFO(
          this->get_logger(),
          "Step request %d/%d succeeded. Observed delta=1, base=%u new_step=%u",
          successful_requests_,
          total_requests_,
          expected_base_step_,
          last_step_);

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
        last_step_);
      fail();
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
      case Phase::REQUESTING_INITIAL_PAUSE:
        handleRequestingInitialPause(now);
        return;
      case Phase::WAITING_WORLD_TO_STABILIZE:
        handleWaitingWorldToStabilize(now);
        return;
      case Phase::READY_TO_STEP:
        handleReadyToStep(now);
        return;
      case Phase::WAITING_STEP_RESULT:
        handleWaitingStepResult(now);
        return;
      case Phase::SUCCESS:
      case Phase::FAILURE:
        return;
    }
  }

  void handleWaitingFirstClock(const rclcpp::Time& now)
  {
    if (received_first_clock_ &&
        (now - start_time_).nanoseconds() / 1000000 >= startup_wait_ms_) {
      phase_ = Phase::REQUESTING_INITIAL_PAUSE;
      phase_start_time_ = now;
    }
  }

  void handleRequestingInitialPause(const rclcpp::Time& now)
  {
    (void)now;

    if (!requestPause()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to request initial world pause.");
      fail();
      return;
    }

    stable_step_reference_ = last_step_;
    stable_checks_count_ = 0;
    step_changed_since_last_check_ = false;
    phase_ = Phase::WAITING_WORLD_TO_STABILIZE;
    phase_start_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Pause requested. Waiting for step stabilization from step=%u.",
      stable_step_reference_);
  }

  void handleWaitingWorldToStabilize(const rclcpp::Time& now)
  {
    if (step_changed_since_last_check_) {
      stable_step_reference_ = last_step_;
      stable_checks_count_ = 0;
      step_changed_since_last_check_ = false;
      return;
    }

    if (last_step_ == stable_step_reference_) {
      ++stable_checks_count_;
    } else {
      stable_step_reference_ = last_step_;
      stable_checks_count_ = 0;
      step_changed_since_last_check_ = false;
      return;
    }

    if (stable_checks_count_ >= stable_checks_required_) {
      phase_ = Phase::READY_TO_STEP;
      phase_start_time_ = now;
      RCLCPP_INFO(
        this->get_logger(),
        "World appears paused. Starting step-by-step validation from step=%u.",
        last_step_);
      return;
    }

    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for step stabilization after pause request.");
      fail();
    }
  }

  void handleReadyToStep(const rclcpp::Time& now)
  {
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
  }

  void handleWaitingStepResult(const rclcpp::Time& now)
  {
    if ((now - phase_start_time_).seconds() > 2.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for world step after request %d.",
        sent_requests_);
      fail();
    }
  }

  bool requestPause()
  {
    gz::msgs::WorldControl req;
    gz::msgs::Boolean rep;
    bool result = false;

    req.set_pause(true);

    const bool executed = gz_node_.Request(
      control_service_,
      req,
      static_cast<unsigned int>(control_timeout_ms_),
      rep,
      result);

    if (!executed) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to call Gazebo control service '%s'.",
        control_service_.c_str());
      return false;
    }

    if (!result || !rep.data()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Gazebo control service '%s' returned failure. transport_result=%s response=%s",
        control_service_.c_str(),
        result ? "true" : "false",
        rep.data() ? "true" : "false");
      return false;
    }

    return true;
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

  gz::transport::Node gz_node_;

  rclcpp::Time start_time_;
  rclcpp::Time phase_start_time_;

  Phase phase_{Phase::WAITING_FIRST_CLOCK};

  bool done_{false};
  bool has_last_step_{false};
  bool received_first_clock_{false};
  bool step_changed_since_last_check_{false};

  int exit_code_{EXIT_FAILURE};

  uint32_t last_step_{0};
  uint32_t expected_base_step_{0};
  uint32_t stable_step_reference_{0};

  int total_requests_{5};
  int request_period_ms_{300};
  int startup_wait_ms_{1500};
  int control_timeout_ms_{3000};
  int stable_checks_required_{3};

  int sent_requests_{0};
  int successful_requests_{0};
  int stable_checks_count_{0};

  std::string world_name_;
  std::string control_service_;
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