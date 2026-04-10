#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <provant_simulator_interfaces/msg/step_clock.hpp>

using namespace std::chrono_literals;

class ControlLoopTesterNode : public rclcpp::Node
{
public:
  ControlLoopTesterNode() : Node("controlloop_tester_node")
  {
    startup_wait_ms_ = this->declare_parameter<int>("startup_wait_ms", 1500);
    world_name_ = this->declare_parameter<std::string>("world_name", "temporal_loop_test");
    control_timeout_ms_ = this->declare_parameter<int>("control_timeout_ms", 3000);
    stable_checks_required_ = this->declare_parameter<int>("stable_checks_required", 3);
    no_step_verification_ms_ = this->declare_parameter<int>("no_step_verification_ms", 2000);

    control_service_ = "/world/" + world_name_ + "/control";

    step_clock_sub_ = this->create_subscription<provant_simulator_interfaces::msg::StepClock>(
      "/provant_simulator/step_clock", rclcpp::QoS(100),
      std::bind(&ControlLoopTesterNode::onStepClock, this, std::placeholders::_1));

    simulation_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/provant_simulator/simulation_state", rclcpp::QoS(10),
      std::bind(&ControlLoopTesterNode::onSimulationState, this, std::placeholders::_1));

    set_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/provant_simulator/set_simulation_state", rclcpp::QoS(10));

    sim_reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "/provant_simulator/reset",
      [](const std::shared_ptr<std_srvs::srv::Empty::Request>,
         std::shared_ptr<std_srvs::srv::Empty::Response>) {});

    timer_ = this->create_wall_timer(20ms, std::bind(&ControlLoopTesterNode::onTimer, this));

    start_time_ = this->now();
    phase_start_time_ = start_time_;

    RCLCPP_INFO(this->get_logger(), "Control loop tester started.");
  }

  bool done() const { return done_; }
  int exitCode() const { return exit_code_; }

private:
  enum class Phase
  {
    WAITING_FIRST_CLOCK,
    REQUESTING_INITIAL_PAUSE,
    WAITING_WORLD_TO_STABILIZE,
    STARTING_SIMULATION,
    WAITING_RUNNING_STATE,
    VERIFYING_NO_NEW_STEP_CLOCK,
    SUCCESS,
    FAILURE
  };

  static int64_t toNanoseconds(const builtin_interfaces::msg::Time & t)
  {
    return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
  }

  void onStepClock(const provant_simulator_interfaces::msg::StepClock::SharedPtr msg)
  {
    const auto step = msg->step;
    const auto time_ns = toNanoseconds(msg->time);

    const uint32_t previous_step = last_step_;
    last_step_ = step;
    last_time_ns_ = time_ns;
    received_first_clock_ = true;
    step_changed_since_last_check_ = true;

    if (!printed_first_clock_) {
      printed_first_clock_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Initial step_clock received: step=%u time_ns=%ld",
        last_step_, static_cast<long>(last_time_ns_));
      return;
    }

    if (phase_ == Phase::VERIFYING_NO_NEW_STEP_CLOCK) {
      if (step > verification_reference_step_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "A new global step_clock arrived after pause confirmation and without bootstrap: "
          "reference_step=%u new_step=%u reference_time_ns=%ld new_time_ns=%ld",
          verification_reference_step_,
          step,
          static_cast<long>(verification_reference_time_ns_),
          static_cast<long>(time_ns));
        fail();
        return;
      }
    }

    (void)previous_step;
  }

  void onSimulationState(const std_msgs::msg::String::SharedPtr msg)
  {
    last_simulation_state_ = msg->data;
  }

  void onTimer()
  {
    if (done_) {
      return;
    }

    const auto now = this->now();

    if ((now - start_time_).seconds() > 50.0) {
      RCLCPP_ERROR(this->get_logger(), "Global timeout.");
      fail();
      return;
    }

    switch (phase_) {
      case Phase::WAITING_FIRST_CLOCK:
        handleWaitingFirstClock(now);
        return;
      case Phase::REQUESTING_INITIAL_PAUSE:
        handleRequestingInitialPause();
        return;
      case Phase::WAITING_WORLD_TO_STABILIZE:
        handleWaitingWorldToStabilize(now);
        return;
      case Phase::STARTING_SIMULATION:
        handleStartingSimulation();
        return;
      case Phase::WAITING_RUNNING_STATE:
        handleWaitingRunningState(now);
        return;
      case Phase::VERIFYING_NO_NEW_STEP_CLOCK:
        handleVerifyingNoNewStepClock(now);
        return;
      case Phase::SUCCESS:
      case Phase::FAILURE:
        return;
    }
  }

  void handleWaitingFirstClock(const rclcpp::Time & now)
  {
    if (received_first_clock_ &&
        (now - start_time_).nanoseconds() / 1000000 >= startup_wait_ms_) {
      phase_ = Phase::REQUESTING_INITIAL_PAUSE;
      phase_start_time_ = now;
    }
  }

  void handleRequestingInitialPause()
  {
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
      "Pause requested. Waiting for global step_clock stabilization. current_step=%u current_time_ns=%ld",
      last_step_,
      static_cast<long>(last_time_ns_));
  }

  void handleWaitingWorldToStabilize(const rclcpp::Time & now)
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
      return;
    }

    if (stable_checks_count_ >= stable_checks_required_) {
      phase_ = Phase::STARTING_SIMULATION;
      phase_start_time_ = now;
      RCLCPP_INFO(
        this->get_logger(),
        "World appears paused. Starting simulation_manager without bootstrap. stable_step=%u stable_time_ns=%ld",
        last_step_,
        static_cast<long>(last_time_ns_));
      return;
    }

    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for step stabilization after pause request.");
      fail();
    }
  }

  void handleStartingSimulation()
  {
    std_msgs::msg::String msg;
    msg.data = "start";
    set_state_pub_->publish(msg);

    phase_ = Phase::WAITING_RUNNING_STATE;
    phase_start_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Start command sent to simulation_manager. Waiting for running state.");
  }

  void handleWaitingRunningState(const rclcpp::Time & now)
  {
    if (last_simulation_state_ == "running") {
      verification_reference_step_ = last_step_;
      verification_reference_time_ns_ = last_time_ns_;

      phase_ = Phase::VERIFYING_NO_NEW_STEP_CLOCK;
      phase_start_time_ = now;

      RCLCPP_INFO(
        this->get_logger(),
        "simulation_manager is running. Bootstrap disabled. Verifying that no new global step_clock arrives. "
        "reference_step=%u reference_time_ns=%ld",
        verification_reference_step_,
        static_cast<long>(verification_reference_time_ns_));
      return;
    }

    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for running state.");
      fail();
    }
  }

  void handleVerifyingNoNewStepClock(const rclcpp::Time & now)
  {
    if ((now - phase_start_time_).nanoseconds() / 1000000 >= no_step_verification_ms_) {
      RCLCPP_INFO(
        this->get_logger(),
        "No new global step_clock observed after pause confirmation and without bootstrap. "
        "World remained paused. reference_step=%u reference_time_ns=%ld",
        verification_reference_step_,
        static_cast<long>(verification_reference_time_ns_));

      phase_ = Phase::SUCCESS;
      done_ = true;
      exit_code_ = EXIT_SUCCESS;
      rclcpp::shutdown();
    }
  }

  bool requestPause()
  {
    gz::msgs::WorldControl req;
    gz::msgs::Boolean rep;
    bool result = false;
    req.set_pause(true);
    const bool executed = gz_node_.Request(
      control_service_, req, static_cast<unsigned int>(control_timeout_ms_), rep, result);
    return executed && result && rep.data();
  }

  void fail()
  {
    phase_ = Phase::FAILURE;
    done_ = true;
    exit_code_ = EXIT_FAILURE;
    rclcpp::shutdown();
  }

  rclcpp::Subscription<provant_simulator_interfaces::msg::StepClock>::SharedPtr step_clock_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr simulation_state_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_state_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr sim_reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  gz::transport::Node gz_node_;

  rclcpp::Time start_time_;
  rclcpp::Time phase_start_time_;

  Phase phase_{Phase::WAITING_FIRST_CLOCK};

  bool done_{false};
  bool received_first_clock_{false};
  bool printed_first_clock_{false};
  bool step_changed_since_last_check_{false};

  int exit_code_{EXIT_FAILURE};

  uint32_t last_step_{0};
  uint32_t stable_step_reference_{0};
  uint32_t verification_reference_step_{0};

  int64_t last_time_ns_{0};
  int64_t verification_reference_time_ns_{0};

  int startup_wait_ms_{1500};
  int control_timeout_ms_{3000};
  int stable_checks_required_{3};
  int no_step_verification_ms_{2000};
  int stable_checks_count_{0};

  std::string world_name_;
  std::string control_service_;
  std::string last_simulation_state_;
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