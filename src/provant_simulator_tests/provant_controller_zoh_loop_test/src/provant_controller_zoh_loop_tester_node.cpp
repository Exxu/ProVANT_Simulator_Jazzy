#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <provant_simulator_interfaces/msg/actuator.hpp>
#include <provant_simulator_interfaces/msg/step_clock.hpp>

using namespace std::chrono_literals;

class ControllerZohTesterNode : public rclcpp::Node
{
public:
  using StepClockMsg = provant_simulator_interfaces::msg::StepClock;
  using ActuatorMsg = provant_simulator_interfaces::msg::Actuator;

  ControllerZohTesterNode()
  : rclcpp::Node("provant_controller_zoh_loop_tester_node")
  {
    startup_wait_ms_ = this->declare_parameter<int>("startup_wait_ms", 1500);
    startup_after_pause_ms_ = this->declare_parameter<int>("startup_after_pause_ms", 1500);
    no_step_verification_ms_ = this->declare_parameter<int>("no_step_verification_ms", 1000);
    control_timeout_ms_ = this->declare_parameter<int>("control_timeout_ms", 3000);
    required_actuator_msgs_ = this->declare_parameter<int>("required_actuator_msgs", 7);
    stable_checks_required_ = this->declare_parameter<int>("stable_checks_required", 3);
    control_period_steps_ = this->declare_parameter<int>("control_period_steps", 3);
    expected_step_dt_ns_ = this->declare_parameter<int64_t>("expected_step_dt_ns", 1000000LL);
    time_tolerance_ns_ = this->declare_parameter<int64_t>("time_tolerance_ns", 0LL);
    world_name_ = this->declare_parameter<std::string>("world_name", "temporal_loop_test");
    group_namespace_ = this->declare_parameter<std::string>("group_namespace", "/test_group");
    actuator_topic_ = this->declare_parameter<std::string>("actuator_topic", "actuator_1");

    control_service_ = "/world/" + world_name_ + "/control";

    global_step_clock_sub_ = this->create_subscription<StepClockMsg>(
      "/provant_simulator/step_clock",
      rclcpp::QoS(50),
      std::bind(&ControllerZohTesterNode::onGlobalStepClock, this, std::placeholders::_1));

    local_step_clock_sub_ = this->create_subscription<StepClockMsg>(
      normalizeTopic(group_namespace_, "/step_clock"),
      rclcpp::QoS(50),
      std::bind(&ControllerZohTesterNode::onLocalStepClock, this, std::placeholders::_1));

    actuator_sub_ = this->create_subscription<ActuatorMsg>(
      normalizeTopic(group_namespace_, "/" + actuator_topic_),
      rclcpp::QoS(50),
      std::bind(&ControllerZohTesterNode::onActuator, this, std::placeholders::_1));

    simulation_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/provant_simulator/simulation_state",
      rclcpp::QoS(10),
      std::bind(&ControllerZohTesterNode::onSimulationState, this, std::placeholders::_1));

    manager_step_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/provant_simulator/step",
      rclcpp::QoS(50),
      std::bind(&ControllerZohTesterNode::onManagerStep, this, std::placeholders::_1));

    set_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/provant_simulator/set_simulation_state", rclcpp::QoS(10));

    bootstrap_step_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/provant_simulator/step", rclcpp::QoS(10));

    timer_ = this->create_wall_timer(50ms, std::bind(&ControllerZohTesterNode::onTimer, this));

    start_time_ = this->now();
    phase_start_time_ = start_time_;
    last_progress_time_ = start_time_;

    RCLCPP_INFO(
      this->get_logger(),
      "Controller + ZOH tester started. world='%s' group='%s' actuator_topic='%s' required_actuator_msgs=%d control_period_steps=%d expected_step_dt_ns=%ld",
      world_name_.c_str(),
      group_namespace_.c_str(),
      actuator_topic_.c_str(),
      required_actuator_msgs_,
      control_period_steps_,
      static_cast<long>(expected_step_dt_ns_));
  }

  bool done() const { return done_; }
  int exitCode() const { return exit_code_; }

private:
  enum class Phase
  {
    WAITING_FIRST_CLOCK,
    REQUESTING_INITIAL_PAUSE,
    WAITING_WORLD_TO_STABILIZE,
    WAITING_COMPONENTS_TO_SETTLE,
    STARTING_SIMULATION,
    WAITING_RUNNING_STATE,
    VERIFYING_NO_NEW_STEP_CLOCK_AFTER_START,
    BOOTSTRAPPING_STEP,
    WAITING_FIRST_POST_BOOTSTRAP_CLOCK,
    VERIFYING,
    SUCCESS,
    FAILURE
  };

  static int64_t timeToNanoseconds(const builtin_interfaces::msg::Time & time)
  {
    return static_cast<int64_t>(time.sec) * 1000000000LL + static_cast<int64_t>(time.nanosec);
  }

  static std::string normalizeTopic(const std::string & prefix, const std::string & suffix)
  {
    if (prefix.empty() || prefix == "/") {
      return suffix;
    }

    if (prefix.back() == '/' && !suffix.empty() && suffix.front() == '/') {
      return prefix.substr(0, prefix.size() - 1) + suffix;
    }

    if (prefix.back() != '/' && !suffix.empty() && suffix.front() != '/') {
      return prefix + "/" + suffix;
    }

    return prefix + suffix;
  }

  void onGlobalStepClock(const StepClockMsg::SharedPtr msg)
  {
    const auto new_step = msg->step;
    const auto new_time_ns = timeToNanoseconds(msg->time);

    if (!printed_first_clock_) {
      printed_first_clock_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Initial global step_clock received: step=%u time_ns=%ld",
        new_step,
        static_cast<long>(new_time_ns));
    }

    received_first_clock_ = true;

    if (new_step != last_global_step_) {
      step_changed_since_last_check_ = true;
    }

    last_global_step_ = new_step;
    last_global_time_ns_ = new_time_ns;

    if (
      phase_ == Phase::VERIFYING_NO_NEW_STEP_CLOCK_AFTER_START &&
      new_step > verification_reference_step_)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "A new global step_clock arrived after pause confirmation and before bootstrap step. reference_step=%u new_step=%u reference_time_ns=%ld new_time_ns=%ld",
        verification_reference_step_,
        new_step,
        static_cast<long>(verification_reference_time_ns_),
        static_cast<long>(new_time_ns));
      fail();
      return;
    }
  }

  void onLocalStepClock(const StepClockMsg::SharedPtr msg)
  {
    if (!capture_post_bootstrap_) {
      return;
    }

    if (msg->step <= bootstrap_reference_step_) {
      return;
    }

    if (!local_control_steps_.empty()) {
      const auto previous_step = local_control_steps_.back();
      if (msg->step <= previous_step) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Local control step regressed or repeated. previous=%u new=%u",
          previous_step,
          msg->step);
        fail();
        return;
      }

      const auto delta = static_cast<int>(msg->step - previous_step);
      if (delta != control_period_steps_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unexpected control update spacing. previous=%u new=%u delta=%d expected=%d",
          previous_step,
          msg->step,
          delta,
          control_period_steps_);
        fail();
        return;
      }
    }

    local_control_steps_.push_back(msg->step);
    ++control_update_count_;
    last_progress_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Observed local control step=%u control_updates=%d phase=%s",
      msg->step,
      control_update_count_,
      phaseName(phase_));
  }

  void onActuator(const ActuatorMsg::SharedPtr msg)
  {
    if (!capture_post_bootstrap_) {
      return;
    }

    pending_actuator_msgs_.push_back(*msg);
  }

  void onSimulationState(const std_msgs::msg::String::SharedPtr msg)
  {
    last_simulation_state_ = msg->data;
    RCLCPP_INFO(
      this->get_logger(),
      "simulation_manager state='%s'",
      last_simulation_state_.c_str());
  }

  void onManagerStep(const std_msgs::msg::Empty::SharedPtr /*msg*/)
  {
    ++manager_step_count_;
    RCLCPP_INFO(
      this->get_logger(),
      "simulation_manager published /provant_simulator/step count=%d",
      manager_step_count_);
  }

  void onTimer()
  {
    if (done_) {
      return;
    }

    const auto now = this->now();

    if ((now - start_time_).seconds() > 60.0) {
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
      case Phase::WAITING_COMPONENTS_TO_SETTLE:
        handleWaitingComponentsToSettle(now);
        return;
      case Phase::STARTING_SIMULATION:
        handleStartingSimulation();
        return;
      case Phase::WAITING_RUNNING_STATE:
        handleWaitingRunningState(now);
        return;
      case Phase::VERIFYING_NO_NEW_STEP_CLOCK_AFTER_START:
        handleVerifyingNoNewStepClockAfterStart(now);
        return;
      case Phase::BOOTSTRAPPING_STEP:
        handleBootstrappingStep(now);
        return;
      case Phase::WAITING_FIRST_POST_BOOTSTRAP_CLOCK:
        handleWaitingFirstPostBootstrapClock(now);
        return;
      case Phase::VERIFYING:
        handleVerifying(now);
        return;
      case Phase::SUCCESS:
      case Phase::FAILURE:
        return;
    }
  }

  void handleWaitingFirstClock(const rclcpp::Time & now)
  {
    if (!received_first_clock_) {
      return;
    }

    if ((now - start_time_).nanoseconds() / 1000000 < startup_wait_ms_) {
      return;
    }

    phase_ = Phase::REQUESTING_INITIAL_PAUSE;
    phase_start_time_ = now;
  }

  void handleRequestingInitialPause()
  {
    if (!requestPause()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to request initial world pause.");
      fail();
      return;
    }

    stable_step_reference_ = last_global_step_;
    stable_checks_count_ = 0;
    step_changed_since_last_check_ = false;
    phase_ = Phase::WAITING_WORLD_TO_STABILIZE;
    phase_start_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Pause requested. Waiting for absence of new global step_clock messages. current_step=%u current_time_ns=%ld",
      last_global_step_,
      static_cast<long>(last_global_time_ns_));
  }

  void handleWaitingWorldToStabilize(const rclcpp::Time & now)
  {
    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for the world to stabilize after pause request.");
      fail();
      return;
    }

    if (step_changed_since_last_check_) {
      stable_step_reference_ = last_global_step_;
      stable_checks_count_ = 0;
      step_changed_since_last_check_ = false;
      return;
    }

    if (last_global_step_ == stable_step_reference_) {
      ++stable_checks_count_;
    } else {
      stable_step_reference_ = last_global_step_;
      stable_checks_count_ = 0;
      return;
    }

    if (stable_checks_count_ >= stable_checks_required_) {
      phase_ = Phase::WAITING_COMPONENTS_TO_SETTLE;
      phase_start_time_ = now;
      RCLCPP_INFO(
        this->get_logger(),
        "World appears paused. Waiting %d ms for control_group/controller/ZOH registration and connections.",
        startup_after_pause_ms_);
    }
  }

  void handleWaitingComponentsToSettle(const rclcpp::Time & now)
  {
    if ((now - phase_start_time_).nanoseconds() / 1000000 < startup_after_pause_ms_) {
      return;
    }

    phase_ = Phase::STARTING_SIMULATION;
    phase_start_time_ = now;
  }

  void handleStartingSimulation()
  {
    std_msgs::msg::String msg;
    msg.data = "start";
    set_state_pub_->publish(msg);

    phase_ = Phase::WAITING_RUNNING_STATE;
    phase_start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Requested simulation_manager state 'start'.");
  }

  void handleWaitingRunningState(const rclcpp::Time & now)
  {
    if (last_simulation_state_ == "running") {
      verification_reference_step_ = last_global_step_;
      verification_reference_time_ns_ = last_global_time_ns_;
      step_changed_since_last_check_ = false;

      phase_ = Phase::VERIFYING_NO_NEW_STEP_CLOCK_AFTER_START;
      phase_start_time_ = now;

      RCLCPP_INFO(
        this->get_logger(),
        "simulation_manager is running. Verifying that no new global step_clock arrives before bootstrap. reference_step=%u reference_time_ns=%ld",
        verification_reference_step_,
        static_cast<long>(verification_reference_time_ns_));
      return;
    }

    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for simulation_manager to enter running state.");
      fail();
    }
  }

  void handleVerifyingNoNewStepClockAfterStart(const rclcpp::Time & now)
  {
    if ((now - phase_start_time_).nanoseconds() / 1000000 < no_step_verification_ms_) {
      return;
    }

    phase_ = Phase::BOOTSTRAPPING_STEP;
    phase_start_time_ = now;

    RCLCPP_INFO(
      this->get_logger(),
      "No new global step_clock arrived after start. Sending one bootstrap /provant_simulator/step. reference_step=%u reference_time_ns=%ld",
      verification_reference_step_,
      static_cast<long>(verification_reference_time_ns_));
  }

  void handleBootstrappingStep(const rclcpp::Time & now)
  {
    std_msgs::msg::Empty msg;
    bootstrap_step_pub_->publish(msg);

    bootstrap_reference_step_ = verification_reference_step_;
    bootstrap_reference_time_ns_ = verification_reference_time_ns_;
    last_verified_actuator_step_ = bootstrap_reference_step_;
    last_verified_actuator_time_ns_ = bootstrap_reference_time_ns_;
    verification_baseline_step_ = bootstrap_reference_step_;
    verification_baseline_time_ns_ = bootstrap_reference_time_ns_;

    local_control_steps_.clear();
    pending_actuator_msgs_.clear();
    verified_actuator_msgs_ = 0;
    control_update_count_ = 0;
    observed_direct_control_publish_ = false;
    observed_zoh_hold_publish_ = false;
    capture_post_bootstrap_ = true;
    last_progress_time_ = now;

    phase_ = Phase::WAITING_FIRST_POST_BOOTSTRAP_CLOCK;
    phase_start_time_ = now;

    RCLCPP_INFO(
      this->get_logger(),
      "Bootstrap step sent. Waiting for first post-bootstrap global step_clock. bootstrap_reference_step=%u bootstrap_reference_time_ns=%ld",
      bootstrap_reference_step_,
      static_cast<long>(bootstrap_reference_time_ns_));
  }

  void handleWaitingFirstPostBootstrapClock(const rclcpp::Time & now)
  {
    if (last_global_step_ > bootstrap_reference_step_) {
      phase_ = Phase::VERIFYING;
      phase_start_time_ = now;
      last_progress_time_ = now;

      RCLCPP_INFO(
        this->get_logger(),
        "First post-bootstrap global step_clock observed. global_step=%u global_time_ns=%ld. Starting controller + ZOH verification.",
        last_global_step_,
        static_cast<long>(last_global_time_ns_));
      return;
    }

    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for the first global step_clock after bootstrap step.");
      fail();
    }
  }

  void handleVerifying(const rclcpp::Time & now)
  {
    processPendingActuatorMessages();

    if (done_) {
      return;
    }

    if ((now - last_progress_time_).seconds() > 5.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for controller/ZOH progress. verified_actuator_msgs=%d control_updates=%d pending_actuator_msgs=%zu",
        verified_actuator_msgs_,
        control_update_count_,
        pending_actuator_msgs_.size());
      fail();
      return;
    }

    if (
      verified_actuator_msgs_ >= required_actuator_msgs_ &&
      observed_direct_control_publish_ &&
      observed_zoh_hold_publish_ &&
      control_update_count_ >= 2)
    {
      done_ = true;
      exit_code_ = EXIT_SUCCESS;
      phase_ = Phase::SUCCESS;
      RCLCPP_INFO(
        this->get_logger(),
        "provant_controller_zoh_loop_test succeeded. verified_actuator_msgs=%d control_updates=%d manager_step_count=%d",
        verified_actuator_msgs_,
        control_update_count_,
        manager_step_count_);
      rclcpp::shutdown();
    }
  }

  void processPendingActuatorMessages()
  {
    while (!pending_actuator_msgs_.empty()) {
      const auto msg = pending_actuator_msgs_.front();
      const auto step = msg.pheader.step;
      const auto time_ns = timeToNanoseconds(msg.pheader.timestamp);

      if (step <= verification_baseline_step_) {
        pending_actuator_msgs_.pop_front();
        continue;
      }

      const auto it = std::upper_bound(
        local_control_steps_.cbegin(),
        local_control_steps_.cend(),
        step);

      if (it == local_control_steps_.cbegin()) {
        return;
      }

      const auto control_step = *std::prev(it);
      const auto expected_effort = static_cast<double>(control_step);

      if (step != last_verified_actuator_step_ + 1) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Actuator step sequence mismatch. previous=%u current=%u",
          last_verified_actuator_step_,
          step);
        fail();
        return;
      }

      const auto observed_dt_ns = time_ns - last_verified_actuator_time_ns_;
      const auto time_error_ns = observed_dt_ns - expected_step_dt_ns_;
      const auto abs_time_error_ns = time_error_ns >= 0 ? time_error_ns : -time_error_ns;
      if (abs_time_error_ns > time_tolerance_ns_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unexpected actuator timestamp increment. previous_time_ns=%ld current_time_ns=%ld observed_dt_ns=%ld expected_dt_ns=%ld tolerance_ns=%ld",
          static_cast<long>(last_verified_actuator_time_ns_),
          static_cast<long>(time_ns),
          static_cast<long>(observed_dt_ns),
          static_cast<long>(expected_step_dt_ns_),
          static_cast<long>(time_tolerance_ns_));
        fail();
        return;
      }

      if (std::abs(msg.effort - expected_effort) > 1e-9) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unexpected actuator effort on step=%u. observed=%f expected=%f (held from control_step=%u)",
          step,
          msg.effort,
          expected_effort,
          control_step);
        fail();
        return;
      }

      if (control_step == step) {
        observed_direct_control_publish_ = true;
      } else {
        observed_zoh_hold_publish_ = true;
      }

      ++verified_actuator_msgs_;
      last_verified_actuator_step_ = step;
      last_verified_actuator_time_ns_ = time_ns;
      last_progress_time_ = this->now();

      RCLCPP_INFO(
        this->get_logger(),
        "Validated actuator msg %d/%d step=%u time_ns=%ld effort=%f source=%s held_control_step=%u",
        verified_actuator_msgs_,
        required_actuator_msgs_,
        step,
        static_cast<long>(time_ns),
        msg.effort,
        control_step == step ? "controller" : "zoh",
        control_step);

      pending_actuator_msgs_.pop_front();
    }
  }

  bool requestPause()
  {
    gz::msgs::WorldControl request;
    request.set_pause(true);

    gz::msgs::Boolean response;
    bool result = false;

    const bool executed = gz_node_.Request(
      control_service_,
      request,
      control_timeout_ms_,
      response,
      result);

    if (!executed) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to call Gazebo control service '%s'.",
        control_service_.c_str());
      return false;
    }

    if (!result || !response.data()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Gazebo control service '%s' rejected the pause request. transport_result=%s response=%s",
        control_service_.c_str(),
        result ? "true" : "false",
        response.data() ? "true" : "false");
      return false;
    }

    return true;
  }

  void fail()
  {
    done_ = true;
    exit_code_ = EXIT_FAILURE;
    phase_ = Phase::FAILURE;
    rclcpp::shutdown();
  }

  const char * phaseName(Phase phase) const
  {
    switch (phase) {
      case Phase::WAITING_FIRST_CLOCK:
        return "WAITING_FIRST_CLOCK";
      case Phase::REQUESTING_INITIAL_PAUSE:
        return "REQUESTING_INITIAL_PAUSE";
      case Phase::WAITING_WORLD_TO_STABILIZE:
        return "WAITING_WORLD_TO_STABILIZE";
      case Phase::WAITING_COMPONENTS_TO_SETTLE:
        return "WAITING_COMPONENTS_TO_SETTLE";
      case Phase::STARTING_SIMULATION:
        return "STARTING_SIMULATION";
      case Phase::WAITING_RUNNING_STATE:
        return "WAITING_RUNNING_STATE";
      case Phase::VERIFYING_NO_NEW_STEP_CLOCK_AFTER_START:
        return "VERIFYING_NO_NEW_STEP_CLOCK_AFTER_START";
      case Phase::BOOTSTRAPPING_STEP:
        return "BOOTSTRAPPING_STEP";
      case Phase::WAITING_FIRST_POST_BOOTSTRAP_CLOCK:
        return "WAITING_FIRST_POST_BOOTSTRAP_CLOCK";
      case Phase::VERIFYING:
        return "VERIFYING";
      case Phase::SUCCESS:
        return "SUCCESS";
      case Phase::FAILURE:
        return "FAILURE";
    }
    return "UNKNOWN";
  }

  int startup_wait_ms_{1500};
  int startup_after_pause_ms_{1500};
  int no_step_verification_ms_{1000};
  int control_timeout_ms_{3000};
  int required_actuator_msgs_{7};
  int stable_checks_required_{3};
  int control_period_steps_{3};
  int verified_actuator_msgs_{0};
  int control_update_count_{0};
  int manager_step_count_{0};
  int exit_code_{EXIT_FAILURE};

  int64_t expected_step_dt_ns_{1000000LL};
  int64_t time_tolerance_ns_{0LL};
  int64_t last_global_time_ns_{0LL};
  int64_t verification_reference_time_ns_{0LL};
  int64_t bootstrap_reference_time_ns_{0LL};
  int64_t verification_baseline_time_ns_{0LL};
  int64_t last_verified_actuator_time_ns_{0LL};

  uint32_t last_global_step_{0};
  uint32_t stable_step_reference_{0};
  uint32_t verification_reference_step_{0};
  uint32_t bootstrap_reference_step_{0};
  uint32_t verification_baseline_step_{0};
  uint32_t last_verified_actuator_step_{0};

  bool done_{false};
  bool received_first_clock_{false};
  bool printed_first_clock_{false};
  bool step_changed_since_last_check_{false};
  bool capture_post_bootstrap_{false};
  bool observed_direct_control_publish_{false};
  bool observed_zoh_hold_publish_{false};

  std::string world_name_;
  std::string group_namespace_;
  std::string actuator_topic_;
  std::string control_service_;
  std::string last_simulation_state_;

  std::size_t stable_checks_count_{0};
  Phase phase_{Phase::WAITING_FIRST_CLOCK};

  rclcpp::Time start_time_;
  rclcpp::Time phase_start_time_;
  rclcpp::Time last_progress_time_;

  std::vector<uint32_t> local_control_steps_;
  std::deque<ActuatorMsg> pending_actuator_msgs_;

  gz::transport::Node gz_node_;

  rclcpp::Subscription<StepClockMsg>::SharedPtr global_step_clock_sub_;
  rclcpp::Subscription<StepClockMsg>::SharedPtr local_step_clock_sub_;
  rclcpp::Subscription<ActuatorMsg>::SharedPtr actuator_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr simulation_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr manager_step_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr bootstrap_step_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerZohTesterNode>();
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
