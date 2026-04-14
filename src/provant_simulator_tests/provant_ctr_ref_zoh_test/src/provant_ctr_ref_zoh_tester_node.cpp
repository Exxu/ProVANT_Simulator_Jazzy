#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <map>
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
#include <provant_simulator_interfaces/msg/float64_array.hpp>
#include <provant_simulator_interfaces/msg/step_clock.hpp>

using namespace std::chrono_literals;

class CtrRefZohTesterNode : public rclcpp::Node
{
public:
  using StepClockMsg = provant_simulator_interfaces::msg::StepClock;
  using FloatArrayMsg = provant_simulator_interfaces::msg::Float64Array;
  using ActuatorMsg = provant_simulator_interfaces::msg::Actuator;

  CtrRefZohTesterNode()
  : rclcpp::Node("provant_ctr_ref_zoh_tester_node")
  {
    startup_wait_ms_ = this->declare_parameter<int>("startup_wait_ms", 1500);
    startup_after_pause_ms_ = this->declare_parameter<int>("startup_after_pause_ms", 1500);
    no_step_verification_ms_ = this->declare_parameter<int>("no_step_verification_ms", 1000);
    control_timeout_ms_ = this->declare_parameter<int>("control_timeout_ms", 3000);
    required_actuator_msgs_ = this->declare_parameter<int>("required_actuator_msgs", 7);
    required_reference_msgs_ = this->declare_parameter<int>("required_reference_msgs", 3);
    pause_quiet_period_ms_ = this->declare_parameter<int>("pause_quiet_period_ms", 600);
    control_period_steps_ = this->declare_parameter<int>("control_period_steps", 3);
    expected_step_dt_ns_ = this->declare_parameter<int64_t>("expected_step_dt_ns", 1000000LL);
    time_tolerance_ns_ = this->declare_parameter<int64_t>("time_tolerance_ns", 0LL);
    world_name_ = this->declare_parameter<std::string>("world_name", "temporal_loop_test");
    group_namespace_ = this->declare_parameter<std::string>("group_namespace", "/test_group");
    actuator_topic_ = this->declare_parameter<std::string>("actuator_topic", "actuator_1");
    reference_tolerance_ = this->declare_parameter<double>("reference_tolerance", 1e-9);
    effort_tolerance_ = this->declare_parameter<double>("effort_tolerance", 1e-9);

    control_service_ = "/world/" + world_name_ + "/control";

    global_step_clock_sub_ = this->create_subscription<StepClockMsg>(
      "/provant_simulator/step_clock",
      rclcpp::QoS(100),
      std::bind(&CtrRefZohTesterNode::onGlobalStepClock, this, std::placeholders::_1));

    local_step_clock_sub_ = this->create_subscription<StepClockMsg>(
      normalizeTopic(group_namespace_, "/step_clock"),
      rclcpp::QoS(100),
      std::bind(&CtrRefZohTesterNode::onLocalStepClock, this, std::placeholders::_1));

    references_sub_ = this->create_subscription<FloatArrayMsg>(
      normalizeTopic(group_namespace_, "/references"),
      rclcpp::QoS(100),
      std::bind(&CtrRefZohTesterNode::onReferences, this, std::placeholders::_1));

    actuator_sub_ = this->create_subscription<ActuatorMsg>(
      normalizeTopic(group_namespace_, "/" + actuator_topic_),
      rclcpp::QoS(100),
      std::bind(&CtrRefZohTesterNode::onActuator, this, std::placeholders::_1));

    simulation_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/provant_simulator/simulation_state",
      rclcpp::QoS(10),
      std::bind(&CtrRefZohTesterNode::onSimulationState, this, std::placeholders::_1));

    manager_step_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/provant_simulator/step",
      rclcpp::QoS(100),
      std::bind(&CtrRefZohTesterNode::onManagerStep, this, std::placeholders::_1));

    set_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/provant_simulator/set_simulation_state", rclcpp::QoS(10));

    bootstrap_step_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/provant_simulator/step", rclcpp::QoS(10));

    timer_ = this->create_wall_timer(50ms, std::bind(&CtrRefZohTesterNode::onTimer, this));

    start_time_ = this->now();
    phase_start_time_ = start_time_;
    last_progress_time_ = start_time_;

    RCLCPP_INFO(
      this->get_logger(),
      "Controller + reference generator + ZOH tester started. world='%s' group='%s' actuator_topic='%s' required_actuator_msgs=%d required_reference_msgs=%d control_period_steps=%d expected_step_dt_ns=%ld",
      world_name_.c_str(),
      group_namespace_.c_str(),
      actuator_topic_.c_str(),
      required_actuator_msgs_,
      required_reference_msgs_,
      control_period_steps_,
      static_cast<long>(expected_step_dt_ns_));
  }

  bool done() const { return done_; }
  int exitCode() const { return exit_code_; }

private:
  struct ControlTick
  {
    uint32_t step;
    int64_t time_ns;
  };

  struct ControlEvent
  {
    uint32_t step;
    int64_t time_ns;
    double expected_effort;
  };

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

  static std::vector<double> expectedReferenceVector(int64_t sim_time_ns)
  {
    const double sec = static_cast<double>(sim_time_ns) / 1e9;
    return {
      std::sin(sec / 2.0),
      std::cos(sec / 2.0),
      sec / 10.0,
      0.0,
      0.0,
      0.0,
      0.5 * std::cos(sec / 2.0),
      -0.5 * std::sin(sec / 2.0),
      0.1,
      0.0,
      0.0,
      0.0,
    };
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
    received_any_global_clock_ = true;
    last_global_clock_wall_time_ = this->now();

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
    }
  }

  void onLocalStepClock(const StepClockMsg::SharedPtr msg)
  {
    if (!capture_post_bootstrap_ || msg->step <= bootstrap_reference_step_) {
      return;
    }

    pending_local_ticks_[msg->step] = ControlTick{msg->step, timeToNanoseconds(msg->time)};
  }

  void onReferences(const FloatArrayMsg::SharedPtr msg)
  {
    if (!capture_post_bootstrap_ || msg->pheader.step <= bootstrap_reference_step_) {
      return;
    }

    pending_references_[msg->pheader.step] = *msg;
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
    RCLCPP_INFO(this->get_logger(), "simulation_manager state='%s'", last_simulation_state_.c_str());
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

    pause_request_wall_time_ = this->now();
    step_changed_since_last_check_ = false;
    phase_ = Phase::WAITING_WORLD_TO_STABILIZE;
    phase_start_time_ = pause_request_wall_time_;

    RCLCPP_INFO(
      this->get_logger(),
      "Pause requested. Waiting for %d ms without new global step_clock messages. current_step=%u current_time_ns=%ld",
      pause_quiet_period_ms_,
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

    rclcpp::Time quiet_reference = pause_request_wall_time_;
    if (received_any_global_clock_ && last_global_clock_wall_time_ > quiet_reference) {
      quiet_reference = last_global_clock_wall_time_;
    }

    const auto quiet_ms = (now - quiet_reference).nanoseconds() / 1000000;
    if (quiet_ms < pause_quiet_period_ms_) {
      return;
    }

    phase_ = Phase::WAITING_COMPONENTS_TO_SETTLE;
    phase_start_time_ = now;
    RCLCPP_INFO(
      this->get_logger(),
      "World appears paused. No new global step_clock for %ld ms. Waiting %d ms for group/controller/reference/ZOH connections.",
      static_cast<long>(quiet_ms),
      startup_after_pause_ms_);
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
      verification_reference_wall_time_ = now;
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
    const auto quiet_ms = (now - verification_reference_wall_time_).nanoseconds() / 1000000;
    if (quiet_ms < no_step_verification_ms_) {
      return;
    }

    phase_ = Phase::BOOTSTRAPPING_STEP;
    phase_start_time_ = now;

    RCLCPP_INFO(
      this->get_logger(),
      "No new global step_clock arrived for %ld ms after start. Sending one bootstrap /provant_simulator/step. reference_step=%u reference_time_ns=%ld",
      static_cast<long>(quiet_ms),
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

    pending_local_ticks_.clear();
    pending_references_.clear();
    pending_actuator_msgs_.clear();
    validated_control_events_.clear();

    validated_reference_msgs_ = 0;
    verified_actuator_msgs_ = 0;
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
        "First post-bootstrap global step_clock observed. global_step=%u global_time_ns=%ld. Starting reference + controller + ZOH verification.",
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
    processPendingReferenceMessages();
    processPendingActuatorMessages();

    if (done_) {
      return;
    }

    if ((now - last_progress_time_).seconds() > 5.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for progress. validated_reference_msgs=%d verified_actuator_msgs=%d pending_local_ticks=%zu pending_references=%zu pending_actuator_msgs=%zu last_global_step=%u",
        validated_reference_msgs_,
        verified_actuator_msgs_,
        pending_local_ticks_.size(),
        pending_references_.size(),
        pending_actuator_msgs_.size(),
        last_global_step_);
      fail();
      return;
    }

    if (
      validated_reference_msgs_ >= required_reference_msgs_ &&
      verified_actuator_msgs_ >= required_actuator_msgs_ &&
      observed_direct_control_publish_ &&
      observed_zoh_hold_publish_)
    {
      done_ = true;
      exit_code_ = EXIT_SUCCESS;
      phase_ = Phase::SUCCESS;
      RCLCPP_INFO(
        this->get_logger(),
        "provant_ctr_ref_zoh_test succeeded. validated_reference_msgs=%d verified_actuator_msgs=%d manager_step_count=%d last_verified_step=%u",
        validated_reference_msgs_,
        verified_actuator_msgs_,
        manager_step_count_,
        last_verified_actuator_step_);
      rclcpp::shutdown();
    }
  }

  void processPendingReferenceMessages()
  {
    bool progressed = true;
    while (progressed && !done_) {
      progressed = false;
      for (auto tick_it = pending_local_ticks_.begin(); tick_it != pending_local_ticks_.end(); ++tick_it) {
        auto ref_it = pending_references_.find(tick_it->first);
        if (ref_it == pending_references_.end()) {
          continue;
        }

        validateReferenceForTick(tick_it->second, ref_it->second);
        if (done_) {
          return;
        }

        pending_references_.erase(ref_it);
        pending_local_ticks_.erase(tick_it);
        progressed = true;
        break;
      }
    }
  }

  void validateReferenceForTick(const ControlTick & tick, const FloatArrayMsg & msg)
  {
    if (msg.pheader.step != tick.step) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Reference/header step mismatch. local_step=%u reference_step=%u",
        tick.step,
        msg.pheader.step);
      fail();
      return;
    }

    const auto expected = expectedReferenceVector(tick.time_ns);
    if (msg.data.size() != expected.size()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unexpected reference vector size on step=%u. observed=%zu expected=%zu",
        tick.step,
        msg.data.size(),
        expected.size());
      fail();
      return;
    }

    for (std::size_t i = 0; i < expected.size(); ++i) {
      if (std::abs(msg.data[i] - expected[i]) > reference_tolerance_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Reference mismatch on step=%u index=%zu observed=%0.17g expected=%0.17g tolerance=%0.17g",
          tick.step,
          i,
          msg.data[i],
          expected[i],
          reference_tolerance_);
        fail();
        return;
      }
    }

    if (!validated_control_events_.empty()) {
      const auto & previous = validated_control_events_.rbegin()->second;
      const auto step_delta = tick.step - previous.step;
      if (step_delta != static_cast<uint32_t>(control_period_steps_)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unexpected control-step spacing. previous_step=%u current_step=%u expected_delta=%d observed_delta=%u",
          previous.step,
          tick.step,
          control_period_steps_,
          step_delta);
        fail();
        return;
      }

      const auto expected_dt = expected_step_dt_ns_ * static_cast<int64_t>(control_period_steps_);
      const auto observed_dt = tick.time_ns - previous.time_ns;
      const auto dt_error = std::llabs(observed_dt - expected_dt);
      if (dt_error > time_tolerance_ns_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unexpected local control tick time increment. previous_time_ns=%ld current_time_ns=%ld observed_dt_ns=%ld expected_dt_ns=%ld tolerance_ns=%ld",
          static_cast<long>(previous.time_ns),
          static_cast<long>(tick.time_ns),
          static_cast<long>(observed_dt),
          static_cast<long>(expected_dt),
          static_cast<long>(time_tolerance_ns_));
        fail();
        return;
      }
    }

    validated_control_events_[tick.step] = ControlEvent{tick.step, tick.time_ns, expected.front()};
    ++validated_reference_msgs_;
    last_progress_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Validated reference msg %d/%d on control step=%u time_ns=%ld first_component=%0.17g",
      validated_reference_msgs_,
      required_reference_msgs_,
      tick.step,
      static_cast<long>(tick.time_ns),
      expected.front());
  }

  const ControlEvent * latestControlEventForStep(uint32_t step) const
  {
    auto it = validated_control_events_.upper_bound(step);
    if (it == validated_control_events_.begin()) {
      return nullptr;
    }

    --it;
    return &it->second;
  }

  void processPendingActuatorMessages()
  {
    while (!pending_actuator_msgs_.empty()) {
      const auto msg = pending_actuator_msgs_.front();
      const auto step = msg.pheader.step;
      const auto time_ns = timeToNanoseconds(msg.pheader.timestamp);

      if (step <= bootstrap_reference_step_) {
        pending_actuator_msgs_.pop_front();
        continue;
      }

      if (step > last_global_step_) {
        return;
      }

      const auto * control_event = latestControlEventForStep(step);
      if (control_event == nullptr) {
        return;
      }

      if (step <= last_verified_actuator_step_) {
        pending_actuator_msgs_.pop_front();
        continue;
      }

      if (step != last_verified_actuator_step_ + 1U) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Actuator step sequence mismatch. previous=%u current=%u last_global_step=%u",
          last_verified_actuator_step_,
          step,
          last_global_step_);
        fail();
        return;
      }

      const auto observed_dt_ns = time_ns - last_verified_actuator_time_ns_;
      const auto time_error_ns = std::llabs(observed_dt_ns - expected_step_dt_ns_);
      if (time_error_ns > time_tolerance_ns_) {
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

      const double expected_effort = control_event->expected_effort;
      if (std::abs(msg.effort - expected_effort) > effort_tolerance_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unexpected actuator effort on global step=%u. observed=%0.17g expected=%0.17g control_step=%u control_time_ns=%ld",
          step,
          msg.effort,
          expected_effort,
          control_event->step,
          static_cast<long>(control_event->time_ns));
        fail();
        return;
      }

      const bool is_control_step = (step == control_event->step);
      if (is_control_step) {
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
        "Validated actuator msg %d/%d global_step=%u time_ns=%ld effort=%0.17g source=%s held_from_control_step=%u",
        verified_actuator_msgs_,
        required_actuator_msgs_,
        step,
        static_cast<long>(time_ns),
        msg.effort,
        is_control_step ? "controller" : "zoh",
        control_event->step);

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

  int startup_wait_ms_{1500};
  int startup_after_pause_ms_{1500};
  int no_step_verification_ms_{1000};
  int control_timeout_ms_{3000};
  int required_actuator_msgs_{7};
  int required_reference_msgs_{3};
  int pause_quiet_period_ms_{600};
  int control_period_steps_{3};
  int validated_reference_msgs_{0};
  int verified_actuator_msgs_{0};
  int manager_step_count_{0};
  int exit_code_{EXIT_FAILURE};

  int64_t expected_step_dt_ns_{1000000LL};
  int64_t time_tolerance_ns_{0LL};
  int64_t last_global_time_ns_{0LL};
  int64_t verification_reference_time_ns_{0LL};
  int64_t bootstrap_reference_time_ns_{0LL};
  int64_t last_verified_actuator_time_ns_{0LL};

  uint32_t last_global_step_{0};
  uint32_t verification_reference_step_{0};
  uint32_t bootstrap_reference_step_{0};
  uint32_t last_verified_actuator_step_{0};

  bool done_{false};
  bool received_first_clock_{false};
  bool printed_first_clock_{false};
  bool step_changed_since_last_check_{false};
  bool received_any_global_clock_{false};
  bool capture_post_bootstrap_{false};
  bool observed_direct_control_publish_{false};
  bool observed_zoh_hold_publish_{false};

  double reference_tolerance_{1e-9};
  double effort_tolerance_{1e-9};

  std::string world_name_;
  std::string group_namespace_;
  std::string actuator_topic_;
  std::string control_service_;
  std::string last_simulation_state_;

  Phase phase_{Phase::WAITING_FIRST_CLOCK};

  rclcpp::Time start_time_;
  rclcpp::Time phase_start_time_;
  rclcpp::Time last_progress_time_;
  rclcpp::Time last_global_clock_wall_time_;
  rclcpp::Time pause_request_wall_time_;
  rclcpp::Time verification_reference_wall_time_;

  std::map<uint32_t, ControlTick> pending_local_ticks_;
  std::map<uint32_t, FloatArrayMsg> pending_references_;
  std::map<uint32_t, ControlEvent> validated_control_events_;
  std::deque<ActuatorMsg> pending_actuator_msgs_;

  gz::transport::Node gz_node_;

  rclcpp::Subscription<StepClockMsg>::SharedPtr global_step_clock_sub_;
  rclcpp::Subscription<StepClockMsg>::SharedPtr local_step_clock_sub_;
  rclcpp::Subscription<FloatArrayMsg>::SharedPtr references_sub_;
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
  auto node = std::make_shared<CtrRefZohTesterNode>();
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
