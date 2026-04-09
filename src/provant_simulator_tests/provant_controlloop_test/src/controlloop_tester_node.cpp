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
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <provant_simulator_interfaces/msg/empty.hpp>
#include <provant_simulator_interfaces/msg/float64_array.hpp>
#include <provant_simulator_interfaces/msg/step_clock.hpp>

using namespace std::chrono_literals;

class ControlLoopTesterNode : public rclcpp::Node
{
public:
  ControlLoopTesterNode() : Node("controlloop_tester_node")
  {
    total_cycles_ = this->declare_parameter<int>("total_cycles", 6);
    request_period_ms_ = this->declare_parameter<int>("request_period_ms", 300);
    startup_wait_ms_ = this->declare_parameter<int>("startup_wait_ms", 1500);
    world_name_ = this->declare_parameter<std::string>("world_name", "temporal_loop_test");
    control_timeout_ms_ = this->declare_parameter<int>("control_timeout_ms", 3000);
    stable_checks_required_ = this->declare_parameter<int>("stable_checks_required", 3);
    group_namespace_ = this->declare_parameter<std::string>("group_namespace", "/test_group");
    expected_step_dt_ns_ = this->declare_parameter<int64_t>("expected_step_dt_ns", 1000000LL);
    time_tolerance_ns_ = this->declare_parameter<int64_t>("time_tolerance_ns", 0LL);
    control_period_steps_ = this->declare_parameter<int>("control_period_steps", 3);

    control_service_ = "/world/" + world_name_ + "/control";

    step_clock_sub_ = this->create_subscription<provant_simulator_interfaces::msg::StepClock>(
      "/provant_simulator/step_clock", rclcpp::QoS(50),
      std::bind(&ControlLoopTesterNode::onStepClock, this, std::placeholders::_1));
    manager_step_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/provant_simulator/step", rclcpp::QoS(50),
      std::bind(&ControlLoopTesterNode::onManagerStep, this, std::placeholders::_1));
    simulation_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/provant_simulator/simulation_state", rclcpp::QoS(10),
      std::bind(&ControlLoopTesterNode::onSimulationState, this, std::placeholders::_1));

    group_ready_sub_ = this->create_subscription<provant_simulator_interfaces::msg::Empty>(
      group_namespace_ + "/ready", rclcpp::QoS(20),
      std::bind(&ControlLoopTesterNode::onGroupReady, this, std::placeholders::_1));
    group_step_clock_sub_ = this->create_subscription<provant_simulator_interfaces::msg::StepClock>(
      group_namespace_ + "/step_clock", rclcpp::QoS(20),
      std::bind(&ControlLoopTesterNode::onGroupStepClock, this, std::placeholders::_1));
    zoh_trigger_sub_ = this->create_subscription<provant_simulator_interfaces::msg::Empty>(
      group_namespace_ + "/zoh_trigger", rclcpp::QoS(20),
      std::bind(&ControlLoopTesterNode::onZohTrigger, this, std::placeholders::_1));
    control_inputs_sub_ = this->create_subscription<provant_simulator_interfaces::msg::Float64Array>(
      group_namespace_ + "/control_inputs", rclcpp::QoS(20),
      std::bind(&ControlLoopTesterNode::onControlInputs, this, std::placeholders::_1));
    disturbances_sub_ = this->create_subscription<provant_simulator_interfaces::msg::Float64Array>(
      group_namespace_ + "/disturbances", rclcpp::QoS(20),
      std::bind(&ControlLoopTesterNode::onDisturbances, this, std::placeholders::_1));

    bootstrap_ready_pub_ = this->create_publisher<provant_simulator_interfaces::msg::Empty>(
      group_namespace_ + "/ready", rclcpp::QoS(10));
    set_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/provant_simulator/set_simulation_state", rclcpp::QoS(10));

    group_reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      group_namespace_ + "/reset",
      [](const std::shared_ptr<std_srvs::srv::Empty::Request>,
         std::shared_ptr<std_srvs::srv::Empty::Response>) {});
    sim_reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "/provant_simulator/reset",
      [](const std::shared_ptr<std_srvs::srv::Empty::Request>,
         std::shared_ptr<std_srvs::srv::Empty::Response>) {});

    timer_ = this->create_wall_timer(50ms, std::bind(&ControlLoopTesterNode::onTimer, this));
    start_time_ = this->now();
    phase_start_time_ = start_time_;

    RCLCPP_INFO(this->get_logger(), "Control loop tester started.");
  }

  bool done() const { return done_; }
  int exitCode() const { return exit_code_; }

private:
  enum class Phase {
    WAITING_FIRST_CLOCK,
    REQUESTING_INITIAL_PAUSE,
    WAITING_WORLD_TO_STABILIZE,
    WAITING_GROUP_REGISTRATION,
    STARTING_SIMULATION,
    WAITING_RUNNING_STATE,
    PUBLISHING_BOOTSTRAP_READY,
    WAITING_BOOTSTRAP_STEP_RESULT,
    WAITING_NEXT_BASE_CLOCK,
    WAITING_GROUP_READY,
    WAITING_STEP_RESULT,
    SUCCESS,
    FAILURE
  };

  static int64_t timeToNanoseconds(const builtin_interfaces::msg::Time & t)
  {
    return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
  }

  bool shouldRunControl(uint32_t step) const
  {
    return (step % static_cast<uint32_t>(control_period_steps_)) == 0U;
  }

  void resetCycleObservations()
  {
    observed_manager_step_for_cycle_ = false;
    observed_step_clock_for_cycle_ = false;
    group_ready_observed_ = false;
    observed_group_step_clock_ = false;
    observed_zoh_trigger_ = false;
    observed_control_inputs_ = false;
    observed_disturbances_ = false;
    observed_step_delta_ = 0;
    observed_time_delta_ns_ = 0;
    ready_step_ = 0;
    ready_time_ns_ = 0;
    next_step_ = 0;
    next_time_ns_ = 0;
    group_step_clock_count_ = 0;
    zoh_trigger_count_ = 0;
    control_inputs_count_ = 0;
    disturbances_count_ = 0;
    ready_count_ = 0;
  }

  void armCycleFromClock(uint32_t step, int64_t time_ns, const rclcpp::Time & now)
  {
    expected_base_step_ = step;
    expected_base_time_ns_ = time_ns;
    expected_manager_step_count_ = manager_step_count_ + 1;
    resetCycleObservations();
    phase_ = Phase::WAITING_GROUP_READY;
    phase_start_time_ = now;
    RCLCPP_INFO(
      this->get_logger(),
      "Observed cycle %d/%d start at global step=%u time_ns=%ld expect_control=%s",
      successful_cycles_ + 1, total_cycles_, expected_base_step_, static_cast<long>(expected_base_time_ns_),
      shouldRunControl(expected_base_step_) ? "true" : "false");
  }

  void onStepClock(const provant_simulator_interfaces::msg::StepClock::SharedPtr msg)
  {
    const auto step = msg->step;
    const auto time_ns = timeToNanoseconds(msg->time);

    const uint32_t previous_step = last_step_;
    const int64_t previous_time_ns = last_time_ns_;
    last_step_ = step;
    last_time_ns_ = time_ns;
    received_first_clock_ = true;
    step_changed_since_last_check_ = true;

    if (!printed_first_clock_) {
      printed_first_clock_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial step_clock received: step=%u time_ns=%ld", last_step_, static_cast<long>(last_time_ns_));
      return;
    }

    if (phase_ == Phase::WAITING_BOOTSTRAP_STEP_RESULT) {
      if (step > bootstrap_base_step_) {
        const auto step_delta = static_cast<int64_t>(step) - static_cast<int64_t>(bootstrap_base_step_);
        const auto time_delta = time_ns - bootstrap_base_time_ns_;
        if (step_delta != 1 || llabs(time_delta - expected_step_dt_ns_) > time_tolerance_ns_) {
          RCLCPP_ERROR(this->get_logger(), "Bootstrap cycle invalid. step_delta=%ld time_delta_ns=%ld", static_cast<long>(step_delta), static_cast<long>(time_delta));
          fail();
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Bootstrap cycle succeeded. base_step=%u new_step=%u", bootstrap_base_step_, step);
        phase_ = Phase::WAITING_NEXT_BASE_CLOCK;
        phase_start_time_ = this->now();
      }
      return;
    }

    if (phase_ == Phase::WAITING_NEXT_BASE_CLOCK) {
      if (step > previous_step && previous_step != 0U) {
        armCycleFromClock(step, time_ns, this->now());
      }
      return;
    }

    if (phase_ == Phase::WAITING_GROUP_READY) {
      if (step > expected_base_step_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "A new global step_clock arrived before ready for the observed cycle. base_step=%u new_step=%u",
          expected_base_step_, step);
        fail();
      }
      return;
    }

    if (phase_ != Phase::WAITING_STEP_RESULT) {
      return;
    }

    if (step <= expected_base_step_) {
      return;
    }

    observed_step_clock_for_cycle_ = true;
    next_step_ = step;
    next_time_ns_ = time_ns;
    observed_step_delta_ = static_cast<int64_t>(step) - static_cast<int64_t>(expected_base_step_);
    observed_time_delta_ns_ = time_ns - expected_base_time_ns_;
    maybeCompleteCycle();
  }

  void onManagerStep(const std_msgs::msg::Empty::SharedPtr)
  {
    ++manager_step_count_;
    if (phase_ == Phase::WAITING_GROUP_READY) {
      RCLCPP_ERROR(this->get_logger(), "simulation_manager published /step before group ready for observed cycle starting at step %u", expected_base_step_);
      fail();
      return;
    }

    if (phase_ == Phase::WAITING_STEP_RESULT && manager_step_count_ >= expected_manager_step_count_) {
      observed_manager_step_for_cycle_ = true;
      maybeCompleteCycle();
    }
  }

  void onSimulationState(const std_msgs::msg::String::SharedPtr msg)
  {
    last_simulation_state_ = msg->data;
  }

  void onGroupReady(const provant_simulator_interfaces::msg::Empty::SharedPtr msg)
  {
    if (phase_ != Phase::WAITING_GROUP_READY) {
      return;
    }

    const auto step = msg->pheader.step;
    const auto time_ns = timeToNanoseconds(msg->pheader.timestamp);
    if (step != expected_base_step_ || time_ns != expected_base_time_ns_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Ready mismatch. expected step=%u time_ns=%ld observed step=%u time_ns=%ld",
        expected_base_step_, static_cast<long>(expected_base_time_ns_), step, static_cast<long>(time_ns));
      fail();
      return;
    }

    ++ready_count_;
    if (ready_count_ > 1) {
      RCLCPP_ERROR(this->get_logger(), "More than one ready observed for cycle starting at step %u", expected_base_step_);
      fail();
      return;
    }

    group_ready_observed_ = true;
    ready_step_ = step;
    ready_time_ns_ = time_ns;
    validateGroupActivityAndAdvance();
  }

  void onGroupStepClock(const provant_simulator_interfaces::msg::StepClock::SharedPtr msg)
  {
    if (phase_ != Phase::WAITING_GROUP_READY || msg->step != expected_base_step_) {
      return;
    }
    ++group_step_clock_count_;
    observed_group_step_clock_ = true;
  }

  void onZohTrigger(const provant_simulator_interfaces::msg::Empty::SharedPtr msg)
  {
    if (phase_ != Phase::WAITING_GROUP_READY || msg->pheader.step != expected_base_step_) {
      return;
    }
    ++zoh_trigger_count_;
    observed_zoh_trigger_ = true;
  }

  void onControlInputs(const provant_simulator_interfaces::msg::Float64Array::SharedPtr msg)
  {
    if (phase_ != Phase::WAITING_GROUP_READY || msg->pheader.step != expected_base_step_) {
      return;
    }
    ++control_inputs_count_;
    observed_control_inputs_ = true;
  }

  void onDisturbances(const provant_simulator_interfaces::msg::Float64Array::SharedPtr msg)
  {
    if (phase_ != Phase::WAITING_GROUP_READY || msg->pheader.step != expected_base_step_) {
      return;
    }
    ++disturbances_count_;
    observed_disturbances_ = true;
  }

  void validateGroupActivityAndAdvance()
  {
    const bool expect_control = shouldRunControl(expected_base_step_);

    if (expect_control) {
      if (group_step_clock_count_ != 1 || control_inputs_count_ != 1 || disturbances_count_ != 1 || zoh_trigger_count_ != 0) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Invalid control branch activity for step %u. group_step_clock=%d control_inputs=%d disturbances=%d zoh=%d",
          expected_base_step_, group_step_clock_count_, control_inputs_count_, disturbances_count_, zoh_trigger_count_);
        fail();
        return;
      }
    } else {
      if (group_step_clock_count_ != 0 || control_inputs_count_ != 0 || disturbances_count_ != 0 || zoh_trigger_count_ != 1) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Invalid ZOH branch activity for step %u. group_step_clock=%d control_inputs=%d disturbances=%d zoh=%d",
          expected_base_step_, group_step_clock_count_, control_inputs_count_, disturbances_count_, zoh_trigger_count_);
        fail();
        return;
      }
    }

    phase_ = Phase::WAITING_STEP_RESULT;
    phase_start_time_ = this->now();
    RCLCPP_INFO(
      this->get_logger(),
      "Group ready observed for cycle starting at step %u. Waiting for one manager /step and the next global step_clock.",
      expected_base_step_);
  }

  void maybeCompleteCycle()
  {
    if (phase_ != Phase::WAITING_STEP_RESULT) {
      return;
    }
    if (!observed_manager_step_for_cycle_ || !observed_step_clock_for_cycle_) {
      return;
    }
    if (manager_step_count_ != expected_manager_step_count_) {
      RCLCPP_ERROR(this->get_logger(), "Unexpected manager step count. expected=%d observed=%d", expected_manager_step_count_, manager_step_count_);
      fail();
      return;
    }
    if (ready_count_ != 1 || ready_step_ != expected_base_step_ || ready_time_ns_ != expected_base_time_ns_) {
      RCLCPP_ERROR(this->get_logger(), "Ready accounting invalid for cycle starting at step %u", expected_base_step_);
      fail();
      return;
    }
    if (observed_step_delta_ != 1) {
      RCLCPP_ERROR(this->get_logger(), "Expected delta_step=1 but observed %ld", static_cast<long>(observed_step_delta_));
      fail();
      return;
    }
    const auto time_error_ns = observed_time_delta_ns_ - expected_step_dt_ns_;
    if (llabs(time_error_ns) > time_tolerance_ns_) {
      RCLCPP_ERROR(this->get_logger(), "Unexpected delta_time_ns=%ld expected=%ld", static_cast<long>(observed_time_delta_ns_), static_cast<long>(expected_step_dt_ns_));
      fail();
      return;
    }

    ++successful_cycles_;
    RCLCPP_INFO(
      this->get_logger(),
      "Cycle %d/%d succeeded. %u -> %u",
      successful_cycles_, total_cycles_, expected_base_step_, next_step_);
    if (successful_cycles_ >= total_cycles_) {
      phase_ = Phase::SUCCESS;
      done_ = true;
      exit_code_ = EXIT_SUCCESS;
      RCLCPP_INFO(this->get_logger(), "Test succeeded.");
      rclcpp::shutdown();
      return;
    }

    phase_ = Phase::WAITING_NEXT_BASE_CLOCK;
    phase_start_time_ = this->now();
  }

  void onTimer()
  {
    if (done_) return;
    const auto now = this->now();
    if ((now - start_time_).seconds() > 50.0) {
      RCLCPP_ERROR(this->get_logger(), "Global timeout.");
      fail();
      return;
    }

    switch (phase_) {
      case Phase::WAITING_FIRST_CLOCK: handleWaitingFirstClock(now); return;
      case Phase::REQUESTING_INITIAL_PAUSE: handleRequestingInitialPause(); return;
      case Phase::WAITING_WORLD_TO_STABILIZE: handleWaitingWorldToStabilize(now); return;
      case Phase::WAITING_GROUP_REGISTRATION: handleWaitingGroupRegistration(now); return;
      case Phase::STARTING_SIMULATION: handleStartingSimulation(); return;
      case Phase::WAITING_RUNNING_STATE: handleWaitingRunningState(now); return;
      case Phase::PUBLISHING_BOOTSTRAP_READY: handlePublishingBootstrapReady(now); return;
      case Phase::WAITING_BOOTSTRAP_STEP_RESULT: handleWaitingBootstrapStepResult(now); return;
      case Phase::WAITING_NEXT_BASE_CLOCK: handleWaitingNextBaseClock(now); return;
      case Phase::WAITING_GROUP_READY: handleWaitingGroupReady(now); return;
      case Phase::WAITING_STEP_RESULT: handleWaitingStepResult(now); return;
      case Phase::SUCCESS:
      case Phase::FAILURE: return;
    }
  }

  void handleWaitingFirstClock(const rclcpp::Time & now)
  {
    if (received_first_clock_ && (now - start_time_).nanoseconds() / 1000000 >= startup_wait_ms_) {
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
      phase_ = Phase::WAITING_GROUP_REGISTRATION;
      phase_start_time_ = now;
      RCLCPP_INFO(this->get_logger(), "World appears paused. Waiting for control group publishers / registration.");
      return;
    }
    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for step stabilization after pause request.");
      fail();
    }
  }

  void handleWaitingGroupRegistration(const rclcpp::Time & now)
  {
    const auto ready_publishers = group_ready_sub_->get_publisher_count();
    const auto step_publishers = group_step_clock_sub_->get_publisher_count();
    if (ready_publishers > 0 && step_publishers > 0 && (now - phase_start_time_).seconds() > 0.5) {
      phase_ = Phase::STARTING_SIMULATION;
      phase_start_time_ = now;
      return;
    }
    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for control_group_manager publishers.");
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
  }

  void handleWaitingRunningState(const rclcpp::Time & now)
  {
    if (last_simulation_state_ == "running") {
      phase_ = Phase::PUBLISHING_BOOTSTRAP_READY;
      phase_start_time_ = now;
      RCLCPP_INFO(this->get_logger(), "simulation_manager is running. Publishing bootstrap ready.");
      return;
    }
    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for running state.");
      fail();
    }
  }

  void handlePublishingBootstrapReady(const rclcpp::Time & now)
  {
    if ((now - phase_start_time_).nanoseconds() / 1000000 < request_period_ms_) {
      return;
    }
    bootstrap_base_step_ = last_step_;
    bootstrap_base_time_ns_ = last_time_ns_;
    bootstrap_expected_manager_step_count_ = manager_step_count_ + 1;
    provant_simulator_interfaces::msg::Empty msg;
    bootstrap_ready_pub_->publish(msg);
    phase_ = Phase::WAITING_BOOTSTRAP_STEP_RESULT;
    phase_start_time_ = now;
    RCLCPP_INFO(this->get_logger(), "Published bootstrap ready for group %s. base_step=%u", group_namespace_.c_str(), bootstrap_base_step_);
  }

  void handleWaitingBootstrapStepResult(const rclcpp::Time & now)
  {
    if (manager_step_count_ < bootstrap_expected_manager_step_count_) {
      if ((now - phase_start_time_).seconds() > 3.0) {
        RCLCPP_ERROR(this->get_logger(), "Timed out waiting for bootstrap manager step.");
        fail();
      }
      return;
    }
    if ((now - phase_start_time_).seconds() > 3.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting bootstrap step result.");
      fail();
    }
  }

  void handleWaitingNextBaseClock(const rclcpp::Time & now)
  {
    if ((now - phase_start_time_).seconds() > 3.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for the next observable global step_clock to start a cycle.");
      fail();
    }
  }

  void handleWaitingGroupReady(const rclcpp::Time & now)
  {
    if ((now - phase_start_time_).seconds() > 3.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for control group ready / activity for cycle starting at step %u.", expected_base_step_);
      fail();
    }
  }

  void handleWaitingStepResult(const rclcpp::Time & now)
  {
    if ((now - phase_start_time_).seconds() > 3.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for end-to-end temporal loop result for cycle starting at step %u.", expected_base_step_);
      fail();
    }
  }

  bool requestPause()
  {
    gz::msgs::WorldControl req;
    gz::msgs::Boolean rep;
    bool result = false;
    req.set_pause(true);
    const bool executed = gz_node_.Request(control_service_, req, static_cast<unsigned int>(control_timeout_ms_), rep, result);
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
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr manager_step_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr simulation_state_sub_;
  rclcpp::Subscription<provant_simulator_interfaces::msg::Empty>::SharedPtr group_ready_sub_;
  rclcpp::Subscription<provant_simulator_interfaces::msg::StepClock>::SharedPtr group_step_clock_sub_;
  rclcpp::Subscription<provant_simulator_interfaces::msg::Empty>::SharedPtr zoh_trigger_sub_;
  rclcpp::Subscription<provant_simulator_interfaces::msg::Float64Array>::SharedPtr control_inputs_sub_;
  rclcpp::Subscription<provant_simulator_interfaces::msg::Float64Array>::SharedPtr disturbances_sub_;
  rclcpp::Publisher<provant_simulator_interfaces::msg::Empty>::SharedPtr bootstrap_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_state_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr group_reset_srv_;
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
  bool observed_manager_step_for_cycle_{false};
  bool observed_step_clock_for_cycle_{false};
  bool group_ready_observed_{false};
  bool observed_group_step_clock_{false};
  bool observed_zoh_trigger_{false};
  bool observed_control_inputs_{false};
  bool observed_disturbances_{false};

  int exit_code_{EXIT_FAILURE};
  uint32_t last_step_{0};
  uint32_t expected_base_step_{0};
  uint32_t stable_step_reference_{0};
  uint32_t bootstrap_base_step_{0};
  uint32_t ready_step_{0};
  uint32_t next_step_{0};
  int64_t last_time_ns_{0};
  int64_t expected_base_time_ns_{0};
  int64_t bootstrap_base_time_ns_{0};
  int64_t expected_step_dt_ns_{1000000};
  int64_t time_tolerance_ns_{0};
  int64_t observed_step_delta_{0};
  int64_t observed_time_delta_ns_{0};
  int64_t ready_time_ns_{0};
  int64_t next_time_ns_{0};

  int total_cycles_{6};
  int request_period_ms_{300};
  int startup_wait_ms_{1500};
  int control_timeout_ms_{3000};
  int stable_checks_required_{3};
  int control_period_steps_{3};
  int stable_checks_count_{0};
  int successful_cycles_{0};
  int manager_step_count_{0};
  int expected_manager_step_count_{0};
  int bootstrap_expected_manager_step_count_{0};
  int group_step_clock_count_{0};
  int zoh_trigger_count_{0};
  int control_inputs_count_{0};
  int disturbances_count_{0};
  int ready_count_{0};

  std::string world_name_;
  std::string control_service_;
  std::string group_namespace_;
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
