\
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

#include <provant_simulator_interfaces/msg/step_clock.hpp>
#include <provant_simulator_interfaces/srv/control_group_register.hpp>

using namespace std::chrono_literals;

class TemporalLoopTesterNode : public rclcpp::Node
{
public:
  TemporalLoopTesterNode()
  : Node("temporal_loop_tester_node")
  {
    total_cycles_ = this->declare_parameter<int>("total_cycles", 5);
    request_period_ms_ = this->declare_parameter<int>("request_period_ms", 300);
    startup_wait_ms_ = this->declare_parameter<int>("startup_wait_ms", 1500);
    world_name_ = this->declare_parameter<std::string>("world_name", "temporal_loop_test");
    control_timeout_ms_ = this->declare_parameter<int>("control_timeout_ms", 3000);
    stable_checks_required_ = this->declare_parameter<int>("stable_checks_required", 3);
    group_namespace_ = this->declare_parameter<std::string>("group_namespace", "/test_group");

    control_service_ = "/world/" + world_name_ + "/control";

    step_clock_sub_ =
      this->create_subscription<provant_simulator_interfaces::msg::StepClock>(
        "/provant_simulator/step_clock",
        rclcpp::QoS(50),
        std::bind(&TemporalLoopTesterNode::onStepClock, this, std::placeholders::_1));

    manager_step_sub_ =
      this->create_subscription<std_msgs::msg::Empty>(
        "/provant_simulator/step",
        rclcpp::QoS(50),
        std::bind(&TemporalLoopTesterNode::onManagerStep, this, std::placeholders::_1));

    simulation_state_sub_ =
      this->create_subscription<std_msgs::msg::String>(
        "/provant_simulator/simulation_state",
        rclcpp::QoS(10),
        std::bind(&TemporalLoopTesterNode::onSimulationState, this, std::placeholders::_1));

    ready_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      group_namespace_ + "/ready", rclcpp::QoS(10));

    set_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/provant_simulator/set_simulation_state", rclcpp::QoS(10));

    register_client_ =
      this->create_client<provant_simulator_interfaces::srv::ControlGroupRegister>(
        "/provant_simulator/register_group");

    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      group_namespace_ + "/reset",
      std::bind(
        &TemporalLoopTesterNode::onGroupReset,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    sim_reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "/provant_simulator/reset",
      std::bind(
        &TemporalLoopTesterNode::onSimulationReset,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    timer_ = this->create_wall_timer(
      50ms, std::bind(&TemporalLoopTesterNode::onTimer, this));

    start_time_ = this->now();
    phase_start_time_ = start_time_;

    RCLCPP_INFO(
      this->get_logger(),
      "Tester started. total_cycles=%d request_period_ms=%d startup_wait_ms=%d world_name='%s' group_namespace='%s'",
      total_cycles_, request_period_ms_, startup_wait_ms_, world_name_.c_str(), group_namespace_.c_str());
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
    REGISTERING_GROUP,
    WAITING_REGISTER_RESPONSE,
    STARTING_SIMULATION,
    WAITING_RUNNING_STATE,
    PUBLISHING_READY,
    WAITING_STEP_RESULT,
    SUCCESS,
    FAILURE
  };

  void onGroupReset(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    ++group_reset_count_;
    RCLCPP_INFO(this->get_logger(), "Group reset service called.");
  }

  void onSimulationReset(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    ++sim_reset_count_;
    RCLCPP_INFO(this->get_logger(), "Simulation reset service called.");
  }

  void onStepClock(const provant_simulator_interfaces::msg::StepClock::SharedPtr msg)
  {
    last_step_ = msg->step;
    received_first_clock_ = true;
    step_changed_since_last_check_ = true;

    if (!printed_first_clock_) {
      printed_first_clock_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial step_clock received: step=%u", last_step_);
      return;
    }

    if (phase_ == Phase::WAITING_STEP_RESULT) {
      if (last_step_ <= expected_base_step_) {
        return;
      }

      const auto delta =
        static_cast<int64_t>(last_step_) - static_cast<int64_t>(expected_base_step_);

      if (expected_manager_step_count_ != manager_step_count_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unexpected manager step publish count. expected=%d observed=%d",
          expected_manager_step_count_,
          manager_step_count_);
        fail();
        return;
      }

      if (delta == 1) {
        ++successful_cycles_;
        phase_ = Phase::PUBLISHING_READY;
        phase_start_time_ = this->now();

        RCLCPP_INFO(
          this->get_logger(),
          "Cycle %d/%d succeeded. manager_step_count=%d base_step=%u new_step=%u",
          successful_cycles_,
          total_cycles_,
          manager_step_count_,
          expected_base_step_,
          last_step_);

        if (successful_cycles_ >= total_cycles_) {
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

  void onManagerStep(const std_msgs::msg::Empty::SharedPtr /*msg*/)
  {
    ++manager_step_count_;
    RCLCPP_INFO(
      this->get_logger(),
      "simulation_manager published /provant_simulator/step. count=%d",
      manager_step_count_);
  }

  void onSimulationState(const std_msgs::msg::String::SharedPtr msg)
  {
    last_simulation_state_ = msg->data;
    RCLCPP_INFO(
      this->get_logger(),
      "simulation_manager published state='%s'",
      last_simulation_state_.c_str());
  }

  void onTimer()
  {
    if (done_) {
      return;
    }

    const auto now = this->now();

    if ((now - start_time_).seconds() > 40.0) {
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
      case Phase::REGISTERING_GROUP:
        handleRegisteringGroup();
        return;
      case Phase::WAITING_REGISTER_RESPONSE:
        handleWaitingRegisterResponse(now);
        return;
      case Phase::STARTING_SIMULATION:
        handleStartingSimulation();
        return;
      case Phase::WAITING_RUNNING_STATE:
        handleWaitingRunningState(now);
        return;
      case Phase::PUBLISHING_READY:
        handlePublishingReady(now);
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
      phase_ = Phase::REGISTERING_GROUP;
      phase_start_time_ = now;
      RCLCPP_INFO(
        this->get_logger(),
        "World appears paused. Registering control group '%s'.",
        group_namespace_.c_str());
      return;
    }

    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for step stabilization after pause request.");
      fail();
    }
  }

  void handleRegisteringGroup()
  {
    if (!register_client_->wait_for_service(0s)) {
      return;
    }

    auto request =
      std::make_shared<provant_simulator_interfaces::srv::ControlGroupRegister::Request>();
    request->group_namespace = group_namespace_;
    register_future_ = register_client_->async_send_request(request);

    phase_ = Phase::WAITING_REGISTER_RESPONSE;
    phase_start_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Registering group '%s' in simulation_manager.",
      group_namespace_.c_str());
  }

  void handleWaitingRegisterResponse(const rclcpp::Time& now)
  {
    if (!register_future_.valid()) {
      RCLCPP_ERROR(this->get_logger(), "Register service future is invalid.");
      fail();
      return;
    }

    const auto status = register_future_.wait_for(0ms);
    if (status == std::future_status::ready) {
      const auto response = register_future_.get();
      if (!response->result) {
        RCLCPP_ERROR(
          this->get_logger(),
          "register_group returned failure: %s",
          response->message.c_str());
        fail();
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Group registration succeeded.");
      phase_ = Phase::STARTING_SIMULATION;
      phase_start_time_ = now;
      return;
    }

    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for register_group response.");
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

    RCLCPP_INFO(this->get_logger(), "Requested simulation_manager state 'start'.");
  }

  void handleWaitingRunningState(const rclcpp::Time& now)
  {
    if (last_simulation_state_ == "running") {
      phase_ = Phase::PUBLISHING_READY;
      phase_start_time_ = now;
      RCLCPP_INFO(this->get_logger(), "simulation_manager is running. Starting cycles.");
      return;
    }

    if ((now - phase_start_time_).seconds() > 5.0) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for simulation_manager to enter running state.");
      fail();
    }
  }

  void handlePublishingReady(const rclcpp::Time& now)
  {
    if ((now - phase_start_time_).nanoseconds() / 1000000 < request_period_ms_) {
      return;
    }

    expected_base_step_ = last_step_;
    expected_manager_step_count_ = manager_step_count_ + 1;

    std_msgs::msg::Empty msg;
    ready_pub_->publish(msg);

    phase_ = Phase::WAITING_STEP_RESULT;
    phase_start_time_ = now;
    ++sent_ready_messages_;

    RCLCPP_INFO(
      this->get_logger(),
      "Published %s/ready for cycle %d/%d. expected_base_step=%u expected_manager_step_count=%d",
      group_namespace_.c_str(),
      sent_ready_messages_,
      total_cycles_,
      expected_base_step_,
      expected_manager_step_count_);
  }

  void handleWaitingStepResult(const rclcpp::Time& now)
  {
    if ((now - phase_start_time_).seconds() > 3.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for end-to-end temporal loop result for cycle %d.",
        sent_ready_messages_);
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

  rclcpp::Subscription<provant_simulator_interfaces::msg::StepClock>::SharedPtr step_clock_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr manager_step_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr simulation_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_state_pub_;
  rclcpp::Client<provant_simulator_interfaces::srv::ControlGroupRegister>::SharedPtr register_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
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
  uint32_t expected_base_step_{0};
  uint32_t stable_step_reference_{0};

  int total_cycles_{5};
  int request_period_ms_{300};
  int startup_wait_ms_{1500};
  int control_timeout_ms_{3000};
  int stable_checks_required_{3};

  int stable_checks_count_{0};
  int sent_ready_messages_{0};
  int successful_cycles_{0};
  int manager_step_count_{0};
  int expected_manager_step_count_{0};
  int group_reset_count_{0};
  int sim_reset_count_{0};

  std::string world_name_;
  std::string control_service_;
  std::string group_namespace_;
  std::string last_simulation_state_;

  rclcpp::Client<provant_simulator_interfaces::srv::ControlGroupRegister>::SharedFuture register_future_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TemporalLoopTesterNode>();
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
