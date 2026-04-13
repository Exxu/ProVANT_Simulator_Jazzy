#include <memory>
#include <functional>
#include <string>

#include <provant_simulator_interfaces/msg/float64_array.hpp>
#include <provant_simulator_interfaces/msg/step_clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

class ControllerInputSourceNode : public rclcpp::Node
{
public:
  using FloatArrayMsg = provant_simulator_interfaces::msg::Float64Array;
  using StepClockMsg = provant_simulator_interfaces::msg::StepClock;
  using EmptySrv = std_srvs::srv::Empty;

  ControllerInputSourceNode()
  : rclcpp::Node("controller_input_source_node")
  {
    const auto step_clock_topic = this->declare_parameter<std::string>("step_clock_topic", "step_clock");

    references_pub_ = this->create_publisher<FloatArrayMsg>("references", rclcpp::QoS(10));
    state_vector_pub_ = this->create_publisher<FloatArrayMsg>("state_vector", rclcpp::QoS(10));

    step_clock_sub_ = this->create_subscription<StepClockMsg>(
      step_clock_topic,
      rclcpp::QoS(10),
      std::bind(&ControllerInputSourceNode::onStepClock, this, std::placeholders::_1));

    reset_srv_ = this->create_service<EmptySrv>(
      "reset",
      [this](const EmptySrv::Request::SharedPtr /*request*/, EmptySrv::Response::SharedPtr /*response*/) {
        RCLCPP_INFO(this->get_logger(), "Reference generator reset requested.");
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Controller input source ready. step_clock_topic='%s'",
      step_clock_topic.c_str());
  }

private:
  void onStepClock(const StepClockMsg::SharedPtr msg)
  {
    const auto step_as_double = static_cast<double>(msg->step);

    FloatArrayMsg references_msg;
    references_msg.pheader.step = msg->step;
    references_msg.pheader.timestamp = msg->time;
    references_msg.data = {2.0 * step_as_double};
    references_pub_->publish(references_msg);

    FloatArrayMsg state_msg;
    state_msg.pheader.step = msg->step;
    state_msg.pheader.timestamp = msg->time;
    state_msg.data = {step_as_double};
    state_vector_pub_->publish(state_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Published references/state_vector for control step=%u reference=%f state=%f",
      msg->step,
      references_msg.data.front(),
      state_msg.data.front());
  }

  rclcpp::Publisher<FloatArrayMsg>::SharedPtr references_pub_;
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr state_vector_pub_;
  rclcpp::Subscription<StepClockMsg>::SharedPtr step_clock_sub_;
  rclcpp::Service<EmptySrv>::SharedPtr reset_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerInputSourceNode>());
  rclcpp::shutdown();
  return 0;
}
