#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <provant_simulator_interfaces/msg/float64_array.hpp>
#include <provant_simulator_interfaces/msg/step_clock.hpp>
#include <rclcpp/rclcpp.hpp>

class StateVectorSourceNode : public rclcpp::Node
{
public:
  using FloatArrayMsg = provant_simulator_interfaces::msg::Float64Array;
  using StepClockMsg = provant_simulator_interfaces::msg::StepClock;

  StateVectorSourceNode()
  : rclcpp::Node("state_vector_source_node")
  {
    const auto step_clock_topic = this->declare_parameter<std::string>("step_clock_topic", "step_clock");
    state_size_ = this->declare_parameter<int>("state_size", 12);

    if (state_size_ <= 0) {
      throw std::runtime_error("The 'state_size' parameter must be greater than zero.");
    }

    state_vector_pub_ = this->create_publisher<FloatArrayMsg>("state_vector", rclcpp::QoS(10));
    step_clock_sub_ = this->create_subscription<StepClockMsg>(
      step_clock_topic,
      rclcpp::QoS(10),
      std::bind(&StateVectorSourceNode::onStepClock, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "State vector source ready. step_clock_topic='%s' state_size=%d",
      step_clock_topic.c_str(),
      state_size_);
  }

private:
  void onStepClock(const StepClockMsg::SharedPtr msg)
  {
    FloatArrayMsg state_msg;
    state_msg.pheader.step = msg->step;
    state_msg.pheader.timestamp = msg->time;
    state_msg.data.assign(static_cast<std::size_t>(state_size_), 0.0);
    state_vector_pub_->publish(state_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Published zero state_vector for control step=%u size=%zu",
      msg->step,
      state_msg.data.size());
  }

  int state_size_{12};
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr state_vector_pub_;
  rclcpp::Subscription<StepClockMsg>::SharedPtr step_clock_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateVectorSourceNode>());
  rclcpp::shutdown();
  return 0;
}
