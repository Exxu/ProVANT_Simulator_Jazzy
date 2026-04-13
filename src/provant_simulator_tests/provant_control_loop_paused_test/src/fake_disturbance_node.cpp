#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <provant_simulator_interfaces/msg/float64_array.hpp>
#include <provant_simulator_interfaces/msg/step_clock.hpp>

class FakeDisturbanceNode : public rclcpp::Node
{
public:
  FakeDisturbanceNode() : Node("fake_disturbance_node")
  {
    const auto topic_name = this->declare_parameter<std::string>("topic_name", "step_clock");
    sub_ = this->create_subscription<provant_simulator_interfaces::msg::StepClock>(
      topic_name, rclcpp::QoS(10),
      std::bind(&FakeDisturbanceNode::onStepClock, this, std::placeholders::_1));
    pub_ = this->create_publisher<provant_simulator_interfaces::msg::Float64Array>(
      "disturbances", rclcpp::QoS(10));
  }

private:
  void onStepClock(const provant_simulator_interfaces::msg::StepClock::SharedPtr msg)
  {
    provant_simulator_interfaces::msg::Float64Array out;
    out.pheader.step = msg->step;
    out.pheader.timestamp = msg->time;
    out.data = {0.0};
    pub_->publish(out);
  }

  rclcpp::Subscription<provant_simulator_interfaces::msg::StepClock>::SharedPtr sub_;
  rclcpp::Publisher<provant_simulator_interfaces::msg::Float64Array>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeDisturbanceNode>());
  rclcpp::shutdown();
  return 0;
}
