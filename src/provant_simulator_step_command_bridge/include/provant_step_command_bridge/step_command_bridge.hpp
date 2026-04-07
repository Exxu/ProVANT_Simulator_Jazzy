#ifndef PROVANT_STEP_COMMAND_BRIDGE__STEP_COMMAND_BRIDGE_HPP_
#define PROVANT_STEP_COMMAND_BRIDGE__STEP_COMMAND_BRIDGE_HPP_

#include <atomic>
#include <string>

#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

namespace provant
{

class StepCommandBridge : public rclcpp::Node
{
public:
  StepCommandBridge();

private:
  void onStepMsg(const std_msgs::msg::Empty::SharedPtr msg);

  std::string worldName_;
  std::string stepTopic_;
  std::string controlService_;
  int timeoutMs_{3000};

  gz::transport::Node gzNode_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stepSub_;
  std::atomic<bool> requestInFlight_{false};
};

}  // namespace provant

#endif  // PROVANT_STEP_COMMAND_BRIDGE__STEP_COMMAND_BRIDGE_HPP_