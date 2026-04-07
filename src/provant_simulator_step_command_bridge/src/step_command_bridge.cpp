// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

#include "provant_step_command_bridge/step_command_bridge.hpp"

#include <functional>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

namespace provant
{

StepCommandBridge::StepCommandBridge()
: Node("provant_step_command_bridge")
{
  worldName_ = this->declare_parameter<std::string>("world_name", "default");
  timeoutMs_ = this->declare_parameter<int>("timeout_ms", 3000);

  stepTopic_ = "/provant_simulator/step";
  controlService_ = "/world/" + worldName_ + "/control";

  stepSub_ = this->create_subscription<std_msgs::msg::Empty>(
    stepTopic_,
    rclcpp::QoS(10),
    std::bind(&StepCommandBridge::onStepMsg, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "StepCommandBridge started. step_topic='%s', world_name='%s', control_service='%s', timeout_ms=%d",
    stepTopic_.c_str(),
    worldName_.c_str(),
    controlService_.c_str(),
    timeoutMs_);
}

void StepCommandBridge::onStepMsg(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  if (requestInFlight_.exchange(true)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Received a step request while another step request is still in flight. Ignoring.");
    return;
  }

  gz::msgs::WorldControl req;
  gz::msgs::Boolean rep;
  bool result = false;

  // Preserve discrete stepping semantics:
  // - keep the simulation paused
  // - execute exactly one step
  req.set_pause(true);
  req.set_multi_step(1);

  const bool executed =
    gzNode_.Request(controlService_, req, static_cast<unsigned int>(timeoutMs_), rep, result);

  if (!executed) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to call Gazebo control service '%s' within timeout.",
      controlService_.c_str());
    requestInFlight_.store(false);
    return;
  }

  if (!result || !rep.data()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Gazebo control service '%s' returned failure. transport_result=%s response=%s",
      controlService_.c_str(),
      result ? "true" : "false",
      rep.data() ? "true" : "false");
    requestInFlight_.store(false);
    return;
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "Successfully requested one simulation step on '%s'.",
    controlService_.c_str());

  requestInFlight_.store(false);
}

}  // namespace provant