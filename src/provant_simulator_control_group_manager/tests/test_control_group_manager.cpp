// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file test_control_group_manager.cpp
/// @brief This file contains the tests cases for a Control Group Manager that has a estate
/// estimator but does not have a disturbance generator.
///
/// @details The required nodes are the controller, reference generator and control inputs
/// zero order hold (ZOH).
///
/// @author Júnio Eduardo de Morais Aquino

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "provant_simulator_control_group_manager/control_group_manager.hpp"
#include "test_fixture.hpp"

class ControlGroupManagerTest : public ControlGroupTestFixture
{
protected:
  rclcpp::NodeOptions getOptions() override
  {
    rclcpp::NodeOptions opts;

    std::vector<rclcpp::Parameter> parameters;
    parameters.reserve(3);

    parameters.emplace_back("control_step", 0.003);
    parameters.emplace_back("has_disturbances", false);
    parameters.emplace_back("has_estimator", true);

    opts.parameter_overrides(parameters);

    std::vector<std::string> arguments;
    arguments.reserve(3);

    arguments.emplace_back("--ros-args");
    arguments.emplace_back("--log-level");
    arguments.emplace_back("debug");

    opts.arguments(arguments);

    return opts;
  }
};

TEST_F(ControlGroupManagerTest, InitializeParameters)  // NOLINT(cert-err58-cpp)
{
  ASSERT_FALSE(node->hasDisturbanceGenerator());
  ASSERT_TRUE(node->hasStateEstimator());

  auto const res = node->controlStep().to_chrono<std::chrono::nanoseconds>();
  auto const durationSecs = static_cast<double>(res.count()) * 1e-9;
  ASSERT_DOUBLE_EQ(durationSecs, 0.003);

  ASSERT_TRUE(node->disturbancesReady());

  ASSERT_FALSE(node->controlReady());

  auto const lastExec = node->lastExecutedOn();
  ASSERT_FALSE(lastExec.has_value());
}

TEST_F(ControlGroupManagerTest, RegistersControlGroup)  // NOLINT(cert-err58-cpp)
{
  using namespace std::chrono_literals;

  // Execute the nodes to register the publishers and subscribers.
  executor.spin_all(10s);
  // Wait a little for the registration to complete.
  std::this_thread::sleep_for(100ms);
  executor.spin_all(10s);

  // Check that the service was called
  ASSERT_EQ(testNode->registrationRequests.size(), 1);
  ASSERT_EQ(testNode->registrationResponses.size(), 1);

  // Verify that the request had the correct format
  auto const req = testNode->registrationRequests.front();
  ASSERT_EQ(req.group_namespace, "/");
}

TEST_F(ControlGroupManagerTest, PublishesTheCorrectMessages)  // NOLINT(cert-err58-cpp)
{
  using namespace std::chrono_literals;

  // Execute the nodes to register the publishers and subscribers.
  executor.spin_all(10s);

  // Check that there is at least one subscriber to the /provant_simulator/step_clock topic.
  EXPECT_GT(testNode->stepClockPub->get_subscription_count(), 0);

  testNode->publishStepClock(0, rclcpp::Time{0, 0, RCL_ROS_TIME});

  // Spin the nodes until the messages are published.
  executor.spin_all(10s);

  // Verify that the correct messages were published
  EXPECT_EQ(testNode->readyMsgs.msgs.size(), 0);
  EXPECT_EQ(testNode->zohTriggerMsgs.msgs.size(), 0);
  EXPECT_EQ(testNode->stepClockMsgs.msgs.size(), 1);

  // Publish a message with the control data
  const std::vector<double> ctrl = {0};
  testNode->publishControlInputs(ctrl);

  // Spin the nodes until the messages are published.
  executor.spin_all(10s);

  // Verify that a ready message was published
  EXPECT_EQ(testNode->zohTriggerMsgs.msgs.size(), 0);
  ASSERT_EQ(testNode->readyMsgs.msgs.size(), 1);

  // Verify that the last updated timestamp was updated
  ASSERT_TRUE(node->lastExecutedOn().has_value());
  const rclcpp::Time zeroTime{0, 0, RCL_ROS_TIME};
  EXPECT_EQ(node->lastExecutedOn().value(), zeroTime);

  // Verify that the ready message had the correct data
  auto const msg = *testNode->readyMsgs.msgs.begin();

  EXPECT_EQ(msg.pheader.step, 0);
  EXPECT_EQ(msg.pheader.timestamp.sec, 0);
  EXPECT_EQ(msg.pheader.timestamp.nanosec, 0);

  // In the second simulation, step the control group should update.
  // Therefore, it should immediately send a message on the ready topic, and on the zoh_trigger
  // topics.

  // Increase the simulation step and spin the nodes
  testNode->publishStepClock(1, 0.001);
  executor.spin_all(10s);

  EXPECT_EQ(testNode->stepClockMsgs.msgs.size(), 1);
  EXPECT_EQ(testNode->zohTriggerMsgs.msgs.size(), 1);
  EXPECT_EQ(testNode->readyMsgs.msgs.size(), 2);

  auto const secondMsg = testNode->readyMsgs.msgs.at(1);
  EXPECT_EQ(secondMsg.pheader.step, 1);

  const rclcpp::Time secondTime{0, 1000000, RCL_ROS_TIME};
  EXPECT_EQ(secondMsg.pheader.timestamp, secondTime);

  // Third step (step number 2)
  // The control group should not update
  testNode->publishStepClock(2, 0.001);
  executor.spin_all(10s);

  EXPECT_EQ(testNode->zohTriggerMsgs.msgs.size(), 2);
  EXPECT_EQ(testNode->readyMsgs.msgs.size(), 3);
  EXPECT_EQ(testNode->stepClockMsgs.msgs.size(), 1);

  // Fourth step (step number 3)
  // The control group should update
  testNode->publishStepClock(3, 0.001);
  executor.spin_all(10s);

  EXPECT_EQ(testNode->zohTriggerMsgs.msgs.size(), 2);
  EXPECT_EQ(testNode->readyMsgs.msgs.size(), 3);
  EXPECT_EQ(testNode->stepClockMsgs.msgs.size(), 2);

  // Check the published step clock message is correct
  auto const thirdMsg = testNode->stepClockMsgs.msgs.back();
  EXPECT_EQ(thirdMsg.step, 3);

  const rclcpp::Time thirdTime{0, 3000000, RCL_ROS_TIME};

  EXPECT_EQ(thirdMsg.time, thirdTime);

  // Verify that the last executed timestamp was updated
  ASSERT_TRUE(node->lastExecutedOn().has_value());
  EXPECT_EQ(node->lastExecutedOn().value(), thirdTime);
}

TEST_F(ControlGroupManagerTest, ResetsComponents)  // NOLINT(cert-err58-cpp)
{
  using namespace std::chrono_literals;

  // Execute the nodes to register the publishers and subscribers.
  this->executor.spin_all(10s);

  // Execute two steps
  testNode->publishStepClock(0, 0.0);
  this->executor.spin_all(10s);

  const std::vector<double> ctrl = {0};
  testNode->publishControlInputs(ctrl);
  executor.spin_all(10s);

  EXPECT_EQ(testNode->readyMsgs.msgs.size(), 1);
  EXPECT_TRUE(node->lastExecutedOn().has_value());

  // Check that the service is available
  std::thread task1{[this]() { ASSERT_TRUE(this->testNode->resetClient->wait_for_service(1s)); }};

  this->executor.spin_all(10s);
  task1.join();

  using EmptySrv = std_srvs::srv::Empty;
  auto req = std::make_shared<EmptySrv::Request>();

  auto call = testNode->resetClient->async_send_request(req);

  ASSERT_EQ(executor.spin_until_future_complete(call, 10s), rclcpp::FutureReturnCode::SUCCESS);

  // Verify that the component reset services is being called.
  ASSERT_EQ(testNode->controllerReset.requests.size(), 1);
  ASSERT_EQ(testNode->controlZohReset.requests.size(), 1);
  ASSERT_EQ(testNode->refGenReset.requests.size(), 1);
  ASSERT_EQ(testNode->estimatorReset.requests.size(), 1);

  ASSERT_EQ(testNode->distGenReset.requests.size(), 0);
  ASSERT_EQ(testNode->distZohReset.requests.size(), 0);

  // Verify that the internal state was reset
  EXPECT_FALSE(node->lastExecutedOn().has_value());
}
