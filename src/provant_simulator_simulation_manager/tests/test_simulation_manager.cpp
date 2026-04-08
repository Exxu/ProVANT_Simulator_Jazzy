// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file test_simulation_manager.cpp
/// @brief This file contains the test cases for the simulation manager.
///
/// @author Júnio Eduardo de Morais Aquino

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "fake_control_group_node.hpp"
#include "provant_simulator_simulation_manager/simulation_manager.hpp"
#include "test_node.hpp"

using namespace std::chrono_literals;

class SimulationManagerTests : public ::testing::Test
{
public:
  /// @brief Initializes the default rclcpp context.
  [[maybe_unused]] static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  /// @brief Shutdown the default rclcpp context.
  [[maybe_unused]] static void TearDownTestCase() { rclcpp::shutdown(); }

protected:
  void SetUp() override
  {
    Test::SetUp();

    managerNode = std::make_shared<provant::SimulationManager>(rclcpp::NodeOptions{});
    testNode = std::make_shared<TestNode>(rclcpp::NodeOptions{}, "/provant_simulator/robot1");

    group1 = std::make_shared<FakeControlGroup>("/provant_simulator/fake_robot1");
    group2 = std::make_shared<FakeControlGroup>("/provant_simulator/fake_robot2");
    group3 = std::make_shared<FakeControlGroup>("/provant_simulator/fake_robot3");

    executor.add_node(managerNode);
    executor.add_node(testNode);
    executor.add_node(group1);
    executor.add_node(group2);
    executor.add_node(group3);
  }

  void spinExecutor() { executor.spin_all(10s); }

  using StrMsg = std_msgs::msg::String;

  std::shared_ptr<provant::SimulationManager> managerNode;
  std::shared_ptr<TestNode> testNode;
  rclcpp::executors::SingleThreadedExecutor executor{};

  using ControlGroupNodePtr = std::shared_ptr<FakeControlGroup>;
  ControlGroupNodePtr group1, group2, group3;

  void registerGroup(const ControlGroupNodePtr & group)
  {
    // Spin the nodes to register the connections
    spinExecutor();

    // Check that the registration service is available.
    ASSERT_TRUE(testNode->registerGroupClient->wait_for_service(100ms));

    // Call the service
    using RequestT = provant_simulator_interfaces::srv::ControlGroupRegister::Request;
    auto request = std::make_shared<RequestT>();
    request->group_namespace = group->get_namespace();

    auto call = testNode->registerGroupClient->async_send_request(request);
    spinExecutor();

    // Check that the response has the correct values
    ASSERT_TRUE(call.valid());
    ASSERT_EQ(call.wait_for(100ms), std::future_status::ready);

    auto const & response = call.get();
    EXPECT_TRUE(response->result);
    EXPECT_TRUE(response->message.empty());

    // Check that the group was registered, and it is not ready
    EXPECT_TRUE(managerNode->isGroupRegistered(group->get_namespace()));
    EXPECT_FALSE(managerNode->isGroupReady(group->get_namespace()));

    // Check that the simulation manager subscribed to the ready topic.
    EXPECT_EQ(group->readyPublisher->get_subscription_count(), 1);
  }

  void setSimulationState(const std::string & state)
  {
    spinExecutor();

    StrMsg msg{};
    msg.data = state;

    auto const prevMsgCount = testNode->simulationStateMsgs.messages().size();
    testNode->setSimulationStatePublisher->publish(msg);
    spinExecutor();

    ASSERT_EQ(testNode->simulationStateMsgs.messages().size(), prevMsgCount + 1);
  }
};

TEST_F(SimulationManagerTests, InitializesValues)  // NOLINT(cert-err58-cpp)
{
  // No control group should be registered.
  EXPECT_EQ(managerNode->registeredGroupCount(), 0);
}

TEST_F(SimulationManagerTests, SubscribesToSetSimulationState)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();

  EXPECT_EQ(testNode->setSimulationStatePublisher->get_subscription_count(), 1);
  EXPECT_EQ(testNode->simulationStateSubscription->get_publisher_count(), 1);
}

TEST_F(SimulationManagerTests, AdvertisesTheStepTopic)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();

  EXPECT_EQ(testNode->stepSubscription->get_publisher_count(), 1);
}

TEST_F(SimulationManagerTests, ProvidesResetServer)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();

  EXPECT_TRUE(testNode->resetServiceClient->wait_for_service(100ms));
}

TEST_F(SimulationManagerTests, CanRegisterGroup)  // NOLINT(cert-err58-cpp)
{
  // Spin the nodes to register the connections
  spinExecutor();

  // Check that the registration service is available.
  ASSERT_TRUE(testNode->registerGroupClient->wait_for_service(100ms));

  // Call the service
  using RequestT = provant_simulator_interfaces::srv::ControlGroupRegister::Request;
  auto request = std::make_shared<RequestT>();
  request->group_namespace = group1->get_namespace();
  EXPECT_EQ(request->group_namespace, "/provant_simulator/fake_robot1");

  auto call = testNode->registerGroupClient->async_send_request(request);
  spinExecutor();

  // Check that the response has the correct values
  ASSERT_TRUE(call.valid());
  ASSERT_EQ(call.wait_for(100ms), std::future_status::ready);

  auto const & response = call.get();
  EXPECT_TRUE(response->result);
  EXPECT_TRUE(response->message.empty());

  // Check that the group was registered, and it is not ready
  EXPECT_TRUE(managerNode->isGroupRegistered(group1->get_namespace()));
  EXPECT_FALSE(managerNode->isGroupReady(group1->get_namespace()));

  // Check that the simulation manager subscribed to the ready topic.
  EXPECT_EQ(
    group1->readyPublisher->get_topic_name(), std::string{"/provant_simulator/fake_robot1/ready"});
  EXPECT_EQ(group1->readyPublisher->get_subscription_count(), 1);
}

TEST_F(SimulationManagerTests, SetSimulationRun)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();

  StrMsg msg{};
  msg.data = "start";

  EXPECT_EQ(testNode->simulationStateMsgs.messages().size(), 0);
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::paused);

  testNode->setSimulationStatePublisher->publish(msg);
  spinExecutor();

  ASSERT_EQ(testNode->simulationStateMsgs.messages().size(), 1);
  auto const resMsg = testNode->simulationStateMsgs.messages().back();

  EXPECT_EQ(resMsg.data, std::string{"running"});
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::running);
}

TEST_F(SimulationManagerTests, SetSimulationStep)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();

  StrMsg msg{};
  msg.data = "step";

  EXPECT_EQ(testNode->simulationStateMsgs.messages().size(), 0);
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::paused);

  testNode->setSimulationStatePublisher->publish(msg);
  spinExecutor();

  ASSERT_EQ(testNode->simulationStateMsgs.messages().size(), 1);
  auto const resMsg = testNode->simulationStateMsgs.messages().back();

  EXPECT_EQ(resMsg.data, std::string{"stepping"});
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::stepping);
}

TEST_F(SimulationManagerTests, SetSimulationPause)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();

  StrMsg pauseMsg{};
  pauseMsg.data = "pause";

  EXPECT_EQ(testNode->simulationStateMsgs.messages().size(), 0);
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::paused);

  testNode->setSimulationStatePublisher->publish(pauseMsg);
  spinExecutor();

  EXPECT_EQ(testNode->simulationStateMsgs.messages().size(), 0);
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::paused);

  // Set the simulation to running
  StrMsg runningMsg{};
  runningMsg.data = "start";

  testNode->setSimulationStatePublisher->publish(runningMsg);
  spinExecutor();

  ASSERT_EQ(testNode->simulationStateMsgs.messages().size(), 1);
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::running);

  const auto firstResMsg = testNode->simulationStateMsgs.messages().back();
  EXPECT_EQ(firstResMsg.data, std::string{"running"});

  // Set the simulation to paused
  testNode->setSimulationStatePublisher->publish(pauseMsg);
  spinExecutor();

  EXPECT_EQ(testNode->simulationStateMsgs.messages().size(), 2);
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::paused);

  const auto secondResMsg = testNode->simulationStateMsgs.messages().back();
  EXPECT_EQ(secondResMsg.data, std::string{"paused"});
}

TEST_F(SimulationManagerTests, SetSimulationStop)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();

  StrMsg msg{};
  msg.data = "stop";

  EXPECT_EQ(testNode->simulationStateMsgs.messages().size(), 0);
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::paused);

  testNode->setSimulationStatePublisher->publish(msg);
  spinExecutor();

  ASSERT_EQ(testNode->simulationStateMsgs.messages().size(), 1);
  auto const resMsg = testNode->simulationStateMsgs.messages().back();

  EXPECT_EQ(resMsg.data, std::string{"stopped"});
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::stopped);
}

TEST_F(SimulationManagerTests, SetSimulationInvalidValue)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();

  StrMsg msg{};
  msg.data = "test";

  EXPECT_EQ(testNode->simulationStateMsgs.messages().size(), 0);
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::paused);

  testNode->setSimulationStatePublisher->publish(msg);
  spinExecutor();

  EXPECT_EQ(testNode->simulationStateMsgs.messages().size(), 0);
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::paused);
}

TEST_F(SimulationManagerTests, CanRegisterMultipleGroups)  // NOLINT(cert-err58-cpp)
{
  registerGroup(group1);
  registerGroup(group2);
  registerGroup(group3);

  EXPECT_EQ(managerNode->registeredGroupCount(), 3);
}

TEST_F(SimulationManagerTests, CannotRegisterRepeatedly)  // NOLINT(cert-err58-cpp)
{
  registerGroup(group1);

  using RequestT = provant_simulator_interfaces::srv::ControlGroupRegister::Request;
  auto request = std::make_shared<RequestT>();
  request->group_namespace = group1->get_namespace();

  auto call = testNode->registerGroupClient->async_send_request(request);
  spinExecutor();

  // Check that the response has the correct values
  ASSERT_TRUE(call.valid());
  ASSERT_EQ(call.wait_for(100ms), std::future_status::ready);

  auto const & response = call.get();
  EXPECT_FALSE(response->result);
  EXPECT_FALSE(response->message.empty());
}

TEST_F(SimulationManagerTests, CallRegisteredGroupsReset)  // NOLINT(cert-err58-cpp)
{
  // Register two groups.
  spinExecutor();
  registerGroup(group1);
  registerGroup(group2);

  // Set the status of group1 to ready
  group1->publishReady();
  spinExecutor();
  EXPECT_TRUE(managerNode->isGroupReady(group1->get_namespace()));

  // Set simulation state to running
  setSimulationState("start");
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::running);
  EXPECT_EQ(testNode->simulationStateMsgs.messages().size(), 1);

  auto const firstMsg = testNode->simulationStateMsgs.messages().back();
  EXPECT_EQ(firstMsg.data, std::string{"running"});

  // Check that the reset services had not been called
  EXPECT_EQ(group1->resetServiceRequests.size(), 0);
  EXPECT_EQ(group2->resetServiceRequests.size(), 0);
  EXPECT_EQ(group3->resetServiceRequests.size(), 0);

  // Reset the node
  auto resetReq = std::make_shared<std_srvs::srv::Empty::Request>();
  auto call = testNode->resetServiceClient->async_send_request(resetReq);
  spinExecutor();

  ASSERT_TRUE(call.valid());
  EXPECT_EQ(call.wait_for(100ms), std::future_status::ready);

  // Check that the reset service of the registered groups was called
  EXPECT_EQ(group1->resetServiceRequests.size(), 1);
  EXPECT_EQ(group2->resetServiceRequests.size(), 1);
  EXPECT_EQ(group3->resetServiceRequests.size(), 0);

  // Check that the status of the groups is set to false
  EXPECT_FALSE(managerNode->isGroupReady(group1->get_namespace()));
  EXPECT_FALSE(managerNode->isGroupReady(group2->get_namespace()));
  EXPECT_FALSE(managerNode->isGroupReady(group3->get_namespace()));

  // Check that the simulation state was set to resetting
  ASSERT_EQ(testNode->simulationStateMsgs.messages().size(), 3);
  auto const secondMsg = testNode->simulationStateMsgs.messages().at(1);

  EXPECT_EQ(secondMsg.data, std::string{"resetting"});

  // Check that the simulation state was set to paused
  auto const lastMsg = testNode->simulationStateMsgs.messages().back();
  EXPECT_EQ(lastMsg.data, std::string{"paused"});
}

TEST_F(SimulationManagerTests, SetsGroupStateToReady)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();
  registerGroup(group1);
  registerGroup(group2);

  // Check that the state of both groups is not ready
  EXPECT_FALSE(managerNode->isGroupReady(group1->get_namespace()));
  EXPECT_FALSE(managerNode->isGroupReady(group2->get_namespace()));

  // Publish a ready message from group2
  group2->publishReady();
  spinExecutor();

  // Check that the status of group2 was set to ready
  EXPECT_FALSE(managerNode->isGroupReady(group1->get_namespace()));
  EXPECT_TRUE(managerNode->isGroupReady(group2->get_namespace()));
}

TEST_F(SimulationManagerTests, PublishesStepWithOnlyOneGroup)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();
  registerGroup(group1);

  // Check that no ready message was received
  EXPECT_EQ(testNode->stepMsgs.messages().size(), 0);

  // Check that the state of both groups is not ready
  EXPECT_FALSE(managerNode->isGroupReady(group1->get_namespace()));
  EXPECT_EQ(managerNode->registeredGroupCount(), 1);

  // Set the simulation state to running
  setSimulationState("start");
  spinExecutor();
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::running);

  // Publish a ready message from group1
  group1->publishReady();
  spinExecutor();

  // Check that a message was received in the step topic
  EXPECT_EQ(testNode->stepMsgs.messages().size(), 1);

  // Check that the state of group 1 is not ready
  EXPECT_FALSE(managerNode->isGroupReady(group1->get_namespace()));
}

TEST_F(SimulationManagerTests, PublishesStepWhenRunning)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();
  registerGroup(group1);
  registerGroup(group2);

  // Check that no ready message was received
  EXPECT_EQ(testNode->stepMsgs.messages().size(), 0);

  // Check that the state of both groups is not ready
  EXPECT_FALSE(managerNode->isGroupReady(group1->get_namespace()));
  EXPECT_FALSE(managerNode->isGroupReady(group2->get_namespace()));
  EXPECT_EQ(managerNode->registeredGroupCount(), 2);

  // Set the simulation state to running
  setSimulationState("start");
  spinExecutor();
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::running);

  // Publish a ready message from group1
  group1->publishReady();
  spinExecutor();

  // Check that the status of the groups was updated
  EXPECT_TRUE(managerNode->isGroupReady(group1->get_namespace()));
  EXPECT_FALSE(managerNode->isGroupReady(group2->get_namespace()));

  // Check that a message was not received in the step topic
  EXPECT_EQ(testNode->stepMsgs.messages().size(), 0);

  // Publish a ready message from group2
  group2->publishReady();
  spinExecutor();

  // Check that a message was received in the step topic
  EXPECT_EQ(testNode->stepMsgs.messages().size(), 1);

  // Check that the simulation state is still running
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::running);

  // Check that the status of the nodes was updated
  EXPECT_FALSE(managerNode->isGroupReady(group1->get_namespace()));
  EXPECT_FALSE(managerNode->isGroupReady(group2->get_namespace()));
}

TEST_F(SimulationManagerTests, DontPublishesStepWhenStepping)  // NOLINT(cert-err58-cpp)
{
  spinExecutor();
  registerGroup(group1);
  registerGroup(group2);

  // Check that no ready message was received
  EXPECT_EQ(testNode->stepMsgs.messages().size(), 0);

  // Check that the state of both groups is not ready
  EXPECT_FALSE(managerNode->isGroupReady(group1->get_namespace()));
  EXPECT_FALSE(managerNode->isGroupReady(group2->get_namespace()));
  EXPECT_EQ(managerNode->registeredGroupCount(), 2);

  // Set the simulation state to running
  setSimulationState("step");
  spinExecutor();
  ASSERT_EQ(managerNode->simulationState(), provant::SimulationState::stepping);

  // Publish a ready message from group2
  group2->publishReady();
  spinExecutor();

  // Check that the status of the groups was updated
  EXPECT_FALSE(managerNode->isGroupReady(group1->get_namespace()));
  EXPECT_TRUE(managerNode->isGroupReady(group2->get_namespace()));

  // Check that a message was not received in the step topic
  EXPECT_EQ(testNode->stepMsgs.messages().size(), 0);

  // Publish a ready message from group1
  group1->publishReady();
  spinExecutor();

  // Check that a message was not received in the step topic
  EXPECT_EQ(testNode->stepMsgs.messages().size(), 0);

  // Check that the simulation state was updated to paused
  EXPECT_EQ(managerNode->simulationState(), provant::SimulationState::paused);

  // Check that the status of the nodes was updated
  EXPECT_FALSE(managerNode->isGroupReady(group1->get_namespace()));
  EXPECT_FALSE(managerNode->isGroupReady(group2->get_namespace()));
}
