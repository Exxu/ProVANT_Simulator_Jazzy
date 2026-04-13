// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file test_controller.cpp
/// @brief This file contains the test cases for the controller node.
///
/// @author Júnio Eduardo de Morais Aquino

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "mock_control_strategy.hpp"
#include "provant_simulator_controller/controller.hpp"
#include "test_controller_node.hpp"

using namespace std::chrono_literals;

class ControllerTests : public ::testing::Test
{
public:
  [[maybe_unused]] static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  [[maybe_unused]] static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  void SetUp() override
  {
    Test::SetUp();

    using ::testing::Return;

    auto controlStrategy = std::make_unique<provant::MockControlStrategy>();
    ON_CALL(*controlStrategy, configImpl()).WillByDefault(Return(true));

    controllerNode =
      std::make_shared<provant::Controller>(rclcpp::NodeOptions{}, std::move(controlStrategy));
    testNode = std::make_shared<TestControllerNode>();

    executor.add_node(controllerNode);
    executor.add_node(testNode);
  }

  void TearDown() override { Test::TearDown(); }

  std::shared_ptr<TestControllerNode> testNode;
  std::shared_ptr<provant::Controller> controllerNode;
  rclcpp::executors::SingleThreadedExecutor executor;
};

TEST_F(ControllerTests, InitializesValues)
{
  EXPECT_FALSE(this->controllerNode->stateVectorReady());
  EXPECT_FALSE(this->controllerNode->referencesReady());

  executor.spin_all(10s);
  EXPECT_FALSE(this->controllerNode->stateVectorReady());
  EXPECT_FALSE(this->controllerNode->referencesReady());

  EXPECT_EQ(this->testNode->referencesPublisher->get_subscription_count(), 1);
  EXPECT_EQ(this->testNode->stateVectorPublisher->get_subscription_count(), 1);
}

TEST_F(ControllerTests, UpdatesReferences)
{
  executor.spin_all(10s);

  EXPECT_EQ(this->testNode->referencesPublisher->get_subscription_count(), 1);

  // Verify that the references state was not ready
  EXPECT_FALSE(this->controllerNode->referencesReady());

  // Publish a references message and verify that the state was correctly updated
  const std::vector<double> data = {0};
  testNode->publishReferences(data);
  executor.spin_all(10s);

  EXPECT_TRUE(this->controllerNode->referencesReady());
}

TEST_F(ControllerTests, UpdatesStateVector)
{
  executor.spin_all(10s);

  EXPECT_EQ(this->testNode->stateVectorPublisher->get_subscription_count(), 1);

  EXPECT_FALSE(this->controllerNode->stateVectorReady());

  const std::vector<double> data = {0};
  testNode->publishStateVector(data);
  executor.spin_all(10s);

  EXPECT_TRUE(this->controllerNode->stateVectorReady());
}

TEST_F(ControllerTests, CanReset)
{
  // Verify that the controller node provides a server for the reset service.
  executor.spin_all(10s);
  EXPECT_TRUE(testNode->resetServiceClient->wait_for_service(1s));

  // Publish a state vector message to update its state
  const std::vector<double> data = {0};
  testNode->publishStateVector(data);
  executor.spin_all(10s);

  EXPECT_TRUE(controllerNode->stateVectorReady());

  // Call the reset service
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto call = testNode->resetServiceClient->async_send_request(request);
  ASSERT_TRUE(call.valid());

  executor.spin_all(10s);

  ASSERT_EQ(call.wait_for(100ms), std::future_status::ready);
  EXPECT_FALSE(controllerNode->stateVectorReady());

  // Publish a references message to update its state
  testNode->publishReferences(data);
  executor.spin_all(10s);

  EXPECT_TRUE(controllerNode->referencesReady());

  // Call the reset service
  auto secondCall = testNode->resetServiceClient->async_send_request(request);
  ASSERT_TRUE(secondCall.valid());

  executor.spin_all(10s);

  ASSERT_EQ(call.wait_for(100ms), std::future_status::ready);
  EXPECT_FALSE(controllerNode->referencesReady());
}

TEST(ControlStrategyTest, CallsControlStrategyReset)
{
  using ::testing::Exactly;
  using ::testing::Return;

  auto strategy = std::make_unique<provant::MockControlStrategy>();
  EXPECT_CALL(*strategy, resetImpl()).Times(Exactly(1));
  ON_CALL(*strategy, configImpl()).WillByDefault(Return(true));

  rclcpp::init(0, nullptr);

  auto controllerNode =
    std::make_shared<provant::Controller>(rclcpp::NodeOptions{}, std::move(strategy));
  auto testNode = std::make_shared<TestControllerNode>();
  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(controllerNode);
  executor.add_node(testNode);

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto call = testNode->resetServiceClient->async_send_request(request);

  ASSERT_TRUE(call.valid());

  executor.spin_all(10s);

  rclcpp::shutdown();
}

TEST(ControlStrategyTest, ExecuteControlStrategy)
{
  using ::testing::_;
  using ::testing::Exactly;
  using ::testing::Return;

  const std::vector<double> arg = {0.0};
  std::vector<double> firstRet = {100.0};
  std::vector<double> secondRet = {200.0};

  std::vector<double> errorRet = {3.0};
  std::vector<double> refRet = {4.1};
  std::vector<double> stateRet = {4.2};

  auto strategy = std::make_unique<provant::MockControlStrategy>();
  EXPECT_CALL(*strategy, executeImpl(_, _)).WillOnce(Return(firstRet)).WillOnce(Return(secondRet));
  ON_CALL(*strategy, configImpl()).WillByDefault(Return(true));

  EXPECT_CALL(*strategy, getErrorVectorImpl()).WillRepeatedly(Return(errorRet));
  EXPECT_CALL(*strategy, getReferencesImpl()).WillRepeatedly(Return(refRet));
  EXPECT_CALL(*strategy, getStateVectorImpl()).WillRepeatedly(Return(stateRet));

  rclcpp::init(0, nullptr);
  auto controllerNode =
    std::make_shared<provant::Controller>(rclcpp::NodeOptions{}, std::move(strategy));
  auto testNode = std::make_shared<TestControllerNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controllerNode);
  executor.add_node(testNode);
  executor.spin_all(10s);

  EXPECT_FALSE(controllerNode->referencesReady());
  EXPECT_FALSE(controllerNode->stateVectorReady());

  testNode->publishReferences(arg);
  executor.spin_all(10s);

  EXPECT_TRUE(controllerNode->referencesReady());
  EXPECT_FALSE(controllerNode->stateVectorReady());
  EXPECT_EQ(testNode->controlInputsMsgs.size(), 0);

  testNode->publishStateVector(arg);
  executor.spin_all(10s);

  // Verify that the first message is correct
  ASSERT_EQ(testNode->controlInputsMsgs.size(), 1);
  auto const firstMsg = testNode->controlInputsMsgs.front();

  EXPECT_EQ(testNode->refMsgs.size(), 1);
  EXPECT_EQ(testNode->stateVecMsgs.size(), 1);
  EXPECT_EQ(testNode->errorVecMsgs.size(), 1);

  EXPECT_EQ(firstMsg.pheader.step, 0);
  const rclcpp::Time zeroTime{0, 0, RCL_ROS_TIME};
  EXPECT_EQ(firstMsg.pheader.timestamp, zeroTime);
  EXPECT_EQ(firstMsg.data, firstRet);

  // Verify that the state was updated
  EXPECT_FALSE(controllerNode->referencesReady());
  EXPECT_FALSE(controllerNode->stateVectorReady());

  // Publish a state vector message and verify that the state is updated
  const rclcpp::Time oneTime{1, 0, RCL_ROS_TIME};
  testNode->publishStateVector(arg, 1, oneTime);
  executor.spin_all(10s);

  EXPECT_TRUE(controllerNode->stateVectorReady());
  EXPECT_FALSE(controllerNode->referencesReady());
  EXPECT_EQ(testNode->controlInputsMsgs.size(), 1);

  // Publish the reference message
  testNode->publishReferences(arg, 1, oneTime);
  executor.spin_all(10s);

  EXPECT_FALSE(controllerNode->stateVectorReady());
  EXPECT_FALSE(controllerNode->referencesReady());
  EXPECT_EQ(testNode->controlInputsMsgs.size(), 2);

  // Verify that the second message is correct
  auto const secondMsg = testNode->controlInputsMsgs.back();

  EXPECT_EQ(secondMsg.pheader.step, 1);
  EXPECT_EQ(secondMsg.pheader.timestamp, oneTime);
  EXPECT_EQ(secondMsg.data, secondRet);

  EXPECT_EQ(testNode->refMsgs.size(), 2);
  EXPECT_EQ(testNode->stateVecMsgs.size(), 2);
  EXPECT_EQ(testNode->errorVecMsgs.size(), 2);

  // Verify that the messages are correct
  EXPECT_EQ(testNode->refMsgs.back().pheader.step, 1);
  EXPECT_EQ(testNode->refMsgs.back().pheader.timestamp, oneTime);
  EXPECT_EQ(testNode->refMsgs.back().data, refRet);

  EXPECT_EQ(testNode->stateVecMsgs.back().pheader.step, 1);
  EXPECT_EQ(testNode->stateVecMsgs.back().pheader.timestamp, oneTime);
  EXPECT_EQ(testNode->stateVecMsgs.back().data, stateRet);

  EXPECT_EQ(testNode->errorVecMsgs.back().pheader.step, 1);
  EXPECT_EQ(testNode->errorVecMsgs.back().pheader.timestamp, oneTime);
  EXPECT_EQ(testNode->errorVecMsgs.back().data, errorRet);

  rclcpp::shutdown();
}

TEST(ControlStrategyTest, ThrowsOnConfigError)
{
  using ::testing::Return;

  auto strategy = std::make_unique<provant::MockControlStrategy>();
  EXPECT_CALL(*strategy, configImpl()).WillOnce(Return(false));

  rclcpp::init(0, nullptr);
  EXPECT_THROW(
    std::make_shared<provant::Controller>(rclcpp::NodeOptions{}, std::move(strategy)),
    std::runtime_error);

  rclcpp::shutdown();
}

class FakeControlStrategy : public provant::IController
{
protected:
  std::vector<double> execute(
    const std::vector<double> & states, const std::vector<double> & references) override
  {
    (void)states;
    (void)references;
    return {};
  }
  void reset() override {}
  [[nodiscard]] std::vector<double> getErrorVector() const override { return {}; }
  [[nodiscard]] std::vector<double> getReferences() const override { return {}; }
  [[nodiscard]] std::vector<double> getStateVector() const override { return {}; }
};

TEST(ControlStrategyTest, ConfigReturnsTrueByDefault)
{
  FakeControlStrategy strategy;
  ASSERT_TRUE(strategy.setUp());
}
