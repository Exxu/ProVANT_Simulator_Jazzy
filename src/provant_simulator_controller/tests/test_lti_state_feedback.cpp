// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file test_lti_state_feedback.cpp
/// @brief This file contains the tests cases for the LTIStateFeedbackController class.
///
/// @author Júnio Eduardo de Morais Aquino

#include <gtest/gtest.h>

#include <vector>

#include "provant_simulator_controller/lti_state_feedback_controller.hpp"

class TestControlStrategy : public provant::LTIStateFeedbackController
{
protected:
  [[nodiscard]] Eigen::MatrixXd gainMatrix() const override
  {
    return Eigen::Matrix<double, 2, 2>::Identity();
  }

  [[nodiscard]] Eigen::VectorXd stateEquilibrium() const override
  {
    Eigen::VectorXd x0{2};
    x0(0) = 1.0;
    x0(1) = 2.0;
    return x0;
  }

  [[nodiscard]] Eigen::VectorXd controlInputsEquilibrium() const override
  {
    Eigen::VectorXd u0{2};
    u0(0) = 3.0;
    u0(1) = 4.0;
    return u0;
  }
};

TEST(LTIControllerTest, CalculationsAreCorrect)
{
  const std::vector<double> states = {10, 20};
  const std::vector<double> ref = {5, 30};

  TestControlStrategy strategy{};
  EXPECT_TRUE(strategy.setUp());

  auto const x0 = strategy.getStateEquilibrium();
  ASSERT_NO_THROW(strategy.resetControlStrategy());

  EXPECT_DOUBLE_EQ(x0(0), 1.0);
  EXPECT_DOUBLE_EQ(x0(1), 2.0);

  auto const u0 = strategy.getControlInputsEquilibrium();
  EXPECT_DOUBLE_EQ(u0(0), 3.0);
  EXPECT_DOUBLE_EQ(u0(1), 4.0);

  auto const gains = strategy.getGainMatrix();
  EXPECT_DOUBLE_EQ(gains(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(gains(1, 1), 1.0);
  EXPECT_DOUBLE_EQ(gains(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(gains(1, 0), 0.0);

  auto const res = strategy.compute(states, ref);
  ASSERT_EQ(res.size(), 2);
  EXPECT_DOUBLE_EQ(res.at(0), 7.0);
  EXPECT_DOUBLE_EQ(res.at(1), -8.0);

  auto const x = strategy.stateVector();
  ASSERT_EQ(x.size(), 2);
  EXPECT_DOUBLE_EQ(x.at(0), 9.0);
  EXPECT_DOUBLE_EQ(x.at(1), 18.0);

  auto err = strategy.errorVector();
  ASSERT_EQ(err.size(), 2);
  EXPECT_DOUBLE_EQ(err.at(0), 4.0);
  EXPECT_DOUBLE_EQ(err.at(1), -12.0);

  auto const cRef = strategy.references();
  ASSERT_EQ(cRef.size(), 2);
  EXPECT_DOUBLE_EQ(cRef.at(0), 5.0);
  EXPECT_DOUBLE_EQ(cRef.at(1), 30.0);
}
