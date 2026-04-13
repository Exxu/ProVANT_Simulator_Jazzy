// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file lti_state_feedback_controller.hpp
/// @brief This file contains the declaration of the LTIStateFeedbackController class.
///
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_CONTROLLER__LTI_STATE_FEEDBACK_CONTROLLER_HPP_
#define PROVANT_SIMULATOR_CONTROLLER__LTI_STATE_FEEDBACK_CONTROLLER_HPP_

#include <Eigen/Eigen>
#include <vector>

#include "controller_interface.hpp"

namespace provant
{
/// @brief The LTI state feedback controller is an implementation of a full state feedback
/// controller for linear or linearized systems.
/// @details This class allows the user to specify an equilibrium point, and constant gain matrix
/// for calculation of a control law for a LTI system.
class LTIStateFeedbackController : public IController
{
public:
  /// @brief Get the constant matrix used to calculate the control inputs.
  /// @return Constant gain matrix.
  const Eigen::MatrixXd & getGainMatrix() const;
  /// @brief Get the equilibrium point in which the controlled model was trimmed.
  /// @return State vector equilibrium point.
  const Eigen::VectorXd & getStateEquilibrium() const;
  /// @brief Get the control inputs values at the equilibrium point.
  /// @return Control inputs equilibrium point.
  const Eigen::VectorXd & getControlInputsEquilibrium() const;

protected:
  bool config() override;

  /// @brief Execute the control law.
  /// @param states Current state vector of the controlled system.
  /// @param references References to be tracked by the controlled system.
  /// @return Control inputs to apply to the controlled system.
  std::vector<double> execute(
    const std::vector<double> & states, const std::vector<double> & references) final;

  /// @brief Reset the internal state of the control strategy.
  void reset() override;

  /// @brief The references used in the control law computation.
  /// @return References that must be tracked by the controlled system.
  [[nodiscard]] std::vector<double> getReferences() const final;
  /// @brief The state vector used in the control law computation.
  /// @return State vector minus the state equilibrium point.
  [[nodiscard]] std::vector<double> getStateVector() const final;
  /// @brief The error vector used in the control law computation.
  /// @return State vector minus the state equilibrium point minus references.
  [[nodiscard]] std::vector<double> getErrorVector() const final;

  /// @brief The constant matrix used to compute the control signals.
  [[nodiscard]] virtual Eigen::MatrixXd gainMatrix() const = 0;
  /// @brief The state equilibrium point in which the model was trimmed.
  /// @return Vector of state equilibrium point.
  [[nodiscard]] virtual Eigen::VectorXd stateEquilibrium() const = 0;
  /// @brief Return the control inputs value on the point in which the model is trimmed.
  /// @return Vector of control inputs.
  [[nodiscard]] virtual Eigen::VectorXd controlInputsEquilibrium() const = 0;
  /// @brief Obtain the error vector for use in the control law computation.
  /// @details If desired, the user can override this method to augment the error vector.
  /// @return Error vector for usage wit the control law computation.
  [[nodiscard]] virtual Eigen::VectorXd calculateErrorVector(const Eigen::VectorXd & error);

private:
  /// @brief The gain matrix used to compute the control signals.
  Eigen::MatrixXd _gains;
  /// @brief The state equilibrium point where the model was trimmed.
  Eigen::VectorXd _stateEquilibrium;
  /// @brief The control inputs obtained during model trimming.
  Eigen::VectorXd _controlInputsEquilibrium;

  /// @brief The references used to compute the control law.
  Eigen::VectorXd _references;
  /// @brief The state vector used to compute the control law.
  Eigen::VectorXd _states;
  /// @brief The error vector used to compute the control law.
  Eigen::VectorXd _error;
};
}  // namespace provant

#endif  // PROVANT_SIMULATOR_CONTROLLER__LTI_STATE_FEEDBACK_CONTROLLER_HPP_
