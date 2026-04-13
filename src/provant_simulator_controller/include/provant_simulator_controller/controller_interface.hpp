// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file controller_interface.hpp
/// @brief This file contains the declaration of the controller interface class.
///
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_CONTROLLER__CONTROLLER_INTERFACE_HPP_
#define PROVANT_SIMULATOR_CONTROLLER__CONTROLLER_INTERFACE_HPP_

#include <vector>

namespace provant
{
/// @brief Interface for the implementation of control strategies.
/// @details This class contains the public interface that must be followed by controllers developed
/// for use with the ProVANT Simulator.
/// The user must override the execute, getReferences, getStateVector, and getErrorVector methods
/// and implement the desired functionality for their control strategies.
/// Additionally, the config method can be overridden to initialize the resources used by the
/// control law.
class IController
{
public:
  IController() = default;
  virtual ~IController() = default;

  // Forbid copies and moves
  IController(const IController & other) = delete;
  IController(IController && other) = delete;
  IController & operator=(const IController & other) = delete;
  IController & operator=(IController && other) = delete;

  // Define the public default operations of the class
  /// @brief Set up the controller for execution.
  /// @return True if configuration is successful and false otherwise.
  [[nodiscard]] bool setUp();
  /// @brief Compute the control signals given the state vector and references.
  /// @param states Current state vector of the controlled system.
  /// @param references Current references to be tracked by the controlled system.
  /// @return Vector of control inputs to be applied to the controlled system. Must be at the same
  /// order as specified to the controller zero order hold (ZOH).
  [[nodiscard]] std::vector<double> compute(
    const std::vector<double> & states, const std::vector<double> & references);
  /// @brief Reset the control strategy to its initial state.
  void resetControlStrategy();
  /// @brief Get the reference vector used for the control law computation.
  /// @details This method is exposed to allow control strategies to include a reference signal
  /// generation to report the currently used references.
  /// @return References to be tracked by the controlled system.
  [[nodiscard]] std::vector<double> references() const;
  /// @brief Get the state vector used in the control law computation.
  /// @return The system state vector.
  [[nodiscard]] std::vector<double> stateVector() const;
  /// @brief Get the current error vector.
  /// @return Current error vector.
  [[nodiscard]] std::vector<double> errorVector() const;

protected:
  /// @brief Configure the controller for the computation of the control law.
  /// @details This method will be executed once when the controller interface object is
  /// constructed.
  /// It is intended to be used to initialize any resources used by the control strategy, such as
  /// reading gain values from an input file, creating threads, or initializing other matrices
  /// used in the computation.
  /// @return True if configuration is successful and false otherwise.
  [[nodiscard]] virtual bool config();
  /// @brief Execute the control law given the state and reference vectors.
  /// @param states Current state vector of the controlled system.
  /// @param references References to be tracked by the controlled system.
  /// @return Control signals to apply to the controlled system.
  [[nodiscard]] virtual std::vector<double> execute(
    const std::vector<double> & states, const std::vector<double> & references) = 0;

  /// @brief Reset the control strategy to its initial state.
  /// @details Must reset any internal variables of the control strategy to the values they had
  /// when the control strategy was constructed.
  virtual void reset() = 0;

  /// @brief Get the current error vector.
  /// @return Current error vector.
  [[nodiscard]] virtual std::vector<double> getErrorVector() const = 0;
  /// @brief Get the reference vector used in the control law computation.
  /// @return Reference vector used in the control law computation.
  [[nodiscard]] virtual std::vector<double> getReferences() const = 0;
  /// @brief Get the state vector used in the control law computation.
  /// @return State vector used in the control law computation.
  [[nodiscard]] virtual std::vector<double> getStateVector() const = 0;
};
}  // namespace provant

#endif  // PROVANT_SIMULATOR_CONTROLLER__CONTROLLER_INTERFACE_HPP_
