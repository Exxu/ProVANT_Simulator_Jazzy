// This file is part of the ProVANT simulator project.
// Licensed under the terms of the MIT open source license. More details at
// https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
// Copyright 2022 Guilherme V. Raffo
/// @file controller.hpp
/// @brief This file contains the declaration of the ProVANT Simulator controller class.
///
/// @author Júnio Eduardo de Morais Aquino

#ifndef PROVANT_SIMULATOR_CONTROLLER__CONTROLLER_HPP_
#define PROVANT_SIMULATOR_CONTROLLER__CONTROLLER_HPP_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <provant_simulator_controller/controller_interface.hpp>
#include <provant_simulator_interfaces/msg/float64_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <thread>
#include <vector>

namespace provant
{
/// @brief The Controller class implements a ROS node that executes a control strategy and publishes
/// the resulting signals to apply to the controlled system.
class Controller : public rclcpp::Node
{
public:
  /// @brief Type of message used to receive and publish arrays of data.
  using FloatArrayMsg = provant_simulator_interfaces::msg::Float64Array;
  /// @brief Type of service used by the reset service.
  using EmptySrv = std_srvs::srv::Empty;

  /// @brief Constructs a new Controller node.
  /// @param opts Node options for the controller. Can be used to specify arguments, override the
  /// namespace, topic mappings and etc.
  /// @param controlStrategy The control strategy object applied by the controller.
  explicit Controller(
    const rclcpp::NodeOptions & opts, std::unique_ptr<IController> controlStrategy);

  /// @brief Check if the references has already been received in the current step.
  /// @return True if the references has been received and false otherwise.
  bool referencesReady() const;

  /// @brief Check if the state vector has already been received in the current step.
  /// @return True if the state vector has been received and false otherwise.
  bool stateVectorReady() const;

private:
  /// @brief Control strategy used in the controller.
  std::unique_ptr<IController> _controlStrategy;

  /// @brief The clock type used to measure the control law computation time.
  using Clock = std::chrono::high_resolution_clock;

  /// @brief A ROS publishers for the resulting control inputs.
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr _controlInputsPub;
  /// @brief A ROS publisher for the references used in the control law computation.
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr _referencesPublisher;
  /// @brief A ROS publisher for the state vector used in the control law computation.
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr _stateVectorPublisher;
  /// @brief A ROS publisher for the error vector.
  rclcpp::Publisher<FloatArrayMsg>::SharedPtr _errorPublisher;
  /// @brief A ROS subscription to receive the state vector.
  rclcpp::Subscription<FloatArrayMsg>::SharedPtr _stateVecSub;
  /// @brief A ROS subscription to receive the references.
  rclcpp::Subscription<FloatArrayMsg>::SharedPtr _referencesSub;

  /// @brief Mutex to protect access to the _header, _stateVector and _references members.
  mutable std::mutex _updateMutex;

  /// @brief Stores the message header for usage when publishing the control law execution results.
  std::optional<provant_simulator_interfaces::msg::Header> _header;
  /// @brief The latest state vector.
  /// @details If this property has a value, it indicates that the state vector is ready for
  /// computation of the control law. And if it doesn't, it indicates that the state vector wasn't
  /// received yet.
  std::optional<std::vector<double>> _stateVector;
  /// @brief The latest references to be tracked by the controlled system.
  /// @details If this property has a value, it indicates that the references are ready for
  /// computation of the control law. And if it doesn't, it indicates that the references wasn't
  /// received yet.
  std::optional<std::vector<double>> _references;

  /// @brief A ROS server that allows the controller node to be reset.
  rclcpp::Service<EmptySrv>::SharedPtr _resetServiceClient;
  /// @brief Reset the controller and the control strategy to their initial state.
  /// @param req Request that originated the call.
  /// @param res Result of the request service execution.
  void onResetCall();

  /// @brief Update the state vector data, and if ready, execute the control strategy.
  /// @param msg Message containing the state vector data.
  void onStateVectorMessage(FloatArrayMsg::UniquePtr msg);

  /// @brief Update the references value, and if ready, execute the control strategy.
  /// @param msg Message containing the references.
  void onReferencesMessage(FloatArrayMsg::UniquePtr msg);

  /// @brief Verify if the references and state vector data have already been received, and if so
  /// execute the control strategy.
  void checkControlLaw();

  /// @brief Execute the control strategy and publish the control inputs.
  void executeControlLaw();
};
}  // namespace provant

#endif  // PROVANT_SIMULATOR_CONTROLLER__CONTROLLER_HPP_
