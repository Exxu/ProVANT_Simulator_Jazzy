/*
* This file is part of the ProVANT simulator project.
* Licensed under the terms of the MIT open source license. More details at
* https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
* Copyright 2022 Guilherme V. Raffo
*/
/**
* @file joystick_reference_generator.hpp
* @brief This file contains the declaration of the JoystickReferenceGenerator 
* class.
*
* @author Lorena Oliveira
* @author Guilherme Barbosa
*/

#ifndef PROVANT_SIMULATOR_REFERENCE_GENERATOR__REFERENCE_GENERATOR_HPP_
#define PROVANT_SIMULATOR_REFERENCE_GENERATOR__REFERENCE_GENERATOR_HPP_

#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <vector>
#include <sensor_msgs/msg/joy.hpp>

#include "provant_simulator_reference_generator/base_reference_generator.hpp"

namespace provant::ref_gen
{
class JoystickReferenceGenerator : public BaseReferenceGenerator
{
public:
  JoystickReferenceGenerator(const rclcpp::NodeOptions & options);

protected:
  /// @todo Criar subscriber para o tópico do joystick.
  // O tópico do joystick deve salvar a última leitura do joystick em alguma
  // variável ou variáveis internas desta classe.
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  void joy_callback(const sensor_msgs::msg::Joy& msgS);

  std::vector<double> computeReferences(uint32_t step, rclcpp::Time t) override;

  /// @todo Atualizar computeReferences para implementação da referência via joystick.
  // Essa função deve aplicar de escala aos valores recebidos do joystick
  // e usando esses valores integrar para encontrar as velocidades, e as 
  // posições de referência .
  // Após realizado o cálculo, retornar um vetor na mesma ordem que a função
  // atual.
  void reset() override;
  /// @todo Implementar reset para resetar o estado dos integradores 
  // (voltar a condição inicial, = 0).

  /// @todo Criar um novo launch file para lançar nó junto com o quadrotor e
  // o controlador do LQR.
  // Adicionar nesse launch file pacotes necessários para o joystick.

  double _x_double_dot, _y_double_dot, _z_double_dot, _yaw_double_dot;
  double _x_dot = 0;
  double _y_dot = 0;
  double _z_dot = 0;
  double _yaw_dot = 0;
  double _x = 0;
  double _y = 0;
  double _z = 0;
  double _yaw = 0;

  double _prev_x_double_dot = 0.0;
  double _prev_y_double_dot = 0.0;
  double _prev_z_double_dot = 0.0;
  double _prev_yaw_double_dot = 0.0;

  double _prev_x_dot = 0.0;
  double _prev_y_dot = 0.0;
  double _prev_z_dot = 0.0;
  double _prev_yaw_dot = 0.0;

  double _prev_x = 0.0;
  double _prev_y = 0.0;
  double _prev_z = 0.0;
  double _prev_yaw = 0.0;

  /// @brief Stores the previous computation time.
  double _prev_time = 0.0;

  std::mutex _mutex;
}; 
}  // namespace provant::ref_gen

#endif  // PROVANT_SIMULATOR_REFERENCE_GENERATOR__REFERENCE_GENERATOR_HPP_
