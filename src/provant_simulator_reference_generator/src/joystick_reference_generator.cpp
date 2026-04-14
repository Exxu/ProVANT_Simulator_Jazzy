/*
* This file is part of the ProVANT simulator project.
* Licensed under the terms of the MIT open source license. More details at
* https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
* Copyright 2022 Guilherme V. Raffo
*/
/**
* @file reference_generator.cpp
* @brief This file contains the implementation of the ReferenceGenerator class.
*
* @author Lorena Oliveira
* @author Guilherme Barbosa
*/
#include "provant_simulator_reference_generator/joystick_reference_generator.hpp"

#include <cmath>
#include <vector>

using provant::ref_gen::JoystickReferenceGenerator;

JoystickReferenceGenerator::JoystickReferenceGenerator(const rclcpp::NodeOptions & options)
: BaseReferenceGenerator(options) {

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
  "joy", qos, std::bind(&JoystickReferenceGenerator::joy_callback, this, std::placeholders::_1));

}

std::vector<double> JoystickReferenceGenerator::computeReferences(uint32_t /*step*/, rclcpp::Time t)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "Computing the reference vector.");

  auto const sec = static_cast<double>(t.nanoseconds()) / 1e9;
  double const  deltaT  = sec - _prev_time;

  double const acc_scale_factor = 5.0;

  std::lock_guard<std::mutex> _lock(this->_mutex);

  this->_x_double_dot = _x_double_dot / acc_scale_factor;
  this->_y_double_dot = _y_double_dot / acc_scale_factor;
  this->_z_double_dot = _z_double_dot / acc_scale_factor;
  this->_yaw_double_dot = _yaw_double_dot / acc_scale_factor;

  // First step, integrate acceleration to obtain speed
  _x_dot += 0.5 * deltaT * (_prev_x_double_dot + _x_double_dot);
  _y_dot += 0.5 * deltaT * (_prev_y_double_dot + _y_double_dot);
  _z_dot += 0.5 * deltaT * (_prev_z_double_dot + _z_double_dot);
  _yaw_dot += 0.5 * deltaT * (_prev_yaw_double_dot + _yaw_double_dot);

  // double const p_dot_min = -1.0;
  // double const p_dot_max = 1.0;
  // double const yaw_dot_min = -M_PI/100;
  // double const yaw_dot_max = M_PI/100;

  // _x_dot = std::max(p_dot_min, std::min(p_dot_max, _x_dot));
  // _y_dot = std::max(p_dot_min, std::min(p_dot_max, _y_dot));
  // _z_dot = std::max(p_dot_min, std::min(p_dot_max, _z));
  // _yaw_dot = std::max(yaw_dot_min, std::min(yaw_dot_max, _yaw_dot));

  if (_z < 1 ) {
    _z = 1.0;
  }
  // else if (_z > 3) {
  //   _z_dot = -0.1;
  // } 

  // Second step, integrate speed to obtain positions
  _x += 0.5 * deltaT * (_prev_x_dot + _x_dot);
  _y += 0.5 * deltaT * (_prev_y_dot + _y_dot);
  _z += 0.5 * deltaT * (_prev_z_dot + _z_dot);
  _yaw += 0.5 * deltaT * (_prev_yaw_dot + _yaw_dot);

  // double const p_min = -10.0;
  double const z_min = 1.0;
  double const p_max = 10.0;

  // _x = std::max(p_min, std::min(p_max, _x));
  // _y = std::max(p_min, std::min(p_max, _y));
  //_z = std::max(z_min, std::min(p_max, _z));
  // _yaw = std::max(-M_PI, std::min(M_PI, _yaw));

  /// @todo Add saturation for both speed and positions

  // Update previous values
  _prev_time = sec;
  _prev_x_double_dot = _x_double_dot;
  _prev_y_double_dot = _y_double_dot;
  _prev_z_double_dot = _z_double_dot;
  _prev_yaw_double_dot = _yaw_double_dot;

  _prev_x_dot = _x_dot;
  _prev_y_dot = _y_dot;
  _prev_z_dot = _z_dot;
  _prev_yaw_dot = _yaw_dot;

  _prev_x = _x;
  _prev_y = _y;
  _prev_z = _z;
  _prev_yaw = _yaw;

  return{
    _x,                    // x
    _y,                    // y
    _z,                    // z
    0.0,                    // roll angle
    0.0,                    // pitch angle
    _yaw,                    // yaw angle
    _x_dot,                // x_dot
    _y_dot,                // y_dot
    _z_dot,                // z_dot
    0.0,                    // roll_dot
    0.0,                    // pitch_dot
    _yaw_dot 
  };
}

void JoystickReferenceGenerator::joy_callback(const sensor_msgs::msg::Joy& msgS) 
{
  std::lock_guard<std::mutex> _lock(this->_mutex);

  this->_x_double_dot = msgS.axes[0];
  this->_y_double_dot = msgS.axes[1];
  this->_z_double_dot = msgS.axes[2];
  this->_yaw_double_dot = msgS.axes[3];
}

void JoystickReferenceGenerator::reset() {RCLCPP_DEBUG_STREAM(get_logger(), "Reset method called.");}
