// Copyright (c) 2023 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Trajectory adapted from the official humble CartesianVelocityExampleController:
// humble commit 3b50164eabbe2d461baca007bbbbe03c024387bd,
// src/fr3/cartesian_velocity_example_controller.cpp
#include <franka_example_controllers/cartesian_velocity_example_controller.hpp>

#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace franka_example_controllers {

controller_interface::CallbackReturn CartesianVelocityExampleController::on_init() {
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(false);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*state*/) {
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(false);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          franka_cartesian_velocity_->get_command_interface_names()};
}

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*state*/) {
  if (!franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_)) {
    franka_cartesian_velocity_->release_interfaces();
    return controller_interface::CallbackReturn::ERROR;
  }
  elapsed_time_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/) {
  franka_cartesian_velocity_->setCommand(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  franka_cartesian_velocity_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianVelocityExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTimeMax = 4.0;
  constexpr double kVelocityMax = 0.05;
  const double dt = period.seconds();
  if (command_interfaces_.size() != 6 || !std::isfinite(dt) || dt < 0.0) {
    return controller_interface::return_type::ERROR;
  }
  // Wrap at a full out-and-back cycle to avoid unbounded time accumulation.
  elapsed_time_ = std::fmod(elapsed_time_ + dt, 2.0 * kTimeMax);
  const double direction = elapsed_time_ < kTimeMax ? 1.0 : -1.0;
  const double velocity =
      direction * kVelocityMax / 2.0 * (1.0 - std::cos(2.0 * kPi * elapsed_time_ / kTimeMax));
  const Eigen::Vector3d linear_velocity(std::cos(kPi / 4.0) * velocity, 0.0,
                                        -std::sin(kPi / 4.0) * velocity);
  return franka_cartesian_velocity_->setCommand(linear_velocity, Eigen::Vector3d::Zero())
             ? controller_interface::return_type::OK
             : controller_interface::return_type::ERROR;
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerInterface)
