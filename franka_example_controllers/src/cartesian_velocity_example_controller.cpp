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
// humble commit 3b50164eabbe2d461baca007bbbbe03c024387bd, src/fr3/cartesian_velocity_example_controller.cpp
#include <franka_example_controllers/cartesian_velocity_example_controller.hpp>

#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace franka_example_controllers {

controller_interface::CallbackReturn CartesianVelocityExampleController::on_init() {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {"vx/cartesian_velocity", "vy/cartesian_velocity", "vz/cartesian_velocity",
           "wx/cartesian_velocity", "wy/cartesian_velocity", "wz/cartesian_velocity"}};
}

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*state*/) {
  const auto names = command_interface_configuration().names;
  if (command_interfaces_.size() != names.size()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < names.size(); ++i) {
    if (command_interfaces_[i].get_name() != names[i]) {
      return controller_interface::CallbackReturn::ERROR;
    }
    command_interfaces_[i].set_value(0.0);
  }
  elapsed_time_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/) {
  for (auto& interface : command_interfaces_) {
    interface.set_value(0.0);
  }
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
  command_interfaces_[0].set_value(std::cos(kPi / 4.0) * velocity);
  command_interfaces_[1].set_value(0.0);
  command_interfaces_[2].set_value(-std::sin(kPi / 4.0) * velocity);
  for (size_t i = 3; i < 6; ++i) {
    command_interfaces_[i].set_value(0.0);
  }
  return controller_interface::return_type::OK;
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerInterface)
