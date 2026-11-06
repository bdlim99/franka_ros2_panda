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

// Backported from official humble commit 3b50164eabbe2d461baca007bbbbe03c024387bd.
#include <franka_example_controllers/cartesian_pose_example_controller.hpp>

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace franka_example_controllers {

controller_interface::CallbackReturn CartesianPoseExampleController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(false);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseExampleController::on_configure(
    const rclcpp_lifecycle::State& /*state*/) {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(false);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          franka_cartesian_pose_->get_command_interface_names()};
}

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::state_interface_configuration() const {
  auto names = franka_cartesian_pose_->get_state_interface_names();
  names.push_back("panda/robot_time");
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn CartesianPoseExampleController::on_activate(
    const rclcpp_lifecycle::State& /*state*/) {
  initialization_flag_ = true;
  if (!franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_) ||
      !franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_)) {
    franka_cartesian_pose_->release_interfaces();
    RCLCPP_ERROR(get_node()->get_logger(), "Cartesian pose interfaces are not available");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto time_interface =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                   [](const auto& item) { return item.get_name() == "panda/robot_time"; });
  if (time_interface == state_interfaces_.end()) {
    franka_cartesian_pose_->release_interfaces();
    RCLCPP_ERROR(get_node()->get_logger(), "Robot time interface is not available");
    return controller_interface::CallbackReturn::ERROR;
  }
  robot_time_index_ = std::distance(state_interfaces_.begin(), time_interface);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/) {
  franka_cartesian_pose_->release_interfaces();
  initialization_flag_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPoseExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  const double robot_time = state_interfaces_[robot_time_index_].get_value();
  if (!std::isfinite(robot_time) || (!initialization_flag_ && robot_time < previous_robot_time_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid or decreasing robot time");
    return controller_interface::return_type::ERROR;
  }
  if (initialization_flag_) {
    std::tie(orientation_, position_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    initial_robot_time_ = robot_time;
    initialization_flag_ = false;
  }
  previous_robot_time_ = robot_time;
  constexpr double kPi = 3.14159265358979323846;
  const double elapsed_time = robot_time - initial_robot_time_;
  const double radius = 0.1;
  const double angle = kPi / 4 * (1 - std::cos(kPi / 5.0 * elapsed_time));
  Eigen::Vector3d new_position = position_;
  new_position(0) -= radius * std::sin(angle);
  new_position(2) -= radius * (std::cos(angle) - 1);
  return franka_cartesian_pose_->setCommand(orientation_, new_position)
             ? controller_interface::return_type::OK
             : controller_interface::return_type::ERROR;
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerInterface)
