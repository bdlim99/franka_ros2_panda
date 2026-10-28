// Copyright (c) 2021 Franka Emika GmbH
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

#include <franka_hardware/robot.hpp>

#include <cassert>
#include <exception>
#include <mutex>

#include <franka/control_tools.h>
#include <rclcpp/logging.hpp>

namespace franka_hardware {

Robot::Robot(const std::string& robot_ip, const rclcpp::Logger& logger) {
  tau_command_.fill(0.);
  franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
  if (!franka::hasRealtimeKernel()) {
    rt_config = franka::RealtimeConfig::kIgnore;
    RCLCPP_WARN(
        logger,
        "You are not using a real-time kernel. Using a real-time kernel is strongly recommended!");
  }
  robot_ = std::make_unique<franka::Robot>(robot_ip, rt_config);
  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  franka_hardware_model_ = std::make_unique<Model>(model_.get());
}

void Robot::write(const std::array<double, 7>& efforts) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  tau_command_ = efforts;
}

franka::RobotState Robot::read() {
  std::lock_guard<std::mutex> lock(read_mutex_);
  return {current_state_};
}

franka_hardware::Model* Robot::getModel() {
  return franka_hardware_model_.get();
}

void Robot::stopRobot() {
  if (!stopped_) {
    finish_ = true;
    control_thread_->join();
    finish_ = false;
    stopped_ = true;
  }
}

void Robot::initializeTorqueControl() {
  assert(isStopped());
  write({});
  stopped_ = false;
  const auto kTorqueControl = [this]() {
    robot_->control(
        [this](const franka::RobotState& state, const franka::Duration& /*period*/) {
          {
            std::lock_guard<std::mutex> lock(read_mutex_);
            current_state_ = state;
          }
          std::lock_guard<std::mutex> lock(write_mutex_);
          franka::Torques out(tau_command_);
          out.motion_finished = finish_;
          return out;
        },
        true, franka::kMaxCutoffFrequency);
  };
  control_thread_ = std::make_unique<std::thread>(kTorqueControl);
}

void Robot::writeCartesianVelocity(const std::array<double, 6>& velocities) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  cartesian_velocity_command_ = velocities;
}

void Robot::initializeCartesianVelocityControl() {
  assert(isStopped());
  writeCartesianVelocity({});
  control_error_ = false;
  stopped_ = false;
  control_thread_ = std::make_unique<std::thread>([this]() {
    try {
      robot_->control(
          [this](const franka::RobotState& state, franka::Duration /*period*/) {
            {
              std::lock_guard<std::mutex> lock(read_mutex_);
              current_state_ = state;
            }
            std::lock_guard<std::mutex> lock(write_mutex_);
            franka::CartesianVelocities out(cartesian_velocity_command_);
            if (finish_) {
              out.O_dP_EE.fill(0.0);
              out.motion_finished = true;
            }
            return out;
          },
          franka::ControllerMode::kJointImpedance, true, franka::kDefaultCutoffFrequency);
    } catch (const std::exception& e) {
      control_error_ = true;
      RCLCPP_ERROR(rclcpp::get_logger("franka_hardware"), "Cartesian velocity control failed: %s",
                   e.what());
    }
  });
}

void Robot::initializeContinuousReading() {
  assert(isStopped());
  stopped_ = false;
  const auto kReading = [this]() {
    robot_->read([this](const franka::RobotState& state) {
      {
        std::lock_guard<std::mutex> lock(read_mutex_);
        current_state_ = state;
      }
      return !finish_;
    });
  };
  control_thread_ = std::make_unique<std::thread>(kReading);
}

Robot::~Robot() {
  stopRobot();
}

bool Robot::isStopped() const {
  return stopped_;
}
}  // namespace franka_hardware
