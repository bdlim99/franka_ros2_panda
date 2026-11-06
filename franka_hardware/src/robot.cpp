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
#include <future>
#include <mutex>

#include <franka/control_tools.h>
#include <rclcpp/logging.hpp>

namespace franka_hardware {
namespace {
// In libfranka 0.8, unwinding a control callback invokes cancelMotion()/StopMove.
// Use that existing cancellation path on lifecycle switches, rather than sending
// a discontinuous hold pose or declaring MotionFinished while still moving.
class CartesianControlStopped : public std::exception {
 public:
  const char* what() const noexcept override { return "Cartesian control stopped"; }
};
}  // namespace

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
  if (cartesian_control_active_) {
    std::unique_lock<std::mutex> lock(cartesian_mutex_);
    if (cartesian_streaming_) {
      cartesian_changed_.wait(lock, [this]() {
        return finish_ || control_error_ || cartesian_state_sequence_ > cartesian_read_sequence_;
      });
    }
    cartesian_read_sequence_ = cartesian_state_sequence_;
    return current_state_;
  }
  std::lock_guard<std::mutex> lock(read_mutex_);
  return {current_state_};
}

franka_hardware::Model* Robot::getModel() {
  return franka_hardware_model_.get();
}

void Robot::stopRobot() {
  if (!stopped_) {
    finish_ = true;
    // Take the same mutex as wait() so a stop notification cannot be lost.
    {
      std::lock_guard<std::mutex> lock(cartesian_mutex_);
      cartesian_changed_.notify_all();
    }
    control_thread_->join();
    cartesian_control_active_ = false;
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
  std::lock_guard<std::mutex> lock(cartesian_mutex_);
  cartesian_velocity_command_ = velocities;
  cartesian_command_state_sequence_ = cartesian_read_sequence_;
  cartesian_streaming_ = true;
  cartesian_changed_.notify_all();
}

void Robot::writeCartesianPose(const std::array<double, 16>& pose) {
  std::lock_guard<std::mutex> lock(cartesian_mutex_);
  cartesian_pose_command_ = pose;
  cartesian_command_state_sequence_ = cartesian_read_sequence_;
  cartesian_streaming_ = true;
  cartesian_changed_.notify_all();
}

void Robot::waitForCartesianCommand(const franka::RobotState& state) {
  std::unique_lock<std::mutex> lock(cartesian_mutex_);
  if (cartesian_state_sequence_ == 0) {
    cartesian_pose_command_ = state.O_T_EE_c;
    cartesian_velocity_command_.fill(0.0);
  }
  current_state_ = state;
  ++cartesian_state_sequence_;
  cartesian_changed_.notify_all();
  // ActiveControl readOnce/writeOnce is unavailable in libfranka 0.8. Return a
  // hold during activation, then pair each callback state with its ROS write.
  if (cartesian_streaming_) {
    cartesian_changed_.wait(lock, [this]() {
      return finish_ || cartesian_command_state_sequence_ == cartesian_state_sequence_;
    });
  }
  if (finish_) {
    throw CartesianControlStopped();
  }
}

void Robot::initializeCartesianVelocityControl() {
  initializeCartesianControl(false);
}

void Robot::initializeCartesianPoseControl() {
  initializeCartesianControl(true);
}

void Robot::initializeCartesianControl(bool pose_control) {
  assert(isStopped());
  {
    std::lock_guard<std::mutex> lock(cartesian_mutex_);
    cartesian_streaming_ = false;
    cartesian_state_sequence_ = 0;
    cartesian_read_sequence_ = 0;
    cartesian_command_state_sequence_ = 0;
  }
  cartesian_control_active_ = true;
  control_error_ = false;
  auto ready = std::make_shared<std::promise<void>>();
  auto ready_future = ready->get_future();
  control_thread_ = std::make_unique<std::thread>([this, pose_control, ready]() {
    bool initialized = false;
    const auto exchange = [this, &initialized, &ready](const franka::RobotState& state) {
      waitForCartesianCommand(state);
      if (!initialized) {
        initialized = true;
        ready->set_value();
      }
    };
    try {
      if (pose_control) {
        robot_->control(
            [this, &exchange, &initialized](const franka::RobotState& state,
                                            franka::Duration /*period*/) {
              const bool first_callback = !initialized;
              exchange(state);
              std::lock_guard<std::mutex> lock(cartesian_mutex_);
              return franka::CartesianPose(first_callback ? state.O_T_EE_c
                                                          : cartesian_pose_command_);
            },
            franka::ControllerMode::kJointImpedance, false, franka::kMaxCutoffFrequency);
      } else {
        robot_->control(
            [this, &exchange](const franka::RobotState& state, franka::Duration /*period*/) {
              exchange(state);
              std::lock_guard<std::mutex> lock(cartesian_mutex_);
              return franka::CartesianVelocities(cartesian_velocity_command_);
            },
            franka::ControllerMode::kJointImpedance, false, franka::kMaxCutoffFrequency);
      }
    } catch (const CartesianControlStopped&) {
      if (!initialized) {
        ready->set_exception(std::current_exception());
      }
    } catch (const std::exception& e) {
      control_error_ = true;
      if (!initialized) {
        ready->set_exception(std::current_exception());
      }
      RCLCPP_ERROR(rclcpp::get_logger("franka_hardware"), "Cartesian %s control failed: %s",
                   pose_control ? "pose" : "velocity", e.what());
    }
    // Wake a read waiting for a state if libfranka exits on an error.
    std::lock_guard<std::mutex> lock(cartesian_mutex_);
    cartesian_changed_.notify_all();
  });
  stopped_ = false;
  try {
    ready_future.get();
  } catch (...) {
    stopRobot();
    throw;
  }
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
