// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0.
#pragma once

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include <franka_hardware/collision_behavior.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_hardware {

// Exposes the configuration successfully sent to the robot, not robot readback.
class CollisionBehaviorParameterServer {
 public:
  explicit CollisionBehaviorParameterServer(const CollisionBehavior& behavior) {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    node_ = std::make_shared<rclcpp::Node>("franka_collision_behavior", "/", options);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Thresholds successfully sent via setCollisionBehavior; not robot readback.";
    const auto declare = [&](const std::string& name, const auto& values) {
      node_->declare_parameter(name, std::vector<double>(values.begin(), values.end()),
                               descriptor, true);
    };
    declare("lower_torque_thresholds", behavior.lower_torque);
    declare("upper_torque_thresholds", behavior.upper_torque);
    declare("lower_force_thresholds", behavior.lower_force);
    declare("upper_force_thresholds", behavior.upper_force);
    executor_.add_node(node_);
    // Bounded spins let destruction stop this worker even before it first spins.
    worker_ = std::thread([this]() {
      while (!stopping_.load() && rclcpp::ok(node_->get_node_base_interface()->get_context())) {
        executor_.spin_once(std::chrono::milliseconds(100));
      }
    });
  }

  ~CollisionBehaviorParameterServer() {
    stopping_.store(true);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  std::atomic<bool> stopping_{false};
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread worker_;
};
}  // namespace franka_hardware
