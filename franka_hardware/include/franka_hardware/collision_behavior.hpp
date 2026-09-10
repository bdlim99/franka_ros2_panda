// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0.
#pragma once

#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace franka_hardware {

// Configuration only: these values are not read back from the robot.
struct CollisionBehavior {
  std::array<double, 7> lower_torque;
  std::array<double, 7> upper_torque;
  std::array<double, 6> lower_force;
  std::array<double, 6> upper_force;

  static bool isConfigured(const std::unordered_map<std::string, std::string>& parameters) {
    return parameters.count("lower_torque_thresholds") ||
           parameters.count("upper_torque_thresholds") ||
           parameters.count("lower_force_thresholds") || parameters.count("upper_force_thresholds");
  }

  explicit CollisionBehavior(const std::unordered_map<std::string, std::string>& parameters)
      : lower_torque(parse<7>(parameters, "lower_torque_thresholds")),
        upper_torque(parse<7>(parameters, "upper_torque_thresholds")),
        lower_force(parse<6>(parameters, "lower_force_thresholds")),
        upper_force(parse<6>(parameters, "upper_force_thresholds")) {}

 private:
  template <size_t N>
  static std::array<double, N> parse(const std::unordered_map<std::string, std::string>& parameters,
                                     const std::string& name) {
    const auto entry = parameters.find(name);
    if (entry == parameters.end()) {
      throw std::invalid_argument("Missing collision behavior parameter: " + name);
    }
    std::istringstream stream(entry->second);
    std::array<double, N> values{};
    for (auto& value : values) {
      if (!(stream >> value) || !std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(name + " must contain " + std::to_string(N) +
                                    " finite positive numbers");
      }
    }
    stream >> std::ws;
    if (!stream.eof()) {
      throw std::invalid_argument("Too many values or invalid trailing data in " + name);
    }
    return values;
  }

};
}  // namespace franka_hardware
