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

#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <franka_example_controllers/cartesian_velocity_example_controller.hpp>
#include <hardware_interface/handle.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

class CartesianVelocityControllerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    controller = std::make_unique<franka_example_controllers::CartesianVelocityExampleController>();
    ASSERT_EQ(controller->init("test_cartesian_velocity"), controller_interface::return_type::OK);
    for (const auto& axis : {"vx", "vy", "vz", "wx", "wy", "wz"}) {
      handles.emplace_back(axis, "cartesian_velocity", &commands[handles.size()]);
    }
    std::vector<hardware_interface::LoanedCommandInterface> loaned;
    for (auto& handle : handles) {
      loaned.emplace_back(handle);
    }
    controller->assign_interfaces(std::move(loaned), {});
    ASSERT_EQ(controller->on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
  }
  void advance(double seconds) {
    ASSERT_EQ(controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(seconds)),
              controller_interface::return_type::OK);
  }
  std::unique_ptr<franka_example_controllers::CartesianVelocityExampleController> controller;
  std::array<double, 6> commands{};
  std::vector<hardware_interface::CommandInterface> handles;
};

TEST_F(CartesianVelocityControllerTest, FollowsOfficialTrajectoryAndReturnsToOrigin) {
  const double peak = 0.05 / std::sqrt(2.0);
  EXPECT_EQ(commands, (std::array<double, 6>{}));
  advance(2.0);
  EXPECT_NEAR(commands[0], peak, 1e-12);
  EXPECT_NEAR(commands[2], -peak, 1e-12);
  for (const auto index : {1, 3, 4, 5}) {
    EXPECT_DOUBLE_EQ(commands[index], 0.0);
  }
  advance(2.0);
  EXPECT_NEAR(commands[0], 0.0, 1e-12);
  advance(2.0);
  EXPECT_NEAR(commands[0], -peak, 1e-12);
  EXPECT_NEAR(commands[2], peak, 1e-12);
  advance(2.0);
  EXPECT_NEAR(commands[0], 0.0, 1e-12);
  // Integrate a full cycle at the actual controller rate.
  double displacement = 0.0;
  for (int i = 0; i < 8000; ++i) {
    advance(0.001);
    displacement += commands[0] * 0.001;
  }
  EXPECT_NEAR(displacement, 0.0, 1e-10);
}

TEST_F(CartesianVelocityControllerTest, StopsAndRestartsWithZeroCommand) {
  advance(2.0);
  ASSERT_EQ(controller->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(commands, (std::array<double, 6>{}));
  ASSERT_EQ(controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  advance(0.0);
  EXPECT_EQ(commands, (std::array<double, 6>{}));
}

TEST_F(CartesianVelocityControllerTest, RejectsMissingInterfaces) {
  controller->release_interfaces();
  EXPECT_EQ(controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
}

TEST_F(CartesianVelocityControllerTest, PluginCanBeLoaded) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", "controller_interface::ControllerInterface");
  auto plugin =
      loader.createSharedInstance("franka_example_controllers/CartesianVelocityExampleController");
  EXPECT_EQ(plugin->init("loaded_cartesian_velocity"), controller_interface::return_type::OK);
}
