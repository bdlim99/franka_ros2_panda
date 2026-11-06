# ROS 2 integration for Franka Emika research robots

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## Cartesian example controllers

The pose and velocity controllers use the official humble semantic component layout:
`franka_example_controllers` computes trajectories, `franka_semantic_components` provides
`FrankaCartesianPoseInterface` / `FrankaCartesianVelocityInterface`, and `franka_hardware`
exports the interfaces and communicates with libfranka. Sources and interface tests were
backported from upstream commit `3b50164eabbe2d461baca007bbbbe03c024387bd`.
The backport fixes vector sizing for Eigen writes, checks vector size instead of capacity,
clears loaned references on reassignment, and normalizes quaternion rotation commands.

The installed libfranka 0.8 has no ActiveControl `readOnce`/`writeOnce` API. Its callback
adapter remains inside `robot.hpp` / `robot.cpp`, pairing each state with the following
ROS command. Cartesian rate limiting and low-pass filtering are disabled, matching the
upstream defaults. The pose reference remains `O_T_EE_c` to retain the installed
libfranka example's startup behavior. Elbow commands are not exported by this hardware.
The pose command interface is `0/cartesian_pose_command` through `15/cartesian_pose_command`.
There is no automatic CSV trace or plotting executable.

## License

All packages of `franka_ros2` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html

[fci-docs]: https://frankaemika.github.io/docs
