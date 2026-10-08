<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Omni Playground: worlds

Gazebo Harmonic simulation of three- to six-wheel omnidirectional robots in two mazes: the worlds, their maps, the kinematics node (wheel commands and odometry from the velocity command), and the launcher that starts Gazebo and spawns the chosen robot (robot state publisher, ros2_control controllers and ROS–Gazebo bridge). It does not depend on EasyNav: `easynav_playground_omni` adds the navigation.

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_omni_worlds
source install/setup.bash
```

## Simulation

To run only Gazebo and a robot (`3w`, `3w_v2`, `4w`, `5w`, `6w`) in a maze (`maze1`, `maze2`):

```bash
ros2 launch easynav_playground_omni_worlds gazebo_sim.launch.yaml robot:=4w world:=maze2
```

The simulated wheel command limits are ±35 rad/s, enough for 0.6 m/s.

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | `gazebo_sim.launch.yaml` |
| `src/` | `kinematics` node |
| `scripts/` | Test scripts for the motors |
| `config/gz_bridge/` | ROS–Gazebo bridge topics |
| `maps/` | Occupancy grids of the two mazes |
| `worlds/` | `maze1.sdf`, `maze2.sdf` |
| `rviz/` | RViz2 configurations for the simulation |

## Authors and licensing

The mazes, maps, bridge configuration, kinematics node and test scripts are [Yohan Prakoso](https://github.com/YePeOn7/ros2_omni_robot_sim)'s original simulator: Copyright (c) 2025 Yohan Prakoso, under the MIT license; see [LICENSE](./LICENSE). The launcher was written by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), under Apache-2.0; see [LICENSE-APACHE](./LICENSE-APACHE).
