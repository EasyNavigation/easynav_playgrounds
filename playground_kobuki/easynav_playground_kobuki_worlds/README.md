<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Kobuki Playground: worlds

Gazebo Harmonic simulation of a Kobuki (Turtlebot 2) in the AWS RoboMaker small house: the world and its models, the map of the house, and the launchers that start Gazebo and spawn one or several Kobukis (robot state publisher and ROS–Gazebo bridge). It does not depend on EasyNav: `easynav_playground_kobuki` adds the navigation.

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_kobuki_worlds
source install/setup.bash
```

## Simulation

To run Gazebo and the Kobuki without EasyNav or RViz2:

```bash
ros2 launch easynav_playground_kobuki_worlds gazebo_sim.launch.yaml
```

```bash
ros2 launch easynav_playground_kobuki_worlds multirobot_gazebo_sim.launch.yaml
```

To add a robot to a running simulation, use `kobuki.launch.yaml` with a `namespace` and a spawn pose (`x`, `y`, `Y`...).

The simulated Kobuki publishes `/scan_raw`, `/odom`, `/joint_states` and TF, and listens on `/cmd_vel`. With `camera:=true` it also publishes `/rgbd_camera/{image,depth_image,points,camera_info}`.

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | `gazebo_sim` (world and one Kobuki), `multirobot_gazebo_sim` (world and two Kobukis), `world` (Gazebo with the house), `kobuki` (spawns a Kobuki in a running simulation), in YAML |
| `config/bridge/` | ROS–Gazebo bridge topics |
| `maps/` | `home2.yaml`: occupancy grid of the house (5 cm) |
| `worlds/`, `models/`, `photos/` | Small house world |

## Authors and licensing

The launchers, bridge configuration and map were developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), under Apache-2.0; see [LICENSE](./LICENSE).

The small house world (`worlds/`, `models/`, `photos/`) comes from [aws-robomaker-small-house-world](https://github.com/IntelligentRoboticsLabs/aws-robomaker-small-house-world) (`ros2` branch), under the MIT No Attribution license; see [models/LICENSE](./models/LICENSE). Only the models the world uses were copied.
