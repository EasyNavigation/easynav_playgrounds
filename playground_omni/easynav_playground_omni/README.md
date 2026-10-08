<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Omni Playground

EasyNav configuration for three- to six-wheel omnidirectional robots in Gazebo Harmonic. The default launch starts the `3w_v2` robot in `maze2`, together with EasyNav and RViz2.

## Demo

[![Watch the EasyNav Omni Playground demo](https://img.youtube.com/vi/8tPknIoeD1M/hqdefault.jpg)](https://youtu.be/8tPknIoeD1M)

The omni playground is made of three packages:

| Package | Contents |
| --- | --- |
| `easynav_playground_omni_description` | The robots' models: URDF, meshes and ros2_control controllers. No Gazebo, no EasyNav |
| `easynav_playground_omni_worlds` | The Gazebo simulation: two mazes and their maps, the kinematics node, and the launcher that spawns the chosen robot with its controllers and the ROS–Gazebo bridge. No EasyNav |
| `easynav_playground_omni` | The EasyNav configuration and launcher |

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_omni
source install/setup.bash
```

## Launch EasyNav

```bash
ros2 launch easynav_playground_omni easynav_navigation_gazebo_sim.launch.yaml
```

The `world` launch argument selects both the Gazebo world and its matching EasyNav map. The available options are `maze1` and `maze2` (default):

```bash
ros2 launch easynav_playground_omni easynav_navigation_gazebo_sim.launch.yaml world:=maze1
```

```bash
ros2 launch easynav_playground_omni easynav_navigation_gazebo_sim.launch.yaml world:=maze2
```

Other launch arguments, `params_file`, `rviz_config`, `map_override_file` and `robot`, can also be overridden.

The `robot` argument chooses the robot (`3w`, `3w_v2`, `4w`, `5w`, `6w`; see `easynav_playground_omni_description`), for example:

```bash
ros2 launch easynav_playground_omni easynav_navigation_gazebo_sim.launch.yaml robot:=5w world:=maze1
```

All robot models share EasyNav limits of 0.6 m/s linear and 0.5 rad/s angular velocity.

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | EasyNav launch file, in YAML |
| `params/` | EasyNav parameters, and the map of each world (`map_overrides/<world>/map.yaml`) |
| `rviz/` | RViz2 configuration |

## Authors and licensing

This package was developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), and is licensed under Apache-2.0; see [LICENSE](./LICENSE). The simulator it uses is [Yohan Prakoso](https://github.com/YePeOn7/ros2_omni_robot_sim)'s: see `easynav_playground_omni_worlds` and `easynav_playground_omni_description`.
