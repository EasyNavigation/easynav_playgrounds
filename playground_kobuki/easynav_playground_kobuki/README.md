<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Kobuki Playground

EasyNav configurations for a Kobuki (Turtlebot 2) in a Gazebo Harmonic simulation of the AWS RoboMaker small house. This is the indoor reference playground of EasyNav.

The Kobuki playground is made of three packages:

| Package | Contents |
| --- | --- |
| `easynav_playground_kobuki_description` | The Kobuki (Turtlebot 2) model: URDF and meshes. No Gazebo, no EasyNav |
| `easynav_playground_kobuki_worlds` | The Gazebo simulation: the small house world, its map, and the launchers that spawn one or several Kobukis with the ROS–Gazebo bridge. No EasyNav |
| `easynav_playground_kobuki` | The EasyNav configurations and launchers |

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_kobuki
source install/setup.bash
```

## Launch EasyNav

Each `easynav_<config>.launch.yaml` starts Gazebo, the Kobuki, EasyNav with that configuration and RViz2:

```bash
ros2 launch easynav_playground_kobuki easynav_costmap_rpp.launch.yaml
```

Once RViz2 is up, send a goal with the **2D Goal Pose** tool.

### Costmap configurations

All of them use the `maps/home2.yaml` occupancy map.

| Launch file | Controller | Localizer | Notes |
| --- | --- | --- | --- |
| `easynav_costmap.launch.yaml` | Simple | AMCL | |
| `easynav_costmap_rpp.launch.yaml` | Regulated Pure Pursuit | AMCL | Tuned for the Kobuki footprint |
| `easynav_costmap_rpp_mhamcl.launch.yaml` | Regulated Pure Pursuit | Multi-hypothesis AMCL | |
| `easynav_costmap_rpp_reflex.launch.yaml` | Regulated Pure Pursuit | AMCL | For trying the collision safety reflex by hand |
| `easynav_costmap_rpp_safe.launch.yaml` | Regulated Pure Pursuit | AMCL | Safety mode, see [below](#safety-mode) |
| `easynav_costmap_serest.launch.yaml` | SeReST | AMCL | |
| `easynav_costmap_mppi.launch.yaml` | MPPI | AMCL | |
| `easynav_costmap_mpc.launch.yaml` | MPC | AMCL | |
| `easynav_costmap_mppi_routed.launch.yaml` | MPPI | AMCL | Routes from `maps/routes_1.yaml` |
| `easynav_routes.launch.yaml` | Regulated Pure Pursuit | AMCL | Routes from `maps/routes_1.yaml` |
| `easynav_costmap_mapping.launch.yaml` | — | — | Builds a costmap |

### Simple map configurations

All of them use the `maps/home.map` simple map.

| Launch file | Controller | Localizer | Notes |
| --- | --- | --- | --- |
| `easynav_simple.launch.yaml` | Simple | AMCL | |
| `easynav_simple_serest.launch.yaml` | SeReST | AMCL | |
| `easynav_simple_mapping.launch.yaml` | — | — | Builds a simple map |

### Launch arguments

| Argument | Default | Description |
| --- | --- | --- |
| `params_file` | Per launch file | EasyNav parameters file |
| `rviz_config` | Per launch file | RViz2 configuration file |
| `gui` | `true` | Set to `false` to run Gazebo headless |
| `rviz` | `true` | Set to `false` to skip RViz2 |
| `lidar_range` | `3.0` | Maximum lidar range, in meters |
| `camera` | `false` | Enable the RGBD camera |

For example, to run headless with your own parameters:

```bash
ros2 launch easynav_playground_kobuki easynav_costmap_rpp.launch.yaml gui:=false params_file:=/path/to/my.params.yaml
```

### Safety mode

`easynav_costmap_rpp_safe.launch.yaml` runs EasyNav in safety mode with the process memory locked. It needs real-time limits (`rtprio` >= 80 and, for memory locking, `memlock` unlimited); otherwise EasyNav does not start. See "Real-time system setup" in the EasyNav docs.

It also publishes an "all clear" safety status on `/easynav_safety_status`, standing in for a safety PLC or scanner. Launch it with `safety_channel:=false` to publish your own status, for example a protective stop.

The occupancy map (`home2.yaml`) is in `easynav_playground_kobuki_worlds`; the Simple map (`maps/home.map`) and the routes (`maps/routes_1.yaml`) are EasyNav formats, so they are in this package.

## Multirobot

Two Kobukis, `r1` at (0, 0) and `r2` at (2, 1), each with its own EasyNav and RViz2:

```bash
ros2 launch easynav_playground_kobuki easynav_multirobot.launch.yaml
```

Each robot uses its own namespace for topics (`/r1/scan_raw`), its own TF topic (`/r1/tf`) and prefixed frames (`r1/map`, `r1/base_link`). The parameters are in `params/costmap_multirobot.params.yaml`, with one section per robot. Each section is `costmap.rpp.params.yaml` (Regulated Pure Pursuit, 0.6 m/s) with that robot's `tf_prefix` and initial pose.

## Docker

`docker/` builds an image with EasyNav and this playground; see [docker/README.md](./docker/README.md).

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | EasyNav launch files, in YAML (`rviz_namespaced.launch.py` opens RViz2 for a namespaced robot) |
| `params/` | EasyNav parameters, one file per configuration |
| `maps/` | EasyNav-specific maps: `home.map` (Simple map) and `routes_1.yaml` (routes) |
| `rviz/` | RViz2 configurations |
| `docker/` | Docker image and launcher |

## Authors and licensing

This package was developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), and is licensed under Apache-2.0; see [LICENSE](./LICENSE).
