<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Summit Playground

EasyNav configurations for a Robotnik Summit XL in Gazebo Harmonic, in two worlds: the URJC excavation, an outdoor 3D terrain, and a small indoor warehouse. This is the outdoor reference playground of EasyNav.

The Summit playground is made of three packages:

| Package | Contents |
| --- | --- |
| `easynav_playground_summit_description` | The Summit XL model: URDF, meshes and ros2_control controllers. No Gazebo, no EasyNav |
| `easynav_playground_summit_worlds` | The Gazebo simulation: the URJC excavation and a small warehouse, their maps, and the launchers that spawn the Summit XL with its controllers and the ROS–Gazebo bridge. No EasyNav |
| `easynav_playground_summit` | The EasyNav configurations and launchers |

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_summit
source install/setup.bash
```

## Launch EasyNav

Each `easynav_<config>.launch.yaml` starts Gazebo, the Summit XL, EasyNav with that configuration and RViz2:

```bash
ros2 launch easynav_playground_summit easynav_bonxai_amcl.launch.yaml
```

Once RViz2 is up, send a goal with the **2D Goal Pose** tool.

### NavMap configurations

All of them plan with A* over the NavMap (`maps/excavation_urjc.navmap`).

| Launch file | Controller | Localizer | Notes |
| --- | --- | --- | --- |
| `easynav_bonxai_amcl.launch.yaml` | Regulated Pure Pursuit | NavMap AMCL | AMCL corrects against the Bonxai 3D map (`maps/excavation_urjc.pcd`) |
| `easynav_mppi.launch.yaml` | MPPI | NavMap AMCL | |
| `easynav_mpc.launch.yaml` | MPC | NavMap AMCL | |
| `easynav_gps.launch.yaml` | Regulated Pure Pursuit | GPS (UKF) | `FusionLocalizer` fusing the GPS position, wheel odometry and IMU heading; no Bonxai map |
| `easynav_navmap_dummy.launch.yaml` | — | — | Only loads and shows `maps/excavation_urjc_2.navmap` |

### Warehouse configuration

`easynav_warehouse_amcl.launch.yaml` runs the Summit XL in the AWS RoboMaker small warehouse (`worlds/small_warehouse.world`), an indoor world with shelves, pallets and clutter:

```bash
ros2 launch easynav_playground_summit easynav_warehouse_amcl.launch.yaml
```

| Launch file | Controller | Localizer | Maps |
| --- | --- | --- | --- |
| `easynav_warehouse_amcl.launch.yaml` | Regulated Pure Pursuit | NavMap AMCL | Flat NavMap from `maps/warehouse.yaml`; Bonxai `maps/warehouse.pcd` |

The NavMap is flat, built from a 2D occupancy grid. Its `obstacles` filter keeps the static map and adds the points the sensors see within 3 m and below 1.2 m (`max_range`, `max_height`); the `inflation` filter (radius 2.0 m, scaling 1.5) keeps paths in the middle of the aisles. The RViz view (`rviz/easynav_warehouse.rviz`) shows the inflated layer. Localization error is about 0.15 m.

### Localization

In the excavation, the terrain is smooth and the Bonxai cloud is sparse, so NavMap AMCL has little to correct against: it keeps the heading from the IMU, but its position can drift 0.5–1 m from the true one. The GPS configuration is the accurate one outdoors (about 0.15 m): its map frame is the Gazebo world's (`latitude_origin`/`longitude_origin` are the world's `spherical_coordinates`).

### Launch arguments

| Argument | Default | Description |
| --- | --- | --- |
| `params_file` | Per launch file | EasyNav parameters file |
| `rviz_config` | Per launch file | RViz2 configuration file |
| `gui` | `true` | Set to `false` to run Gazebo headless |
| `rviz` | `true` | Set to `false` to skip RViz2 |

For example, to run headless with your own parameters:

```bash
ros2 launch easynav_playground_summit easynav_bonxai_amcl.launch.yaml gui:=false params_file:=/path/to/my.params.yaml
```

### Real-time cycles

The configurations run EasyNav's real-time cycle (`use_real_time: true`) at 50 Hz (`system_node.rt_freq`; the default 200 Hz is too tight for this simulation). The recovery system watches it: if many cycles in a row start late, it holds the mission, asks for human assistance and finally cancels it. See "Real-time system setup" in the EasyNav docs.

The maps are in `easynav_playground_summit_worlds`.

## Docker

`docker/` builds an image with EasyNav and this playground; see [docker/README.md](./docker/README.md).

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | EasyNav launch files, in YAML |
| `params/` | EasyNav parameters, one file per configuration |
| `rviz/` | RViz2 configurations |
| `docker/` | Docker image and launcher |

## Authors and licensing

This package was developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), and is licensed under Apache-2.0; see [LICENSE](./LICENSE).
