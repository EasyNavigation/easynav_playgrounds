<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Aerial Playground: worlds

Gazebo Harmonic simulation of a flying saucer in a town in Nathan Benderson Park. It includes the world and its models, the town's 3D map, the UFO's flight controller, and the launchers that start Gazebo and spawn the UFO with the ROS–Gazebo bridge. It does not depend on EasyNav: `easynav_playground_aerial` adds the navigation.

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_aerial_worlds
source install/setup.bash
```

## Simulation

To run Gazebo and the UFO without EasyNav or RViz2:

```bash
ros2 launch easynav_playground_aerial_worlds gazebo_sim.launch.yaml
```

The UFO starts hovering at 6 m, in a street of the town. Its spawn pose is set with the `x`, `y`, `z` and `yaw` arguments. To spawn it in a running simulation, use `ufo.launch.yaml`.

The simulated UFO publishes:

| Topic | Type |
| --- | --- |
| `/top_laser/points`, `/bottom_laser/points` | `sensor_msgs/PointCloud2` |
| `/front_camera/image` | `sensor_msgs/Image` |
| `/front_camera/camera_info` | `sensor_msgs/CameraInfo` |
| `/imu/data` | `sensor_msgs/Imu` |
| `/gps/fix` | `sensor_msgs/NavSatFix` |
| `/odom` | `nav_msgs/Odometry` |

It also publishes TF, and listens on `/cmd_vel` (`geometry_msgs/Twist`, in the body frame: linear x, y, z and angular z).

## The flight controller

The UFO has no gravity: Gazebo moves it at the velocity it is given. `scripts/ufo_flight_controller.py` turns the commanded twist into that velocity, as a body with inertia would:
- **Limits:** the command is clamped to the velocity limits.
- **Response:** the velocity follows the command with a first-order response (`time_constant`, 0.3 s), bounded by the acceleration and deceleration limits of each axis.
- **Timeout:** without commands for `cmd_timeout` (0.5 s), the UFO brakes and hovers.

The limits, set in `launch/ufo.launch.yaml`:

| Axis | Velocity | Acceleration | Deceleration |
| --- | --- | --- | --- |
| Horizontal | 3.0 m/s | 1.5 m/s² | 2.5 m/s² |
| Vertical | 1.5 m/s | 1.0 m/s² | 1.5 m/s² |
| Yaw | 1.5 rad/s | 2.0 rad/s² | 3.0 rad/s² |

## Building the world and the map

`worlds/nathan_benderson_town.sdf` was built by `tools/prepare_world.py` from [disaster_world](https://github.com/juanscelyg/disaster_world). It keeps only the ground, the roads and the buildings:
- **Left out:** the vehicles, dumpsters, victims, trees and ground station, and the heaviest buildings.
- **Water:** the animated water is a flat plane.
- **Models:** every model is local, so nothing is downloaded at startup.
- **Textures:** the park keeps only its albedo textures, at 1024 px, because rendering is what slows Gazebo down here.

`maps/nathan_benderson_town.pcd`, the cloud the Bonxai map is built from, was made with `tools/aerial_map_builder.py` while the simulation was running. The tool teleports the UFO over a grid at several heights and turns it to four headings. It puts the clouds of both lidars together in the world frame, keeping one point per voxel (`--voxel`).

The usage of each tool is at the top of its file.

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | `gazebo_sim` (the world and the UFO), `world` (Gazebo with a world) and `ufo` (spawns the UFO and its flight controller in a running simulation), in YAML |
| `config/bridge/` | ROS–Gazebo bridge topics |
| `maps/` | `nathan_benderson_town.pcd` |
| `worlds/`, `models/` | The town world and its models |
| `scripts/` | The flight controller |
| `tools/` | The world and map tools |

## Authors and licensing

The launchers, bridge configuration, flight controller, tools and map were developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), under Apache-2.0; see [LICENSE](./LICENSE). The package includes third-party work under its own licenses:

- **The world:** the town world (`worlds/nathan_benderson_town.sdf`) comes from Juan Sebastián Cely Gutierrez's [disaster_world](https://github.com/juanscelyg/disaster_world), under Apache-2.0, cut down as described above.
- **The buildings:** the building models (`models/`, except `nathan_benderson_park/` and `lake_water/`), by Open Robotics, Nate Koenig and Louise Poubel, come from Gazebo Fuel, under CC0-1.0; see [LICENSE-CC0](./LICENSE-CC0).
- **The park:** the park (`models/nathan_benderson_park/`), by Cole Biesemeyer (Open Robotics), comes from Gazebo Fuel, under CC-BY-4.0; see [LICENSE-CC-BY-4.0](./LICENSE-CC-BY-4.0). Here it has fewer and smaller textures and no trees.
