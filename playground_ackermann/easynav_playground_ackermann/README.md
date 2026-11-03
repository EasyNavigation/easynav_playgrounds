<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Ackermann Playground

EasyNav configurations for a Toyota Prius, a car with Ackermann steering, on the Sonoma Raceway in Gazebo Harmonic. The car is localized with GPS, IMU and odometry, plans with A* over a NavMap of the track and follows the path with RPP or MPC.

The Ackermann playground is made of three packages:

| Package | Contents |
| --- | --- |
| `easynav_playground_ackermann_description` | The Prius model: URDF, meshes and ros2_control controllers. No Gazebo, no EasyNav |
| `easynav_playground_ackermann_worlds` | The Gazebo simulation: the Sonoma Raceway, its map, and the launchers that spawn the Prius with its controllers and the ROS–Gazebo bridge. No EasyNav |
| `easynav_playground_ackermann` | The EasyNav configurations and launchers |

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_ackermann
source install/setup.bash
```

## Launch EasyNav

```bash
ros2 launch easynav_playground_ackermann easynav_rpp.launch.yaml   # Gazebo, the Prius, EasyNav with RPP and RViz2
ros2 launch easynav_playground_ackermann easynav_mpc.launch.yaml   # The same with MPC
```

Send goals on the track with RViz2's 2D Goal Pose. The car starts on the start line, facing along the track.

| Configuration | Parameters | Controller |
| --- | --- | --- |
| RPP | `params/rpp.params.yaml` | Regulated Pure Pursuit |
| MPC | `params/mpc.params.yaml` | MPC with the Ackermann model: it keeps the curvature within the minimum turning radius (4.2 m) and slows down before curves |

Both share the rest of the configuration:
- **Localization:** FusionLocalizer fuses the GPS (in east-north-up coordinates, from the GPS antenna's position), the IMU and the wheel odometry. No AMCL: a race track has few features for a lidar.
- **Map:** the NavMap of the track (`maps/sonoma_track.navmap` in `easynav_playground_ackermann_worlds`), with the walls as obstacles and a wide inflation, so that the cheapest path runs along the middle of the track.
- **Planner:** A* over that NavMap.
- **Limits:** the car's limits in `robot_limits`, with `min_turning_radius` for its steering.

Other launch arguments, `params_file`, `rviz_config`, `world`, `gui` (Gazebo GUI) and `rviz`, can also be overridden.

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | EasyNav launch files, in YAML |
| `params/` | EasyNav parameters (RPP and MPC) |
| `rviz/` | RViz2 configuration: the map, the path, the lidar, the camera, and a view that follows the car |

## Authors and licensing

This package was developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), and is licensed under Apache-2.0; see [LICENSE](./LICENSE). The car and the race track come from third parties: see `easynav_playground_ackermann_description` and `easynav_playground_ackermann_worlds`.
