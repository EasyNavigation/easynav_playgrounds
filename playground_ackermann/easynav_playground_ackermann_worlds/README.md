<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Ackermann Playground: worlds

Gazebo Harmonic simulation of a Toyota Prius on the Sonoma Raceway. It includes the world and its model, the NavMap of the track, and the launchers that start Gazebo and spawn the Prius with its robot state publisher, its ros2_control controllers and the ROS–Gazebo bridge. It does not depend on EasyNav: `easynav_playground_ackermann` adds the navigation.

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_ackermann_worlds
source install/setup.bash
```

## Simulation

To run Gazebo and the Prius without EasyNav or RViz2:

```bash
ros2 launch easynav_playground_ackermann_worlds gazebo_sim.launch.yaml
```

The car starts on the start line, facing along the track. Its spawn pose is set with the `x`, `y`, `z` and `yaw` arguments. To spawn the car in a running simulation, use `prius.launch.yaml`.

The simulated Prius publishes:

| Topic | Type |
| --- | --- |
| `/top_laser/points` | `sensor_msgs/PointCloud2` (3D lidar on the roof) |
| `/front_laser/scan` | `sensor_msgs/LaserScan` (2D lidar on the front bumper) |
| `/front_camera/image`, `/front_camera/depth_image` | `sensor_msgs/Image` |
| `/front_camera/points` | `sensor_msgs/PointCloud2` |
| `/front_camera/camera_info` | `sensor_msgs/CameraInfo` |
| `/imu/data` | `sensor_msgs/Imu` |
| `/gps/fix` | `sensor_msgs/NavSatFix` |
| `/ground_truth` | `nav_msgs/Odometry` |

It also publishes TF, with the controller's odometry as `odom` → `base_footprint`. It listens on `/cmd_vel` (`geometry_msgs/Twist`). A `twist_stamper` forwards that command to the Ackermann steering controller, which takes `TwistStamped` (speed and yaw rate) on `/prius_base_control/reference`.

## Building the track map

`maps/sonoma_track.navmap` is a NavMap of the track's surface, built in two steps from the raceway's mesh (`models/sonoma_raceway/`):

1. `tools/track_cloud.py` samples the drivable surface (the track and the start line) into a point cloud in the world frame. `navmap_ros::from_points` then meshes that cloud into a NavMap.
2. `tools/mark_obstacles.py` adds an `occupancy` layer, where the cells under the walls are lethal. The walls are the steep faces of the barriers. The bridge over the track is higher than the car, so it is not marked as an obstacle.

The usage of each tool is at the top of its file.

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | `gazebo_sim` (the world and the Prius), `world` (Gazebo with a world) and `prius` (spawns the Prius in a running simulation), in YAML |
| `config/bridge/` | ROS–Gazebo bridge topics |
| `maps/` | `sonoma_track.navmap` |
| `worlds/`, `models/` | The Sonoma Raceway world and its model |
| `tools/` | The map tools |

## Authors and licensing

The launchers, bridge configuration, tools and maps were developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), under Apache-2.0; see [LICENSE](./LICENSE). The package includes third-party work under its own licenses:

- **The world:** the Sonoma Raceway world (`worlds/sonoma_raceway.sdf`), by Juan Sebastián Cely Gutierrez, comes from [easynav_wheels_worlds](https://github.com/juanscelyg/easynav_wheels_worlds), under the GPL-3.0-only license; see [worlds/LICENSE](./worlds/LICENSE).
- **The model:** the Sonoma Raceway model (`models/sonoma_raceway/`) is Open Robotics' model in Gazebo Fuel, under CC0-1.0; see [models/sonoma_raceway/LICENSE](./models/sonoma_raceway/LICENSE).
