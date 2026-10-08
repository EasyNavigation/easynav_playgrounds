<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav TIAGo Playground: worlds

Gazebo simulation (Harmonic or newer) of PAL Robotics' TIAGo in the AWS RoboMaker small house: the world and its models, the maps of the house, and the launchers that start Gazebo and spawn TIAGo (robot state publisher, ros2_control controllers and ROS–Gazebo bridge). It does not depend on EasyNav: `easynav_playground_tiago` adds the navigation.

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_tiago_worlds
source install/setup.bash
```

## Simulation

To run Gazebo and TIAGo without EasyNav or RViz2:

```bash
ros2 launch easynav_playground_tiago_worlds gazebo_sim.launch.yaml
```

`tiago.launch.yaml` spawns TIAGo in a running simulation (`x`, `y`, `z`, `Y` set the pose). The arm starts tucked, in PAL's `home` posture.

### Topics

| Topic | Type | Content |
| --- | --- | --- |
| `/scan_raw` | `sensor_msgs/LaserScan` | Base laser (SICK TIM571) |
| `/scan_raw/points` | `sensor_msgs/PointCloud2` | Base laser, as a cloud |
| `/sonar_base` | `sensor_msgs/LaserScan` | Base sonars |
| `/head_front_camera/rgb/image_raw`, `.../rgb/camera_info` | `sensor_msgs/Image`, `CameraInfo` | Head camera (Orbbec Astra), color |
| `/head_front_camera/depth_registered/image_raw`, `.../points` | `sensor_msgs/Image`, `PointCloud2` | Head camera, depth |
| `/imu_sensor_broadcaster/imu` | `sensor_msgs/Imu` | Base IMU |
| `/ft_sensor_controller/wrench` | `geometry_msgs/WrenchStamped` | Wrist force/torque sensor |
| `/mobile_base_controller/odom` | `nav_msgs/Odometry` | Wheel odometry (also the `odom` → `base_footprint` TF) |
| `/joint_states`, `/tf`, `/tf_static` | | Joints and TF |

Gazebo publishes every message of the RGBD camera with the same frame, `head_front_camera_rgb_frame` (x forward), which is right for the point cloud. To project images with `camera_info`, use the optical frame `head_front_camera_rgb_optical_frame`.

### Moving the robot

The base listens on `/mobile_base_controller/cmd_vel` (`geometry_msgs/TwistStamped`), for instance with the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/mobile_base_controller/cmd_vel
```

Without commands, the base controller warns once a second that it is braking ("Velocity command timed out"); this is expected.

The torso, head, arm and gripper have `joint_trajectory_controller`s: `torso_controller`, `head_controller`, `arm_controller` and `gripper_controller`. Each takes `FollowJointTrajectory` goals on `<controller>/follow_joint_trajectory`, or a trajectory on `<controller>/joint_trajectory`. For example, to look down:

```bash
ros2 topic pub --once /head_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [head_1_joint, head_2_joint], points: [{positions: [0.0, -0.6], time_from_start: {sec: 1}}]}"
```

## Maps

The costmap (`maps/home2.yaml`, 5 cm) is the house map shared with `easynav_playground_kobuki`. The other two maps were generated with NavMap's `navmap_tools`:

1. `home2_20cm.navmap`, the flat NavMap: the costmap resampled to 20 cm cells, which keeps the NavMap filters and the AMCL within their cycles.

   ```bash
   ros2 run navmap_tools navmap_resample maps/home2.yaml maps/home2_20cm.navmap 0.2
   ```

2. `house.pcd`, the Bonxai 3D map: `navmap_map_builder` teleports TIAGo through the free space of the house and puts together the head camera's and the laser's clouds at the ground-truth poses, turning to 8 headings at each position (the camera's field of view is narrow). With the simulation up and TIAGo looking down, so that the camera sees the floor and the low obstacles near the robot:

   ```bash
   ros2 launch easynav_playground_tiago_worlds gazebo_sim.launch.yaml gui:=false
   ros2 topic pub --once /head_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
     "{joint_names: [head_1_joint, head_2_joint], points: [{positions: [0.0, -0.6], time_from_start: {sec: 1}}]}"
   ros2 run navmap_tools navmap_map_builder /tmp/house --world default --model tiago \
     --cloud-topic /head_front_camera/depth_registered/points /scan_raw/points \
     --headings 8 --clearance 0.6 --robot-cut 0.6 --spawn-z 0.05
   ```

   Then copy `/tmp/house.pcd` to `maps/`.

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | `gazebo_sim` (world and TIAGo), `world` (Gazebo with the house), `tiago` (spawns TIAGo in a running simulation), in YAML |
| `config/bridge/` | ROS–Gazebo bridge topics |
| `maps/` | `home2.yaml` (occupancy grid, 5 cm), `home2_20cm.navmap` (NavMap) and `house.pcd` (Bonxai) |
| `worlds/`, `models/`, `photos/` | Small house world |

## Authors and licensing

The launchers, bridge configuration and maps were developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), under Apache-2.0; see [LICENSE](./LICENSE). The bridge topics come from PAL Robotics' `tiago_bringup` (Copyright (c) 2022-2024 PAL Robotics S.L., Apache-2.0).

The small house world (`worlds/`, `models/`, `photos/`) comes from [aws-robomaker-small-house-world](https://github.com/IntelligentRoboticsLabs/aws-robomaker-small-house-world) (`ros2` branch), under the MIT No Attribution license; see [models/LICENSE](./models/LICENSE).
