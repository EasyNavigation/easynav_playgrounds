<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Summit Playground: worlds

Gazebo Harmonic simulation of a Robotnik Summit XL in two worlds, the URJC excavation (an outdoor 3D terrain) and a small indoor warehouse: the worlds and their models, their maps, and the launchers that start Gazebo and spawn the Summit XL (robot state publisher, ros2_control controllers and ROS–Gazebo bridge). It does not depend on EasyNav: `easynav_playground_summit` adds the navigation.

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_summit_worlds
source install/setup.bash
```

## Simulation

To run Gazebo and the Summit XL without EasyNav or RViz2:

```bash
ros2 launch easynav_playground_summit_worlds gazebo_sim.launch.yaml
```

To spawn the robot in a running simulation, use `summit.launch.yaml` with a spawn pose (`x`, `y`, `z`).

The simulated Summit XL publishes:

| Topic | Type |
| --- | --- |
| `/front_laser/points` | `sensor_msgs/PointCloud2` (Robosense Helios 16P) |
| `/front_camera/depth/color/points` | `sensor_msgs/PointCloud2` (ZED2) |
| `/imu/data` | `sensor_msgs/Imu` |
| `/gps/fix` | `sensor_msgs/NavSatFix` |
| `/robotnik_base_control/odom` | `nav_msgs/Odometry` |
| `/ground_truth` | `nav_msgs/Odometry` |

It also publishes TF, and listens on `/cmd_vel` (`geometry_msgs/Twist`). `twist_stamper` forwards it to the diff drive controller, which takes `TwistStamped` on `/robotnik_base_control/cmd_vel`.

## Building the warehouse maps

The warehouse maps were generated from the simulation with two tools of NavMap's `navmap_tools`:

1. `navmap_map_builder` builds the 3D cloud (`warehouse.pcd`, for Bonxai). It teleports the robot through the free space (`gz service .../set_pose`) and puts its lidar scans together at the ground-truth poses: a new position is taken when ground was seen around it and no obstacle is within `--clearance`.
2. `navmap_map2d_from_pcd` builds the 2D occupancy grid (`warehouse.pgm`/`.yaml`, for the flat NavMap) from that cloud: points between 0.1 and 1.2 m high are obstacles, free space is flood-filled from a seed inside the walls, and the rest is unknown.

To regenerate them (or build maps of another world), start the simulation without EasyNav and run the tools:

```bash
ros2 launch easynav_playground_summit_worlds gazebo_sim.launch.yaml gui:=false \
  world:=$(ros2 pkg prefix easynav_playground_summit)/share/easynav_playground_summit/worlds/small_warehouse.world
ros2 run navmap_tools navmap_map_builder /tmp/warehouse --world warehouse --model summit_xl \
  --cloud-topic /front_laser/points --ground-truth-topic /ground_truth
ros2 run navmap_tools navmap_map2d_from_pcd /tmp/warehouse.pcd /tmp/warehouse
```

Then copy `warehouse.pcd`, `warehouse.pgm` and `warehouse.yaml` to `maps/`. Run each tool with `--help` for its options (seed, resolution, height band, topics, headings).

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | `gazebo_sim` (a world and the Summit XL), `world` (Gazebo with a world), `summit` (spawns the Summit XL in a running simulation), in YAML |
| `config/bridge/` | ROS–Gazebo bridge topics |
| `maps/` | Excavation (`excavation_urjc.navmap`, `.pcd`) and warehouse (`warehouse.yaml`, `warehouse_20cm.navmap`, `warehouse.pcd`) maps |
| `worlds/`, `models/` | The two worlds and their models |

## Authors and licensing

The launchers, bridge configuration and maps were developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), under Apache-2.0; see [LICENSE](./LICENSE). It includes third-party worlds under their own licenses:

- The URJC excavation world (`worlds/urjc_excavation.world`, `models/urjc_excavation/`) comes from [urjc-excavation-world](https://github.com/juanscelyg/urjc-excavation-world) (`main` branch). Its license file is GPL-3.0; see [models/urjc_excavation/LICENSE](./models/urjc_excavation/LICENSE).
- The small warehouse world (`worlds/small_warehouse.world`, `models/aws_robomaker_warehouse_*`) comes from [aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) (`ros1` branch), under the MIT-0 license; see [models/LICENSE-aws-robomaker](./models/LICENSE-aws-robomaker). Changes for Gazebo Sim: the roof was removed (the lidar and the GUI camera see inside), the Gazebo Sim system plugins, spherical coordinates and a sun were added, the physics step is 5 ms, and the floor model's (`GroundB`) invalid inertia was fixed.
- The warehouse maps (`maps/warehouse.*`) were generated from that world with `navmap_tools`.
