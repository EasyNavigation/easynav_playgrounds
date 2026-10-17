<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Summit Playground: worlds

Gazebo Harmonic simulation of a Robotnik Summit XL in three worlds, the URJC excavation (an outdoor 3D terrain), a small indoor warehouse and a parking garage with three floors linked by ramps: the worlds and their models, their maps, and the launchers that start Gazebo and spawn the Summit XL (robot state publisher, ros2_control controllers and ROS–Gazebo bridge). It does not depend on EasyNav: `easynav_playground_summit` adds the navigation.

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

## The excavation's stitched tiles

The excavation's terrain is a set of photogrammetry tiles (about 30 x 38 m) that share no vertices: each one ends in a vertical skirt under the ground, and the rims of neighbouring tiles were at slightly different heights. Each seam was a step, up to 0.5 m, that could stop the Summit XL. Both meshes (`models/urjc_excavation/meshes/URJC_excavation.stl`, the collision mesh, and `URJC_excavation.dae`, the visual one) have their tiles stitched by `tools/stitch_rims.py`, run twice: the rim vertices of different tiles closer than 0.35 m meet halfway, in height and across, when they differ by less than 0.5 m. Nothing else moves. Seam steps over 0.15 m went from 487 to 225 (0.1 m cells of the top surface); the one at (-33.7, 11.7), which stopped the robot, is gone.

## Building the excavation maps

`maps/excavation_urjc_world.pcd` samples the world's collision mesh (`models/urjc_excavation/meshes/URJC_excavation.stl`) uniformly, 20 points/m², in the map frame: the mesh is raised 5.77 m so that z = 0 is the terrain under the spawn point. `maps/excavation_urjc.navmap` meshes that cloud with `navmap_ros::from_points` at 1 m, dropping surfaces smaller than 4 m². Against the collision mesh its height error is 1 cm (median), 6 cm (p90). `maps/excavation_urjc.pcd`, the cloud the Bonxai map is built from, is unchanged.

## Building the parking garage maps

The parking garage maps come from simulated 3D lidar scans of the world's collision mesh (`models/parking_garage/meshes/parking_garage.dae`), so faces hidden inside the model (the floor under the ramps) never enter them.
- **The scans:** a VLP-16-like sensor 0.8 m above the ground, every 2 m on floors 0–2 (the roof is not reachable), 2 cm range noise.
- **`maps/parking_garage.pcd`:** keeps one point per 0.1 m voxel (the points' centroid), in the map frame with z = 0 on the ground floor under the spawn point (the mesh is lowered 0.078 m).
- **`maps/parking_garage.navmap`:** meshes that cloud with `navmap_ros::from_points` at 0.4 m. Floors 0–2 are one surface that overlaps itself and connects through the ramps.
- **`maps/parking_garage_bonxai.pcd`:** keeps one point per occupied 0.3 m voxel.

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
| `maps/` | Excavation (`excavation_urjc.navmap`, `.pcd`, `excavation_urjc_world.pcd`) and warehouse (`warehouse.yaml`, `warehouse_20cm.navmap`, `warehouse.pcd`) maps |
| `worlds/`, `models/` | The two worlds and their models |

## Authors and licensing

The launchers, bridge configuration and maps were developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), under Apache-2.0; see [LICENSE](./LICENSE). It includes third-party worlds under their own licenses:

- The URJC excavation world (`worlds/urjc_excavation.world`, `models/urjc_excavation/`) comes from [urjc-excavation-world](https://github.com/juanscelyg/urjc-excavation-world) (`main` branch). Its license file is GPL-3.0; see [models/urjc_excavation/LICENSE](./models/urjc_excavation/LICENSE). Its meshes are modified here: their tiles are stitched (see above).
- The small warehouse world (`worlds/small_warehouse.world`, `models/aws_robomaker_warehouse_*`) comes from [aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) (`ros1` branch), under the MIT-0 license; see [models/LICENSE-aws-robomaker](./models/LICENSE-aws-robomaker). Changes for Gazebo Sim: the roof was removed (the lidar and the GUI camera see inside), the Gazebo Sim system plugins, spherical coordinates and a sun were added, the physics step is 5 ms, and the floor model's (`GroundB`) invalid inertia was fixed.
- The warehouse maps (`maps/warehouse.*`) were generated from that world with `navmap_tools`.
- The parking garage world (`worlds/parking_garage*.world`, `worlds/parking_garage.sdf`, `models/parking_garage/`, `models/parking_garage_ground_plane/` and the `models/*_cube/` models) comes from [parking-garage-world](https://github.com/juanscelyg/parking-garage-world). Its license file is GPL-3.0; see [models/parking_garage/LICENSE](./models/parking_garage/LICENSE). The garage model is based on a model by michaelwjl on [free3d.com](https://free3d.com/3d-model/paking-lot-182.html).
