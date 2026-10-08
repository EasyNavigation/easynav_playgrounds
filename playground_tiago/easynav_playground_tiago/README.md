<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav TIAGo Playground

EasyNav configurations for PAL Robotics' TIAGo in a Gazebo simulation (Harmonic or newer) of the AWS RoboMaker small house. The simulated TIAGo has its mobile base (pmb2), lifting torso, pan-tilt head with an RGBD camera, 7-DoF arm with the PAL gripper and wrist force/torque sensor, base laser, sonars and IMU.

The TIAGo playground is made of three packages:

| Package | Contents |
| --- | --- |
| `easynav_playground_tiago_description` | TIAGo's model: URDF, meshes and ros2_control controllers. No Gazebo, no EasyNav |
| `easynav_playground_tiago_worlds` | The Gazebo simulation: the small house world, its maps, and the launchers that spawn TIAGo with its controllers and the ROS–Gazebo bridge. No EasyNav |
| `easynav_playground_tiago` | The EasyNav configurations and launchers |

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_tiago
source install/setup.bash
```

## Launch EasyNav

Each `easynav_<config>.launch.yaml` starts Gazebo, TIAGo, EasyNav with that configuration and RViz2:

```bash
ros2 launch easynav_playground_tiago easynav_costmap_rpp.launch.yaml
```

Once RViz2 is up, send a goal with the **2D Goal Pose** tool.

| Launch file | Maps | Localizer | Planner | Controller |
| --- | --- | --- | --- | --- |
| `easynav_costmap_rpp.launch.yaml` | Costmap from `home2.yaml` | Costmap AMCL with the base laser | Costmap | Regulated Pure Pursuit |
| `easynav_navmap_bonxai_amcl.launch.yaml` | NavMap `home2_20cm.navmap` and Bonxai `house.pcd` | NavMap AMCL against the Bonxai map, with the base laser and the head camera | NavMap A* | Regulated Pure Pursuit |

The maps are in `easynav_playground_tiago_worlds`. The parameters are in `params/costmap.rpp.params.yaml` and `params/navmap.bonxai.amcl.params.yaml`.

### Launch arguments

| Argument | Default | Description |
| --- | --- | --- |
| `params_file` | Per launch file | EasyNav parameters file |
| `rviz_config` | Per launch file | RViz2 configuration file |
| `gui` | `true` | Set to `false` to run Gazebo headless |
| `rviz` | `true` | Set to `false` to skip RViz2 |

For example, to run headless with your own parameters:

```bash
ros2 launch easynav_playground_tiago easynav_costmap_rpp.launch.yaml gui:=false params_file:=/path/to/my.params.yaml
```

### Notes for other configurations

- TIAGo's base laser is 0.095 m above the floor. EasyNav's localizers and obstacle filters drop points below `min_height` (0.1 m by default) as floor hits, so these configurations set `min_height: 0.05`. Do the same in your own parameters, or the laser is ignored.
- The base takes `geometry_msgs/TwistStamped`, so EasyNav runs with `use_cmd_vel_stamped: true`, and the launch files remap `cmd_vel_stamped` to `/mobile_base_controller/cmd_vel` and `odom` to `/mobile_base_controller/odom`.

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | EasyNav launch files, in YAML |
| `params/` | EasyNav parameters, one file per configuration |
| `rviz/` | RViz2 configurations |

## Authors and licensing

This package was developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), and is licensed under Apache-2.0; see [LICENSE](./LICENSE). The TIAGo model is PAL Robotics' work: see `easynav_playground_tiago_description`.
