<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Aerial Playground

EasyNav configuration for an aerial robot, a 1 m flying saucer, in a town in Gazebo Harmonic. The UFO moves in x, y, z and yaw. It plans 3D paths over a Bonxai map of the town, keeping its distance from buildings and the ground, and follows them with MPC.

The aerial playground is made of three packages:

| Package | Contents |
| --- | --- |
| `easynav_playground_aerial_description` | The UFO model: URDF and meshes. No Gazebo, no EasyNav |
| `easynav_playground_aerial_worlds` | The Gazebo simulation: the town, its 3D map, the flight controller, and the launchers that spawn the UFO with the ROS–Gazebo bridge. No EasyNav |
| `easynav_playground_aerial` | The EasyNav configuration and launchers |

## Supported ROS 2 distributions

This playground needs Gazebo Harmonic or newer, so it runs on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only this simulation does not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_aerial
source install/setup.bash
```

## Launch EasyNav

```bash
ros2 launch easynav_playground_aerial easynav_mpc.launch.yaml   # Gazebo, the UFO, EasyNav and RViz2
ros2 launch easynav_playground_aerial ufo_sim.launch.yaml       # Only Gazebo, the UFO and RViz2, to fly it by hand
```

RViz2's 2D Goal Pose puts goals at z = 0, on the ground. To send goals in the air, use the 3D Goal Pose tool of `navmap_rviz_plugin` (shortcut `h`), which is in the toolbar of this playground's RViz2 configuration:
- A left click places the goal on the line from the camera through the cursor, 10 m away by default.
- The mouse wheel changes that distance.
- Dragging sets the goal's yaw.
- A line down to the ground and a cross where it falls show where the goal is.

The configuration (`params/mpc.params.yaml`):
- **Localization:** FusionLocalizer in 3D, fusing the GPS (in east-north-up coordinates), the IMU and the odometry.
- **Map:** the Bonxai map of the town (`maps/nathan_benderson_town.pcd` in `easynav_playground_aerial_worlds`), at 0.5 m.
- **Planner:** the Bonxai planner. It treats buildings as solid blocks down to the ground (`fill_below: highest`), keeps at least 2 m from them and prefers 5 m, and flies no higher than 70 m.
- **Controller:** MPC with the aerial model. It follows the 3D path within the UFO's horizontal and vertical limits.

`gui` (Gazebo GUI) and `rviz` can be set to false; `params_file` can also be overridden.

## Package layout

| Directory | Contents |
| --- | --- |
| `launch/` | EasyNav and simulation launch files, in YAML |
| `params/` | EasyNav parameters |
| `rviz/` | RViz2 configurations: `ufo_easynav.rviz` (the map, the paths and the 3D Goal Pose tool) and `ufo.rviz` (a third-person view that follows the UFO) |

## Authors and licensing

This package was developed by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), and is licensed under Apache-2.0; see [LICENSE](./LICENSE). The town comes from third parties: see `easynav_playground_aerial_worlds`.
