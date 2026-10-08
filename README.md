<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav PlayGrounds

Complete Gazebo simulations integrated with [EasyNav](https://github.com/EasyNavigation/EasyNavigation): a robot, its worlds, their maps and ready-to-run EasyNav configurations. They are the reference configurations of EasyNav, and a good starting point for your own robot's parameters.

Each playground is split in three packages, so that the robot model and the simulation can be used without EasyNav:

| Package | Contents | Depends on |
| --- | --- | --- |
| `easynav_playground_<robot>_description` | The robot model: URDF, meshes and controllers | Neither Gazebo nor EasyNav |
| `easynav_playground_<robot>_worlds` | The Gazebo simulation: worlds, their maps, and the launchers that spawn the robot with its controllers and the ROS–Gazebo bridge | Gazebo, not EasyNav |
| `easynav_playground_<robot>` | The EasyNav configurations and launchers | EasyNav |

| Directory | Robot and worlds | EasyNav configurations |
| --- | --- | --- |
| [`playground_kobuki`](playground_kobuki/easynav_playground_kobuki) | Kobuki (Turtlebot 2) in a small house (**indoor reference**) | Costmap (RPP, MPPI, MPC, SeReST, MH-AMCL, routes, safety mode), Simple, mapping, multirobot |
| [`playground_summit`](playground_summit/easynav_playground_summit) | Robotnik Summit XL in an outdoor excavation and an indoor warehouse (**outdoor reference**) | NavMap and Bonxai (RPP, MPPI, MPC), GPS fusion, warehouse logistics |
| [`playground_tiago`](playground_tiago/easynav_playground_tiago) | PAL Robotics' TIAGo in a small house | Costmap with RPP; NavMap and Bonxai with the laser and the RGBD camera |
| [`playground_omni`](playground_omni/easynav_playground_omni) | Three- to six-wheel omnidirectional robots in two mazes | Costmap with RPP |

For example, with the Kobuki:

```bash
ros2 launch easynav_playground_kobuki easynav_costmap_rpp.launch.yaml        # Gazebo, the Kobuki, EasyNav and RViz2
ros2 launch easynav_playground_kobuki_worlds gazebo_sim.launch.yaml          # Only Gazebo and the Kobuki
```

## Supported ROS 2 distributions

The playgrounds need Gazebo Harmonic or newer, so they run on Jazzy and later distributions, but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on Humble: only these simulations do not.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_kobuki   # or any other playground
source install/setup.bash
```

## Licensing

Each package declares its licenses in its `package.xml` and explains them in its README. The EasyNav configurations and launchers are by the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), under Apache-2.0. The robot models and the worlds come from third parties under their own licenses: Yujin Robot's Kobuki (BSD-3-Clause), Robotnik's Summit XL (BSD-3-Clause), PAL Robotics' TIAGo (Apache-2.0), Yohan Prakoso's omnidirectional robots (MIT), AWS RoboMaker's small house and small warehouse (MIT-0) and the URJC excavation (GPL-3.0).
