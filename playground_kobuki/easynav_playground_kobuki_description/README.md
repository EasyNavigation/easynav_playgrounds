<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Kobuki Playground: description

The model of the Kobuki (Turtlebot 2) used by the EasyNav playgrounds: URDF and meshes, with a laser and an optional RGBD camera. It depends neither on Gazebo nor on EasyNav: `easynav_playground_kobuki_worlds` simulates it, and `easynav_playground_kobuki` navigates with it.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_kobuki_description
source install/setup.bash
```

To get the URDF:

```bash
xacro $(ros2 pkg prefix easynav_playground_kobuki_description)/share/easynav_playground_kobuki_description/urdf/kobuki.urdf.xacro
```

## Package layout

| Directory | Contents |
| --- | --- |
| `urdf/` | `kobuki.urdf.xacro` and its parts (base, stacks, sensors) |
| `meshes/` | Meshes of the base, the stacks and the sensors |

## Authors and licensing

The Kobuki model (`urdf/`, `meshes/`) is Yujin Robot's: Copyright (c) 2012, Yujin Robot. It comes from [kobuki_ros](https://github.com/kobuki-base/kobuki_ros), through the [Juancams/kobuki_ros](https://github.com/Juancams/kobuki_ros) fork (`rolling-simulation` branch), under the BSD 3-Clause license; see [LICENSE](./LICENSE). Only the files these simulations use were copied, and their paths were changed to point to this package, by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos).
