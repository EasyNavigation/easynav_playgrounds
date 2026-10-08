<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Summit Playground: description

The model of the Robotnik Summit XL used by the EasyNav playgrounds: URDF, meshes and the ros2_control controllers, with a 3D lidar, a depth camera, an IMU and a GPS. It depends neither on Gazebo nor on EasyNav: `easynav_playground_summit_worlds` simulates it, and `easynav_playground_summit` navigates with it.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_summit_description
source install/setup.bash
```

To get the URDF:

```bash
xacro $(ros2 pkg prefix easynav_playground_summit_description)/share/easynav_playground_summit_description/urdf/summit_xl.urdf.xacro
```

## Package layout

| Directory | Contents |
| --- | --- |
| `urdf/` | `summit_xl.urdf.xacro` and its parts (base, wheels, sensors, ros2_control) |
| `meshes/` | Meshes of the base, wheels, structures and sensors |
| `config/` | ros2_control controllers (`summit_controllers.yaml`) |

## Authors and licensing

The Summit XL model is Robotnik Automation's: Copyright (c) 2023, Robotnik Automation S.L. The Summit XL model (`urdf/`, `meshes/`, `config/`) comes from [robot_description](https://github.com/Summit-Harmonic/robot_description) and [robotnik_sensors](https://github.com/Summit-Harmonic/robotnik_sensors) (`ros2-devel` branch), under the BSD 3-Clause license; see [LICENSE](./LICENSE).

It was adapted for these simulations by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos). Only the files the simulations use were copied. Changes to the robot model, all for the simulation:

- Paths point to this package; the sensors are reduced to the ones the Summit XL carries, and the unused omni-wheel option was dropped.
- Collisions are boxes and cylinders instead of meshes (mesh-vs-terrain contacts overflowed the physics engine), and the wheels have lower sideways friction, so the skid steering turns as commanded (`wheel_separation_multiplier` in `config/summit_controllers.yaml` is calibrated against Gazebo's ground truth).
- The ZED's two RGB cameras are not simulated (nothing uses them); the depth camera runs at quarter resolution and 10 Hz.
- The IMU has noise and its frame id, the GPS runs at 5 Hz, and the odometry has no zero variances: filters need realistic covariances.
