<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav Ackermann Playground: description

The model of the Toyota Prius used by the EasyNav playgrounds: URDF, meshes and the ros2_control controllers. It is a car with Ackermann steering, with a 3D lidar, a 2D lidar, an RGBD camera, an IMU and a GPS. It depends neither on Gazebo nor on EasyNav: `easynav_playground_ackermann_worlds` simulates it, and `easynav_playground_ackermann` navigates with it.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_ackermann_description
source install/setup.bash
```

To get the URDF:

```bash
xacro $(ros2 pkg prefix easynav_playground_ackermann_description)/share/easynav_playground_ackermann_description/urdf/prius.urdf.xacro
```

## Package layout

| Directory | Contents |
| --- | --- |
| `urdf/` | `prius.urdf.xacro` and its parts (base, axles, wheels, steering wheel, sensors, ros2_control) |
| `meshes/` | Meshes of the car |
| `config/` | ros2_control controllers (`prius_controllers.yaml`) |

## Authors and licensing

The Prius description (`urdf/`, `config/`), by Juan Sebastián Cely Gutierrez, comes from [prius_autonomous_car](https://github.com/juanscelyg/prius_autonomous_car), under the GPL-3.0-only license; see [LICENSE-GPL](./LICENSE-GPL). The meshes (`meshes/`) come from Open Robotics' Prius Hybrid model in Gazebo Fuel, under CC0-1.0; see [LICENSE](./LICENSE).

It was adapted for these simulations by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos). Only the files the simulation uses were copied. Changes to the robot model, all for the simulation:

- Paths point to this package, and the meshes are referenced with `file://`, so Gazebo finds them without extra resource paths.
- `base_footprint` is on the ground under the rear axle, the reference point of the Ackermann kinematics.
- The steering is limited to ±0.6 rad, a real car's lock. The controllers' track widths are 1.572 m (front) and 1.472 m (rear).
- The original camera was replaced by an RGBD camera on the front of the roof, over the windshield, where autonomous cars carry it.
- The IMU reports its orientation in east-north-up coordinates and has its own frame, and the GPS runs at 10 Hz.
