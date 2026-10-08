<!--
Copyright 2026 Intelligent Robotics Lab
SPDX-License-Identifier: Apache-2.0
-->

# EasyNav TIAGo Playground: description

The model of PAL Robotics' TIAGo used by the EasyNav playgrounds: URDF, meshes and the ros2_control controllers. It depends neither on Gazebo nor on EasyNav: `easynav_playground_tiago_worlds` simulates it, and `easynav_playground_tiago` navigates with it.

## Build

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to easynav_playground_tiago_description
source install/setup.bash
```

To get the URDF:

```bash
xacro $(ros2 pkg prefix easynav_playground_tiago_description)/share/easynav_playground_tiago_description/urdf/tiago.urdf.xacro
```

## The TIAGo model

`urdf/tiago.urdf.xacro` is PAL's TIAGo description expanded once with PAL's default configuration: base `pmb2`, arm `tiago-arm` with wrist `wrist-2010`, force/torque sensor `schunk-ft`, end effector `pal-gripper`, laser `sick-571` and camera `orbbec-astra`. It was expanded from the Gazebo Harmonic ports of PAL's packages ([Tiago-Harmonic](https://github.com/Tiago-Harmonic): `tiago_robot` 29211f4, `pmb2_robot` a9c09d2, `pal_gripper` b6eec0f), and only the meshes it uses were copied. The controller values (`config/tiago_controllers.yaml`) come from PAL's `tiago_controller_configuration`, `pmb2_controller_configuration` and `pal_gripper_controller_configuration`.

Changes from PAL's model, for this simulation:

- The meshes and the controllers file point to this package.
- The head camera has a `gz_frame_id`: without it, its point cloud carried Gazebo's internal sensor name as frame.
- The wheels' velocity command limit is the wheel joint's (10.15 rad/s, 1 m/s) instead of 1 rad/s (0.1 m/s), which made the base crawl.
- The arm and torso start in PAL's `home` posture (`initial_value` in ros2_control), instead of tucking the arm with play_motion2 at startup.

## Package layout

| Directory | Contents |
| --- | --- |
| `urdf/` | `tiago.urdf.xacro` |
| `meshes/` | Meshes of the base (pmb2), TIAGo and the PAL gripper |
| `config/` | ros2_control controllers (`tiago_controllers.yaml`) |
| `doc/` | PAL Robotics logo |

## Authors and licensing

This package was put together by Francisco Martín Rico at the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) (Universidad Rey Juan Carlos), and is licensed under Apache-2.0; see [LICENSE](./LICENSE).

It includes third-party work under its own copyright and licenses:

- **The TIAGo robot model** (`urdf/`, `meshes/`) and the values of its controllers (`config/`) are the work of [PAL Robotics](https://pal-robotics.com/): Copyright (c) 2022-2024 PAL Robotics S.L. All rights reserved. They are licensed under the Apache License 2.0 (see [LICENSE](./LICENSE)), and come from PAL's [tiago_robot](https://github.com/pal-robotics/tiago_robot), [pmb2_robot](https://github.com/pal-robotics/pmb2_robot) and [pal_gripper](https://github.com/pal-robotics/pal_gripper), through their Gazebo Harmonic ports at [Tiago-Harmonic](https://github.com/Tiago-Harmonic). TIAGo is a robot by PAL Robotics.

The PAL Robotics logo below is PAL Robotics' own, used as published in its [brand guidelines](https://pal-robotics.com/es/guia-marca-logotipo/), unmodified.

<p align="center">
  <a href="https://pal-robotics.com/">
    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="doc/pal_logo_white.svg">
      <img src="doc/pal_logo_midnight.svg" alt="PAL Robotics" width="200">
    </picture>
  </a>
</p>
