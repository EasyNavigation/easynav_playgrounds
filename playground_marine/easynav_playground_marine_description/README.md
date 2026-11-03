<!--
Copyright 2026 José Miguel Guerrero
SPDX-License-Identifier: Apache-2.0
-->

# easynav_playground_marine_description

The vehicles of the [marine playground](../easynav_playground_marine), as Gazebo models. They need
neither EasyNav nor ROS.

| Model | What it is |
| --- | --- |
| `models/marine_wamv/` | The WAM-V of Gazebo Fuel (OpenRobotics `wam-v`, VRX's 16 ft catamaran) with the changes that make it float and go straight in the world's graded buoyancy: the floats are boxes (that plugin supports only boxes and spheres) and the two joints that let each engine swing about the vertical are fixed. `wamv.sdf.in` adds a thruster on each propeller, drag and an odometry publisher; it steers with the difference of the thrusts |
| `models/marine_tethys/` | `tethys.sdf.in`: the MBARI Tethys LRAUV of Gazebo Fuel (`accurrent/MBARI Tethys LRAUV`) with the plugins of the gz-sim demo `auv_controls.sdf`: fin position controllers, propeller thruster, fin lift and drag, hydrodynamics |

The `*.sdf.in` files are snippets for `<world>`: `@X@ @Y@ @Z@ @YAW@` (and `@MODELS@`, the directory of
the models) are filled in by `marine_sim.launch.py` of the worlds package, which also adds the model's
name to the `<enable>` list of the world's Buoyancy plugin. The meshes come from Gazebo Fuel the first
time (internet).

## License

Apache-2.0 for what is here. The meshes and the original models belong to their authors on Gazebo Fuel:
check their licenses there before redistributing.
