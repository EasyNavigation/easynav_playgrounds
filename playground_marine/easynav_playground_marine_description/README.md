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
| `models/marine_waves/` | The shaders and textures that draw the waves of the water (not a model): the Gerstner waves of the Waves model of Gazebo Fuel (OpenRobotics), with the transparency as a parameter. See its README |
| `models/marine_tethys/` | `tethys.sdf.in`: the MBARI Tethys LRAUV of Gazebo Fuel (`accurrent/MBARI Tethys LRAUV`) with the plugins of the gz-sim demo `auv_controls.sdf`: fin position controllers, propeller thruster, fin lift and drag, hydrodynamics |

The `*.sdf.in` files are snippets for `<world>`: `@X@ @Y@ @Z@ @YAW@` (and `@MODELS@`, the directory of
the models) are filled in by `marine_sim.launch.py` of the worlds package, which also adds the model's
name to the `<enable>` list of the world's Buoyancy plugin. The meshes come from Gazebo Fuel the first
time (internet).

## Licenses

* What is here is Apache-2.0 ([LICENSE](./LICENSE)), except `models/marine_wamv/model.sdf`.
* `models/marine_wamv/model.sdf` is a modified copy of the **WAM-V** of Open Robotics in Gazebo Fuel
  (`OpenRobotics/wam-v`, by Carlos Aguero and Cole Biesemeyer), under **CC-BY-4.0**
  ([LICENSE-CC-BY-4.0](./LICENSE-CC-BY-4.0)). The changes (boxes for the floats, fixed engine joints) are
  described at the top of the file.
* The **MBARI Tethys LRAUV** (`accurrent/MBARI Tethys LRAUV`, by Mabel Zhang) is not copied: `tethys.sdf.in`
  includes it from Gazebo Fuel, where it is also under CC-BY-4.0. The meshes of both models are downloaded
  from Fuel the first time; they are not in this repository.
* `models/marine_waves/` comes from the **Waves** model of Gazebo Fuel (`OpenRobotics/Waves`, version 5,
  "visual only"): its two textures are under **CC-BY-4.0** ([LICENSE-CC-BY-4.0](./LICENSE-CC-BY-4.0)) and its
  two shaders carry an **Apache-2.0** notice (UUV Simulator authors, modified by Rhys Mainwaring). The
  fragment shader here is modified: the transparency is a parameter. The files are copied here.
* The plugin set of `tethys.sdf.in` follows the `auv_controls.sdf` demo of gz-sim (Apache-2.0).

The licenses are those that Gazebo Fuel shows for each model (checked 2026-11-03).
