<!--
Copyright 2026 José Miguel Guerrero
SPDX-License-Identifier: Apache-2.0
-->

# marine_waves

The shaders and textures that draw the waves of the marine playground. It is not a model but a folder of
materials: `tools/synthetic_sea.py` of the worlds package writes the mesh of the water and the visual that
uses them (with the `gz-sim-shader-param-system` plugin), and `tools/wave_node.py` gives the parameters, so
that the surface that is drawn is the wave that moves the vehicle.

| File | What it is | License |
| --- | --- | --- |
| `materials/programs/GerstnerWaves_vs_330.glsl` | Vertex shader: a sum of Gerstner waves (UUV Simulator authors, modified by Rhys Mainwaring), unchanged. With a steepness of 0 they are sinusoids | Apache-2.0 |
| `materials/programs/GerstnerWaves_fs_330.glsl` | Fragment shader (the same authors), with the transparency (`alpha`) as a parameter instead of 0.9 | Apache-2.0 |
| `materials/textures/skybox_lowres.dds`, `wave_normals.dds` | The sky that the water reflects and the bump map | CC-BY-4.0 |

They come from the **Waves** model of Gazebo Fuel (`OpenRobotics/Waves`, version 5, CC-BY-4.0, "waves - visual
only"), the one that `shader_param.sdf` of gz-sim uses. The license of the textures is in
[../../LICENSE-CC-BY-4.0](../../LICENSE-CC-BY-4.0); the shaders carry their Apache-2.0 notice.
